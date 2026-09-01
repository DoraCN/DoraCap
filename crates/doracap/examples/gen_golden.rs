//! 生成一个最小的、含三个规范消息 + 场景元数据的 `.dcap` v2 golden 文件，
//! 并逐区段打印其结构（Header / Schemas / Channels / Chunk / Footer 索引），
//! 供跨语言实现对拍（见 docs/doracap-format.md §15）。
//!
//! 注意：v2 的 chunk payload 是压缩的，因此 golden 验证的是**容器结构 + 解压后的消息布局**，
//! 而非压缩字节本身（压缩字节随压缩库/版本可能变化）。

use doracap_core::{Recorder, Schema, SingleFileReader, SingleFileWriter, StorageReader, Timestamp};
use doracap_msgs::{
    ChannelRole, Codec, Header, Imu, PointCloud, PointField, PoseStamped, SceneMeta, Time,
};

fn schema_of<T: Codec>() -> Schema {
    Schema {
        id: 0,
        type_name: T::TYPE_NAME.to_string(),
        encoding: "rbag1".to_string(),
    }
}

fn ts(sec: i64, nsec: u32) -> Time {
    Time { sec, nsec }
}

fn main() {
    let path = "/tmp/golden.dcap";
    let _ = std::fs::remove_file(path);

    let writer = SingleFileWriter::open(path).unwrap();
    let mut rec = Recorder::new(Box::new(writer));

    // 关键：SingleFileWriter 在首条消息 write 时才把 schema/channel 头写盘，
    // 因此必须先注册**所有**通道，再写任何一条消息。
    rec.add_channel("scene", &schema_of::<SceneMeta>()).unwrap();
    rec.add_channel("imu", &schema_of::<Imu>()).unwrap();
    rec.add_channel("lidar", &schema_of::<PointCloud>())
        .unwrap();
    rec.add_channel("pose", &schema_of::<PoseStamped>())
        .unwrap();

    // SceneMeta（自描述）
    let scene = SceneMeta {
        world_frame: "map".into(),
        channels: vec![
            ChannelRole {
                name: "imu".into(),
                role: "imu".into(),
                frame_id: "imu".into(),
            },
            ChannelRole {
                name: "lidar".into(),
                role: "lidar".into(),
                frame_id: "lidar".into(),
            },
            ChannelRole {
                name: "pose".into(),
                role: "pose".into(),
                frame_id: "map".into(),
            },
        ],
    };
    let mut b = Vec::new();
    scene.encode(&mut b);
    rec.write("scene", Timestamp(0), &b).unwrap();

    let imu = Imu {
        header: Header {
            stamp: ts(0, 0),
            frame_id: "imu".into(),
        },
        orientation: [0.0, 0.0, 0.0, 1.0],
        orientation_cov: [0.0; 9],
        ang_vel: [0.1, 0.2, 0.3],
        ang_vel_cov: [0.0; 9],
        lin_acc: [0.0, 0.0, 9.8],
        lin_acc_cov: [0.0; 9],
    };
    let mut b = Vec::new();
    imu.encode(&mut b);
    rec.write("imu", Timestamp::from_sec_nsec(1, 0), &b)
        .unwrap();

    let cloud = PointCloud {
        header: Header {
            stamp: ts(2, 0),
            frame_id: "lidar".into(),
        },
        height: 1,
        width: 1,
        fields: vec![
            PointField {
                name: "x".into(),
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointField {
                name: "y".into(),
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointField {
                name: "z".into(),
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ],
        is_bigendian: false,
        point_step: 12,
        row_step: 12,
        data: [1.0f32, 2.0, 3.0]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect(),
        is_dense: true,
    };
    let mut b = Vec::new();
    cloud.encode(&mut b);
    rec.write("lidar", Timestamp::from_sec_nsec(2, 0), &b)
        .unwrap();

    let pose = PoseStamped {
        header: Header {
            stamp: ts(2, 0),
            frame_id: "map".into(),
        },
        position: [1.0, 2.0, 3.0],
        orientation: [1.0, 0.0, 0.0, 0.0],
    };
    let mut b = Vec::new();
    pose.encode(&mut b);
    rec.write("pose", Timestamp::from_sec_nsec(2, 0), &b)
        .unwrap();
    rec.finish().unwrap();

    // ---------- 读回并逐区段打印 ----------
    let bytes = std::fs::read(path).unwrap();
    println!("total {} bytes", bytes.len());
    println!();

    // Header
    hex_block(
        "HEADER (magic + version + flags + compressor)",
        &bytes[..17],
    );
    let mut pos = 17;

    // Schemas
    let cnt = u32::from_le_bytes(bytes[pos..pos + 4].try_into().unwrap()) as usize;
    let schemas_start = pos;
    pos += 4;
    for _ in 0..cnt {
        let id = u16::from_le_bytes(bytes[pos..pos + 2].try_into().unwrap());
        pos += 2;
        let tn = read_str(&bytes, &mut pos);
        let enc = read_str(&bytes, &mut pos);
        println!("SCHEMA id={id} type_name={tn:?} encoding={enc:?}");
    }
    hex_block("SCHEMAS section", &bytes[schemas_start..pos]);

    // Channels
    let cnt = u32::from_le_bytes(bytes[pos..pos + 4].try_into().unwrap()) as usize;
    let chans_start = pos;
    pos += 4;
    for _ in 0..cnt {
        let id = u16::from_le_bytes(bytes[pos..pos + 2].try_into().unwrap());
        pos += 2;
        let name = read_str(&bytes, &mut pos);
        let sid = u16::from_le_bytes(bytes[pos..pos + 2].try_into().unwrap());
        pos += 2;
        println!("CHANNEL id={id} name={name:?} schema_id={sid}");
    }
    hex_block("CHANNELS section", &bytes[chans_start..pos]);
    println!("DATA_OFFSET = {pos}\n");

    // Chunks
    const CHUNK_HDR: usize = 10 + 8 + 4 + 8 + 8 + 4 + 4 + 4; // = 50
    let mut mp = pos;
    let mut cidx = 0usize;
    while mp < bytes.len() && &bytes[mp..mp + 10] == b"DCAP_CHUNK" {
        let off = mp;
        let mut q = mp + 10;
        let mb = u64::from_le_bytes(bytes[q..q + 8].try_into().unwrap());
        q += 8;
        let mc = u32::from_le_bytes(bytes[q..q + 4].try_into().unwrap());
        q += 4;
        let ss = u64::from_le_bytes(bytes[q..q + 8].try_into().unwrap());
        q += 8;
        let es = u64::from_le_bytes(bytes[q..q + 8].try_into().unwrap());
        q += 8;
        let us = u32::from_le_bytes(bytes[q..q + 4].try_into().unwrap());
        q += 4;
        let cs = u32::from_le_bytes(bytes[q..q + 4].try_into().unwrap());
        q += 4;
        let crc = u32::from_le_bytes(bytes[q..q + 4].try_into().unwrap());
        q += 4;
        println!(
            "CHUNK #{cidx}: @{off} msg_begin={mb} msg_count={mc} start_ns={ss} end_ns={es} uncompressed={us}B compressed={cs}B crc={crc:08x}"
        );
        hex_block(&format!("CHUNK #{cidx} header"), &bytes[off..off + CHUNK_HDR]);
        mp = q + cs as usize;
        cidx += 1;
    }
    println!();

    // Footer 索引 + 尾标
    let trailer_start = bytes.len() - 32;
    assert_eq!(&bytes[trailer_start..trailer_start + 8], b"DCAP_END");
    let footer_data_offset =
        u64::from_le_bytes(bytes[trailer_start + 8..trailer_start + 16].try_into().unwrap());
    let chunk_count =
        u64::from_le_bytes(bytes[trailer_start + 16..trailer_start + 24].try_into().unwrap());
    let message_count =
        u64::from_le_bytes(bytes[trailer_start + 24..trailer_start + 32].try_into().unwrap());
    let index_start = trailer_start - chunk_count as usize * 28;
    let mut ip = index_start;
    println!(
        "FOOTER: data_offset={footer_data_offset} chunk_count={chunk_count} message_count={message_count}"
    );
    for i in 0..chunk_count as usize {
        let off = u64::from_le_bytes(bytes[ip..ip + 8].try_into().unwrap());
        ip += 8;
        let ss = u64::from_le_bytes(bytes[ip..ip + 8].try_into().unwrap());
        ip += 8;
        let es = u64::from_le_bytes(bytes[ip..ip + 8].try_into().unwrap());
        ip += 8;
        let mc = u32::from_le_bytes(bytes[ip..ip + 4].try_into().unwrap());
        ip += 4;
        println!("  INDEX[{i}]: offset={off} start_ns={ss} end_ns={es} msg_count={mc}");
    }
    hex_block("FOOTER INDEX + TRAILER", &bytes[index_start..]);

    // 用官方读端读回，打印消息（验证结构 + 解压后 message 布局）。
    println!("--- decoded messages (via SingleFileReader) ---");
    let mut reader = SingleFileReader::open(path).unwrap();
    for m in reader.read_all().unwrap() {
        println!(
            "MESSAGE: channel={} stamp_ns={} type={} payload_len={}",
            m.channel,
            m.stamp.0,
            m.schema.type_name,
            m.payload.len()
        );
    }
}

fn read_str(b: &[u8], p: &mut usize) -> String {
    let len = u32::from_le_bytes(b[*p..*p + 4].try_into().unwrap()) as usize;
    *p += 4;
    let s = String::from_utf8(b[*p..*p + len].to_vec()).unwrap();
    *p += len;
    s
}

fn hex_block(label: &str, b: &[u8]) {
    println!("--- {label} ({} B) ---", b.len());
    println!("{}", hex(b));
    println!();
}

fn hex(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}
