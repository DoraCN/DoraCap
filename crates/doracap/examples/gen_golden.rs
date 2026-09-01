//! 生成一个最小的、含三个规范消息 + 场景元数据的 `.dcap` golden 文件，
//! 并逐区段打印其字节，供跨语言实现对拍（见 docs/doracap-format.md §13）。

use doracap_core::{Recorder, Schema, SingleFileWriter, Timestamp};
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

    // Imu
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

    // PointCloud（1 点）
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

    // PoseStamped
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

    // 读回并逐区段打印
    let bytes = std::fs::read(path).unwrap();
    println!("total {} bytes", bytes.len());
    println!();
    hex_block("HEADER (magic+version+flags)", &bytes[..13]);

    let mut pos = 13;
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
    println!("DATA_OFFSET = {pos}");

    // 消息流：从 data_offset 到 footer 起点（file_len - 24）
    let msg_start = pos;
    let msg_end = bytes.len() - 24;
    let mut mp = msg_start;
    let mut idx = 0;
    while mp < msg_end {
        let chan = u16::from_le_bytes(bytes[mp..mp + 2].try_into().unwrap());
        mp += 2;
        let stamp = u64::from_le_bytes(bytes[mp..mp + 8].try_into().unwrap());
        mp += 8;
        let len = u32::from_le_bytes(bytes[mp..mp + 4].try_into().unwrap()) as usize;
        mp += 4;
        let payload = &bytes[mp..mp + len];
        let name = match chan {
            1 => "scene",
            2 => "imu",
            3 => "lidar",
            4 => "pose",
            _ => "?",
        };
        println!(
            "MESSAGE #{idx}: channel_id={chan} ({name}) stamp_ns={stamp} payload_len={len}\n  record hex: {}",
            hex(&bytes[mp - 14..mp + len])
        );
        println!("  payload hex: {}", hex(payload));
        mp += len;
        idx += 1;
    }
    println!(
        "MESSAGE DATA (start={msg_start} len={} bytes)",
        msg_end - msg_start
    );
    hex_block("MESSAGES section", &bytes[msg_start..msg_end]);

    // 尾部 footer (最后 24 字节)
    let footer = &bytes[bytes.len() - 24..];
    hex_block("FOOTER (DCAP_END + data_offset + message_count)", footer);
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
