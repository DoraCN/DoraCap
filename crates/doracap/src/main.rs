//! doracap 命令行（最小闭环）：record / play / info / selftest / bake。
//! `selftest` 验证：record(Imu+PointCloud) -> 单文件 .dcap -> play -> decode -> round-trip。

use std::io::Write;
use std::process::ExitCode;
use std::process::{Command, Stdio};

use doracap_core::{
    OwnedMessage, PlayOptions, Player, Recorder, Schema, SingleFileReader, SingleFileWriter,
    StorageReader, Timestamp,
};
use doracap_msgs::{Codec, Header, Imu, PointCloud, PointField, SceneMeta, Time};

mod json;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("selftest") => selftest(),
        Some("info") => info(args.get(2).map(String::as_str)),
        Some("record") => {
            eprintln!("record: 待实现（当前仅 selftest 闭环）");
            ExitCode::FAILURE
        }
        Some("play") => play(&args),
        Some("bake") => bake(&args),
        _ => {
            eprintln!(
                "usage: doracap (selftest|info <file>|record|play|bake <in.dcap> <out.dcap>)"
            );
            ExitCode::FAILURE
        }
    }
}

fn play(args: &[String]) -> ExitCode {
    let mut file: Option<String> = None;
    let mut rate: f64 = 0.0;
    let mut loop_ = false;
    let mut as_json = false;
    let mut show: Option<String> = None;
    let mut seek: Option<f64> = None;
    let mut seek_ratio: Option<f64> = None;

    let mut i = 2;
    while i < args.len() {
        let a = &args[i];
        if a == "--rate" {
            rate = args.get(i + 1).and_then(|v| v.parse().ok()).unwrap_or(0.0);
            i += 2;
            continue;
        }
        if let Some(v) = a.strip_prefix("--rate=") {
            rate = v.parse().unwrap_or(0.0);
            i += 1;
            continue;
        }
        if a == "--seek"
            && let Some(v) = args.get(i + 1)
        {
            seek = v.parse().ok();
            i += 2;
            continue;
        }
        if let Some(v) = a.strip_prefix("--seek=") {
            seek = v.parse().ok();
            i += 1;
            continue;
        }
        if a == "--seek-ratio"
            && let Some(v) = args.get(i + 1)
        {
            seek_ratio = v.parse().ok();
            i += 2;
            continue;
        }
        if let Some(v) = a.strip_prefix("--seek-ratio=") {
            seek_ratio = v.parse().ok();
            i += 1;
            continue;
        }
        if a == "--show"
            && let Some(v) = args.get(i + 1)
        {
            show = Some(v.clone());
            i += 2;
            continue;
        }
        if let Some(v) = a.strip_prefix("--show=") {
            show = Some(v.to_string());
            i += 1;
            continue;
        }
        match a.as_str() {
            "--loop" => loop_ = true,
            "--json" => as_json = true,
            "--paced" => rate = 1.0,
            _ if !a.starts_with("--") => file = Some(a.clone()),
            _ => {
                eprintln!("play: unknown arg {a}");
                return ExitCode::FAILURE;
            }
        }
        i += 1;
    }

    let Some(file) = file else {
        eprintln!("usage: doracap play <file> [--rate R] [--loop] [--json] [--show CMD]");
        return ExitCode::FAILURE;
    };

    let reader = match SingleFileReader::open(&file) {
        Ok(r) => r,
        Err(e) => return fail(&e.to_string()),
    };
    let mut opts = PlayOptions::default().rate(rate);
    if loop_ {
        opts = opts.looped(true);
    }
    let mut player = match Player::open(Box::new(reader), opts) {
        Ok(p) => p,
        Err(e) => return fail(&e.to_string()),
    };
    if let Some(r) = seek {
        player.seek(doracap_core::Timestamp::from_secs_f64(r));
    }
    if let Some(r) = seek_ratio {
        player.seek_ratio(r);
    }
    eprintln!(
        "play: {} msgs, range {:.3}..{:.3}s, rate={rate}",
        player.messages().len(),
        player.first_stamp().map(|t| t.to_secs_f64()).unwrap_or(0.0),
        player.last_stamp().map(|t| t.to_secs_f64()).unwrap_or(0.0)
    );

    let mut viz = spawn_viz(&show);
    while let Some(m) = player.next_message() {
        let line = if as_json {
            json::message_to_json(&m)
        } else {
            summary(&m)
        };
        if let Some(ref mut child) = viz {
            let stdin = child.stdin.as_mut();
            if let Some(stdin) = stdin {
                if writeln!(stdin, "{line}").is_err() {
                    return ExitCode::FAILURE;
                }
                let _ = stdin.flush();
            }
        } else {
            println!("{line}");
        }
    }
    if let Some(mut child) = viz {
        let _ = child.stdin.take();
        if let Err(e) = child.wait() {
            return fail(&format!("viz exited: {e}"));
        }
    }
    ExitCode::SUCCESS
}

fn summary(m: &OwnedMessage) -> String {
    format!(
        "{} stamp={:.6} type={} payload={}B",
        m.channel,
        m.stamp.to_secs_f64(),
        m.schema.type_name,
        m.payload.len()
    )
}

fn spawn_viz(cmd: &Option<String>) -> Option<std::process::Child> {
    let cmd = cmd.as_ref()?;
    #[cfg(unix)]
    {
        Command::new("sh")
            .arg("-c")
            .arg(cmd)
            .stdin(Stdio::piped())
            .spawn()
            .ok()
    }
    #[cfg(not(unix))]
    {
        Command::new(cmd).stdin(Stdio::piped()).spawn().ok()
    }
}

fn selftest() -> ExitCode {
    let path = std::env::temp_dir().join("doracap_selftest.dcap");
    let _ = std::fs::remove_file(&path);
    match roundtrip(&path) {
        Ok(()) => {
            println!("selftest OK: round-trip verified via {path:?}");
            ExitCode::SUCCESS
        }
        Err(e) => fail(&e),
    }
}

fn info(path: Option<&str>) -> ExitCode {
    let Some(path) = path else {
        eprintln!("usage: doracap info <file.dcap>");
        return ExitCode::FAILURE;
    };
    let mut reader = match SingleFileReader::open(path) {
        Ok(r) => r,
        Err(e) => return fail(&e.to_string()),
    };
    println!("file: {path}");

    // v2 走 chunk 索引：消息数 / 时间范围来自 footer，不整文件载入。
    if !reader.chunk_index().is_empty() {
        let msgs = reader
            .message_count()
            .map(|c| c as usize)
            .unwrap_or_else(|| reader.chunk_index().iter().map(|c| c.msg_count as usize).sum());
        println!("messages: {msgs}");
        if let Some((a, b)) = reader.time_range() {
            println!("range: {:.3} .. {:.3}", a as f64 / 1e9, b as f64 / 1e9);
        }
    } else {
        // v1 fallback：用 Player 全量读（旧文件，无 chunk 索引）。
        let v1_reader = match SingleFileReader::open(path) {
            Ok(r) => r,
            Err(e) => return fail(&e.to_string()),
        };
        let player = match Player::open(Box::new(v1_reader), PlayOptions::default()) {
            Ok(p) => p,
            Err(e) => return fail(&e.to_string()),
        };
        println!("messages: {}", player.messages().len());
        if let Some(first) = player.messages().first() {
            println!(
                "range: {:.3} .. {:.3}",
                first.stamp.to_secs_f64(),
                player
                    .messages()
                    .last()
                    .map(|m| m.stamp.to_secs_f64())
                    .unwrap_or(0.0)
            );
        }
    }

    for c in reader.channels() {
        let schema = reader
            .schemas()
            .iter()
            .find(|s| s.id == c.schema_id)
            .map(|s| s.type_name.clone())
            .unwrap_or_default();
        println!("topic {:?} : {}", c.name, schema);
    }
    // 场景元信息（自描述）：告诉 viz 世界系与各通道角色。
    // 优先只读首个 chunk（scene 约定为 stamp=0 的第一条）；找不到再全量。
    let scene_meta: Option<doracap_msgs::SceneMeta> = if reader.chunk_index().is_empty() {
        reader
            .read_all()
            .ok()
            .and_then(|msgs| msgs.into_iter().find(|m| m.channel == "scene"))
            .and_then(|m| SceneMeta::decode(&m.payload).ok())
    } else {
        let mut found = None;
        for i in 0..reader.chunk_index().len() {
            let msgs = match reader.read_chunk_at(i) {
                Ok(m) => m,
                Err(_) => break,
            };
            if let Some(m) = msgs.into_iter().find(|m| m.channel == "scene") {
                found = SceneMeta::decode(&m.payload).ok();
                break;
            }
        }
        found
    };
    if let Some(meta) = scene_meta {
        println!("scene: world={}", meta.world_frame);
        for ch in &meta.channels {
            println!(
                "  role {:?}: channel={:?} frame={:?}",
                ch.role, ch.name, ch.frame_id
            );
        }
    }
    ExitCode::SUCCESS
}

/// 合成样本 -> 录制到 `path` -> 回放 -> 解码 -> 与原值比对。
fn roundtrip(path: &std::path::Path) -> Result<(), String> {
    let stamp = Time { sec: 100, nsec: 0 };
    let header = Header {
        stamp,
        frame_id: "map".to_string(),
    };

    let cloud = PointCloud {
        header: header.clone(),
        height: 1,
        width: 4,
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
        row_step: 48,
        data: vec![0xAA_u8; 48],
        is_dense: true,
    };

    let imu = Imu {
        header,
        orientation: [0.0, 0.0, 0.0, 1.0],
        orientation_cov: [0.0; 9],
        ang_vel: [0.1, 0.2, 0.3],
        ang_vel_cov: [0.0; 9],
        lin_acc: [0.0, 0.0, 9.8],
        lin_acc_cov: [0.0; 9],
    };

    let writer = SingleFileWriter::open(path).map_err(|e| e.to_string())?;
    let mut rec = Recorder::new(Box::new(writer));
    rec.add_channel("lidar", &schema_of::<PointCloud>())
        .map_err(|e| e.to_string())?;
    rec.add_channel("imu", &schema_of::<Imu>())
        .map_err(|e| e.to_string())?;

    let mut buf = Vec::new();
    cloud.encode(&mut buf);
    rec.write("lidar", ts_of(&cloud.header.stamp), &buf)
        .map_err(|e| e.to_string())?;

    let mut buf = Vec::new();
    imu.encode(&mut buf);
    rec.write("imu", ts_of(&imu.header.stamp), &buf)
        .map_err(|e| e.to_string())?;
    rec.finish().map_err(|e| e.to_string())?;

    let reader = SingleFileReader::open(path).map_err(|e| e.to_string())?;
    let mut player =
        Player::open(Box::new(reader), PlayOptions::default()).map_err(|e| e.to_string())?;
    if player.messages().len() != 2 {
        return Err(format!(
            "expected 2 messages, got {}",
            player.messages().len()
        ));
    }

    let mut got_cloud: Option<PointCloud> = None;
    let mut got_imu: Option<Imu> = None;
    while let Some(m) = player.next_message() {
        match m.channel.as_str() {
            "lidar" => got_cloud = Some(PointCloud::decode(&m.payload).map_err(|e| e.to_string())?),
            "imu" => got_imu = Some(Imu::decode(&m.payload).map_err(|e| e.to_string())?),
            other => return Err(format!("unexpected channel {other}")),
        }
    }

    if got_cloud.as_ref() != Some(&cloud) {
        return Err("PointCloud round-trip mismatch".into());
    }
    if got_imu.as_ref() != Some(&imu) {
        return Err("Imu round-trip mismatch".into());
    }
    Ok(())
}

#[cfg(feature = "fastlio")]
fn bake(args: &[String]) -> ExitCode {
    use std::io::IsTerminal;
    use std::time::Instant;

    use doracap_fastlio::{BagDataSource, LioRecorder};
    use fast_lio::data_source::DataSource;
    use fast_lio::types::LidarType;

    let Some(inp) = args.get(2).cloned() else {
        return fail(
            "usage: doracap bake <in.dcap> <out.dcap> [--lidar avia|velo16|ouster|marsim] [--pcd <file>] [--pos <file>]",
        );
    };
    let Some(out) = args.get(3).cloned() else {
        return fail(
            "usage: doracap bake <in.dcap> <out.dcap> [--lidar avia|velo16|ouster|marsim] [--pcd <file>] [--pos <file>]",
        );
    };

    let lidar_type = match arg_str(args, "--lidar").as_deref() {
        Some("velo16") => LidarType::Velo16,
        Some("ouster") => LidarType::Oust64,
        Some("marsim") => LidarType::Marsim,
        _ => LidarType::Avia,
    };
    let pcd = arg_str(args, "--pcd");
    let pos = arg_str(args, "--pos");

    // 回放输入 .dcap（仅 imu/lidar，跳过已有 pose/scene），喂给 LioRecorder。
    let mut source: Box<dyn DataSource> = match BagDataSource::open(&inp, PlayOptions::default()) {
        Ok(s) => Box::new(s),
        Err(e) => return fail(&e.to_string()),
    };
    let writer = match SingleFileWriter::open(&out) {
        Ok(w) => w,
        Err(e) => return fail(&e.to_string()),
    };
    let cfg = bake_lio_cfg(lidar_type);
    let mut rec = match LioRecorder::new(Box::new(writer), &cfg) {
        Ok(r) => r,
        Err(e) => return fail(&e.to_string()),
    };

    let tty = std::io::stderr().is_terminal();
    eprintln!("[bake] replaying {inp} ...");
    let started = Instant::now();
    let mut n = 0usize;
    let mut last_ui = Instant::now();
    while let Some(data) = source.next() {
        if let Err(e) = rec.push(&data) {
            return fail(&e.to_string());
        }
        n += 1;
        let now = Instant::now();
        if now.duration_since(last_ui).as_secs_f64() >= 0.5 {
            let line = format!(
                "[bake] replayed {n} sensor msgs, {} poses ...",
                rec.poses().len()
            );
            if tty {
                eprint!("\r\x1b[2K{line}");
            } else {
                eprintln!("{line}");
            }
            last_ui = now;
        }
    }
    if tty {
        eprintln!(
            "\r\x1b[2K[bake] replayed {n} sensor msgs; baking poses (may take a while for long clips) ..."
        );
    } else {
        eprintln!(
            "[bake] replayed {n} sensor msgs; baking poses (may take a while for long clips) ..."
        );
    }
    // finish() 会等 FAST-LIO 把已回放的全部帧建完，从而 pose 通道完整。
    if let Err(e) = rec.finish() {
        return fail(&e.to_string());
    }
    if tty {
        eprintln!();
    }

    let poses = rec.poses().len();
    println!(
        "[bake] {n} sensor msgs -> {poses} poses in {:.1}s; wrote {out}",
        started.elapsed().as_secs_f64()
    );

    if let Some(p) = &pcd {
        if let Err(e) = export_pcd(p, &mut rec) {
            return fail(&e.to_string());
        }
    }
    if let Some(p) = &pos {
        if let Err(e) = export_pos(p, rec.poses()) {
            return fail(&e.to_string());
        }
    }
    ExitCode::SUCCESS
}

#[cfg(not(feature = "fastlio"))]
fn bake(_args: &[String]) -> ExitCode {
    eprintln!(
        "bake requires the `fastlio` feature: cargo run --features fastlio -p doracap -- bake <in.dcap> <out.dcap>"
    );
    ExitCode::FAILURE
}

#[cfg(feature = "fastlio")]
fn bake_lio_cfg(lidar_type: fast_lio::types::LidarType) -> fast_lio::laser_mapping::LioConfig {
    // 与 record_play_lio 保持一致：Avia 固定 n_scans=6，避免 fast-lio preprocess 越界。
    let n_scans = match lidar_type {
        fast_lio::types::LidarType::Avia => 6,
        _ => 16,
    };
    fast_lio::laser_mapping::LioConfig {
        lidar_type,
        feature_extract_enable: false,
        point_filter_num: 2,
        n_scans,
        scan_rate: 10,
        timestamp_unit: fast_lio::types::TimeUnit::Us,
        filter_size_surf: 0.5,
        filter_size_map: 0.5,
        ..Default::default()
    }
}

#[cfg(feature = "fastlio")]
fn export_pcd(path: &str, rec: &mut doracap_fastlio::LioRecorder) -> Result<(), String> {
    rec.mapping_mut().ikdtree.flatten_to_storage();
    write_pcd(path, &rec.mapping().ikdtree.pcl_storage)
}

#[cfg(feature = "fastlio")]
fn export_pos(path: &str, rows: &[(f64, [f64; 3], [f64; 4])]) -> Result<(), String> {
    use std::io::Write;
    let mut f = std::fs::File::create(path).map_err(|e| format!("create {path}: {e}"))?;
    f.write_all(b"# time qw qx qy qz pos_x pos_y pos_z\n")
        .map_err(|e| e.to_string())?;
    for (t, pos, quat) in rows {
        let line = format!(
            "{t:.6} {:.8} {:.8} {:.8} {:.8} {:.6} {:.6} {:.6}\n",
            quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]
        );
        f.write_all(line.as_bytes()).map_err(|e| e.to_string())?;
    }
    Ok(())
}

#[cfg(feature = "fastlio")]
fn write_pcd(path: &str, points: &[fast_lio::types::PointType]) -> Result<(), String> {
    use std::io::Write;
    let mut f = std::fs::File::create(path).map_err(|e| format!("create {path}: {e}"))?;
    let mut w =
        |s: String| -> Result<(), String> { f.write_all(s.as_bytes()).map_err(|e| e.to_string()) };
    w("# .PCD v0.7 - Point Cloud Data file format\n".into())?;
    w("VERSION 0.7\n".into())?;
    w("FIELDS x y z intensity\n".into())?;
    w("SIZE 4 4 4 4\n".into())?;
    w("TYPE F F F F\n".into())?;
    w("COUNT 1 1 1 1\n".into())?;
    w(format!("WIDTH {}\n", points.len()))?;
    w("HEIGHT 1\n".into())?;
    w("VIEWPOINT 0 0 0 1 0 0 0\n".into())?;
    w(format!("POINTS {}\n", points.len()))?;
    w("DATA ascii\n".into())?;
    for p in points {
        w(format!("{} {} {} {}\n", p.x, p.y, p.z, p.intensity))?;
    }
    Ok(())
}

#[cfg_attr(not(feature = "fastlio"), allow(dead_code))]
fn arg_str(args: &[String], name: &str) -> Option<String> {
    args.iter()
        .position(|a| a == name)
        .and_then(|i| args.get(i + 1))
        .cloned()
}

fn fail(msg: &str) -> ExitCode {
    eprintln!("FAILED: {msg}");
    ExitCode::FAILURE
}

fn schema_of<T: Codec>() -> Schema {
    Schema {
        id: 0,
        type_name: T::TYPE_NAME.to_string(),
        encoding: "rbag1".to_string(),
    }
}

fn ts_of(t: &Time) -> Timestamp {
    Timestamp::from_sec_nsec(t.sec, t.nsec)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_works() {
        let path =
            std::env::temp_dir().join(format!("doracap_roundtrip_{}.dcap", std::process::id()));
        let _ = std::fs::remove_file(&path);
        roundtrip(&path).expect("roundtrip should succeed");
        // 单文件且带结尾索引
        let bytes = std::fs::read(&path).expect("read .dcap");
        assert_eq!(&bytes[..5], b"#DCAP");
        // 尾部 = DCAP_END(8) + data_offset(8) + message_count(8)
        // v2 footer trailer（末尾 32 字节）：END_MAGIC(8) + data_offset(8) + chunk_count(8) + msg_count(8)
        assert_eq!(&bytes[bytes.len() - 32..bytes.len() - 24], b"DCAP_END");
        let _ = std::fs::remove_file(&path);
    }
}
