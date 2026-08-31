//! doracap 命令行（最小闭环）：record / play / info / selftest。
//! `selftest` 验证：record(Imu+PointCloud) -> 单文件 .dcap -> play -> decode -> round-trip。

use std::io::Write;
use std::process::ExitCode;
use std::process::{Command, Stdio};

use doracap_core::{
    OwnedMessage, PlayOptions, Player, Recorder, Schema, SingleFileReader, SingleFileWriter,
    StorageReader, Timestamp,
};
use doracap_msgs::{Codec, Header, Imu, PointCloud, PointField, Time};

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
        _ => {
            eprintln!("usage: doracap (selftest|info <file>|record|play)");
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
    let reader = match SingleFileReader::open(path) {
        Ok(r) => r,
        Err(e) => return fail(&e.to_string()),
    };
    let player = match Player::open(Box::new(reader), PlayOptions::default()) {
        Ok(p) => p,
        Err(e) => return fail(&e.to_string()),
    };
    println!("file: {path}");
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
    let reader = match SingleFileReader::open(path) {
        Ok(r) => r,
        Err(e) => return fail(&e.to_string()),
    };
    for c in reader.channels() {
        let schema = reader
            .schemas()
            .iter()
            .find(|s| s.id == c.schema_id)
            .map(|s| s.type_name.clone())
            .unwrap_or_default();
        println!("topic {:?} : {}", c.name, schema);
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
        assert_eq!(&bytes[bytes.len() - 24..bytes.len() - 16], b"DCAP_END");
        let _ = std::fs::remove_file(&path);
    }
}
