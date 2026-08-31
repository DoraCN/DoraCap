//! 把 `OwnedMessage` 转成一行 JSON，供 `play --json` / `--show <cmd>` 消费。
//! 不依赖 serde：对规范消息手工序列化；未知类型用 payload hex 兜底。

use doracap_core::OwnedMessage;
use doracap_msgs::{Codec, Imu, PointCloud};

/// 序列化一条消息为单行 JSON。
pub fn message_to_json(m: &OwnedMessage) -> String {
    let mut f: Vec<String> = Vec::new();
    f.push(format!("\"channel\":{}", json_str(&m.channel)));
    f.push(format!("\"stamp\":{:.9}", m.stamp.to_secs_f64()));
    f.push(format!("\"type\":{}", json_str(&m.schema.type_name)));

    match m.schema.type_name.as_str() {
        "doracap/PointCloud" => match PointCloud::decode(&m.payload) {
            Ok(pc) => {
                f.push(format!("\"frame_id\":{}", json_str(&pc.header.frame_id)));
                f.push(format!("\"height\":{}", pc.height));
                f.push(format!("\"width\":{}", pc.width));
                f.push(format!("\"point_step\":{}", pc.point_step));
                f.push(format!("\"row_step\":{}", pc.row_step));
                let pts: Vec<String> = decode_points(&pc)
                    .iter()
                    .map(|p| format!("[{:.4},{:.4},{:.4}]", p[0], p[1], p[2]))
                    .collect();
                f.push(format!("\"points\":[{}]", pts.join(",")));
            }
            Err(_) => f.push(format!("\"payload_hex\":{}", json_str(&hex(&m.payload)))),
        },
        "doracap/Imu" => match Imu::decode(&m.payload) {
            Ok(imu) => {
                f.push(format!("\"frame_id\":{}", json_str(&imu.header.frame_id)));
                f.push(format!(
                    "\"lin_acc\":[{},{},{}],\"ang_vel\":[{},{},{}]",
                    imu.lin_acc[0],
                    imu.lin_acc[1],
                    imu.lin_acc[2],
                    imu.ang_vel[0],
                    imu.ang_vel[1],
                    imu.ang_vel[2]
                ));
            }
            Err(_) => f.push(format!("\"payload_hex\":{}", json_str(&hex(&m.payload)))),
        },
        _ => f.push(format!("\"payload_hex\":{}", json_str(&hex(&m.payload)))),
    }

    format!("{{{}}}", f.join(","))
}

fn json_str(s: &str) -> String {
    format!("\"{}\"", escape(s))
}

fn escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

fn hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02x}")).collect()
}

/// 按 `fields` 的 x/y/z 偏移与 datatype 从 `data` 缓冲解析每个点。
fn decode_points(pc: &PointCloud) -> Vec<[f64; 3]> {
    let find = |name: &str| pc.fields.iter().find(|f| f.name == name);
    let (fx, fy, fz) = (find("x"), find("y"), find("z"));
    let n = pc.width as usize * pc.height as usize;
    let step = pc.point_step as usize;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let base = i * step;
        let mut p = [0.0f64; 3];
        for (slot, f) in [(0, fx), (1, fy), (2, fz)] {
            if let Some(f) = f
                && let Some(v) = read_scalar(&pc.data, base + f.offset as usize, f.datatype)
            {
                p[slot] = v;
            }
        }
        out.push(p);
    }
    out
}

fn read_scalar(data: &[u8], off: usize, datatype: u8) -> Option<f64> {
    match datatype {
        7 => data
            .get(off..off + 4)
            .and_then(|b| b.try_into().ok())
            .map(f32::from_le_bytes)
            .map(|x| x as f64),
        8 => data
            .get(off..off + 8)
            .and_then(|b| b.try_into().ok())
            .map(f64::from_le_bytes),
        _ => None,
    }
}
