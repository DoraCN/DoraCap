//! `fast_lio::types::SensorData` 与 `doracap_msgs` 规范消息的互转。
//! 点云统一映射为 doracap 的 `PointCloud`（用 `fields` 描述，保留逐点时间/ring/offset_time）。

use doracap_core::OwnedMessage;
use doracap_msgs::{Codec, Header, Imu, PointCloud, PointField, Time};
use fast_lio::types::{AviaMsg, AviaPointMsg, ImuRaw, SensorData, StandardMsg, StdPointMsg};

// ---------- 时间 ----------

pub fn sec_nsec(s: f64) -> Time {
    let sec = s.floor() as i64;
    let nsec = ((s - sec as f64) * 1e9).round() as u32;
    Time { sec, nsec }
}

pub fn time_to_secs(t: &Time) -> f64 {
    t.sec as f64 + t.nsec as f64 / 1e9
}

// ---------- Imu ----------

pub fn imu_to_msg(imu: &ImuRaw) -> Imu {
    Imu {
        header: Header {
            stamp: sec_nsec(imu.stamp),
            frame_id: "imu".into(),
        },
        orientation: [0.0, 0.0, 0.0, 1.0],
        orientation_cov: [0.0; 9],
        ang_vel: imu.gyr,
        ang_vel_cov: [0.0; 9],
        lin_acc: imu.acc,
        lin_acc_cov: [0.0; 9],
    }
}

pub fn imu_from_msg(m: &Imu) -> ImuRaw {
    ImuRaw {
        stamp: time_to_secs(&m.header.stamp),
        acc: m.lin_acc,
        gyr: m.ang_vel,
    }
}

// ---------- 点云字段辅助 ----------

fn field<'a>(pc: &'a PointCloud, name: &str) -> Option<&'a PointField> {
    pc.fields.iter().find(|f| f.name == name)
}

fn has_field(pc: &PointCloud, name: &str) -> bool {
    field(pc, name).is_some()
}

fn read_f32(data: &[u8], off: usize) -> Option<f32> {
    data.get(off..off + 4)
        .and_then(|b| b.try_into().ok())
        .map(f32::from_le_bytes)
}

fn read_u16(data: &[u8], off: usize) -> Option<u16> {
    data.get(off..off + 2)
        .and_then(|b| b.try_into().ok())
        .map(u16::from_le_bytes)
}

fn read_u8(data: &[u8], off: usize) -> Option<u8> {
    data.get(off..off + 1).map(|b| b[0])
}

fn read_u32(data: &[u8], off: usize) -> Option<u32> {
    data.get(off..off + 4)
        .and_then(|b| b.try_into().ok())
        .map(u32::from_le_bytes)
}

// ---------- StandardMsg（旋转雷达：x/y/z/intensity/time/ring）----------

const STD_POINT_STEP: usize = 22;

pub fn standard_to_pc(s: &StandardMsg) -> PointCloud {
    let n = s.points.len();
    let fields = vec![
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
        PointField {
            name: "intensity".into(),
            offset: 12,
            datatype: 7,
            count: 1,
        },
        PointField {
            name: "time".into(),
            offset: 16,
            datatype: 7,
            count: 1,
        },
        PointField {
            name: "ring".into(),
            offset: 20,
            datatype: 4,
            count: 1,
        },
    ];
    let mut data = Vec::with_capacity(n * STD_POINT_STEP);
    for p in &s.points {
        data.extend_from_slice(&p.x.to_le_bytes());
        data.extend_from_slice(&p.y.to_le_bytes());
        data.extend_from_slice(&p.z.to_le_bytes());
        data.extend_from_slice(&p.intensity.to_le_bytes());
        data.extend_from_slice(&p.time.to_le_bytes());
        data.extend_from_slice(&p.ring.to_le_bytes());
    }
    PointCloud {
        header: Header {
            stamp: sec_nsec(s.stamp),
            frame_id: "lidar".into(),
        },
        height: 1,
        width: n as u32,
        fields,
        is_bigendian: false,
        point_step: STD_POINT_STEP as u32,
        row_step: (STD_POINT_STEP * n) as u32,
        data,
        is_dense: true,
    }
}

pub fn pc_to_standard(pc: &PointCloud) -> Option<StandardMsg> {
    let (fx, fy, fz) = (field(pc, "x")?, field(pc, "y")?, field(pc, "z")?);
    let fi = field(pc, "intensity")?;
    let ft = field(pc, "time")?;
    let fr = field(pc, "ring")?;
    let step = pc.point_step as usize;
    let n = pc.width as usize * pc.height as usize;
    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        let b = i * step;
        points.push(StdPointMsg {
            x: read_f32(&pc.data, b + fx.offset as usize)?,
            y: read_f32(&pc.data, b + fy.offset as usize)?,
            z: read_f32(&pc.data, b + fz.offset as usize)?,
            intensity: read_f32(&pc.data, b + fi.offset as usize)?,
            time: read_f32(&pc.data, b + ft.offset as usize)?,
            ring: read_u16(&pc.data, b + fr.offset as usize)?,
        });
    }
    Some(StandardMsg {
        stamp: time_to_secs(&pc.header.stamp),
        points,
    })
}

// ---------- AviaMsg（Livox：x/y/z/reflectivity/tag/line/offset_time）----------

const AVIA_POINT_STEP: usize = 20;

pub fn avia_to_pc(a: &AviaMsg) -> PointCloud {
    let n = a.points.len();
    let fields = vec![
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
        PointField {
            name: "reflectivity".into(),
            offset: 12,
            datatype: 2,
            count: 1,
        },
        PointField {
            name: "tag".into(),
            offset: 13,
            datatype: 4,
            count: 1,
        },
        PointField {
            name: "line".into(),
            offset: 15,
            datatype: 2,
            count: 1,
        },
        PointField {
            name: "offset_time".into(),
            offset: 16,
            datatype: 6,
            count: 1,
        },
    ];
    let mut data = Vec::with_capacity(n * AVIA_POINT_STEP);
    for p in &a.points {
        data.extend_from_slice(&p.x.to_le_bytes());
        data.extend_from_slice(&p.y.to_le_bytes());
        data.extend_from_slice(&p.z.to_le_bytes());
        data.push(p.reflectivity);
        data.extend_from_slice(&p.tag.to_le_bytes());
        data.push(p.line);
        data.extend_from_slice(&p.offset_time.to_le_bytes());
    }
    PointCloud {
        header: Header {
            stamp: sec_nsec(a.stamp),
            frame_id: "lidar".into(),
        },
        height: 1,
        width: n as u32,
        fields,
        is_bigendian: false,
        point_step: AVIA_POINT_STEP as u32,
        row_step: (AVIA_POINT_STEP * n) as u32,
        data,
        is_dense: true,
    }
}

pub fn pc_to_avia(pc: &PointCloud) -> Option<AviaMsg> {
    let fx = field(pc, "x")?;
    let fy = field(pc, "y")?;
    let fz = field(pc, "z")?;
    let fr = field(pc, "reflectivity")?;
    let ft = field(pc, "tag")?;
    let fl = field(pc, "line")?;
    let fo = field(pc, "offset_time")?;
    let step = pc.point_step as usize;
    let n = pc.width as usize * pc.height as usize;
    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        let b = i * step;
        points.push(AviaPointMsg {
            x: read_f32(&pc.data, b + fx.offset as usize)?,
            y: read_f32(&pc.data, b + fy.offset as usize)?,
            z: read_f32(&pc.data, b + fz.offset as usize)?,
            reflectivity: read_u8(&pc.data, b + fr.offset as usize)?,
            tag: read_u16(&pc.data, b + ft.offset as usize)?,
            line: read_u8(&pc.data, b + fl.offset as usize)?,
            offset_time: read_u32(&pc.data, b + fo.offset as usize)?,
        });
    }
    Some(AviaMsg {
        stamp: time_to_secs(&pc.header.stamp),
        points,
    })
}

// ---------- 顶层分派 ----------

pub fn sensor_from_pc(pc: &PointCloud) -> Option<SensorData> {
    if has_field(pc, "offset_time") {
        Some(SensorData::LidarAvia(pc_to_avia(pc)?))
    } else {
        Some(SensorData::LidarStandard(pc_to_standard(pc)?))
    }
}

/// 从一条 bag 消息还原成 `SensorData`（对 imu / lidar 分派；其它通道返回 None）。
pub fn decode_message(m: &OwnedMessage) -> Option<SensorData> {
    match m.channel.as_str() {
        "imu" => Imu::decode(&m.payload)
            .ok()
            .map(|x| SensorData::Imu(imu_from_msg(&x))),
        "lidar" => PointCloud::decode(&m.payload)
            .ok()
            .and_then(|pc| sensor_from_pc(&pc)),
        _ => None,
    }
}

/// 把一条 `SensorData` 编码成 `(channel, stamp_secs, payload)`。
pub fn encode(data: &SensorData) -> (&'static str, f64, Vec<u8>) {
    match data {
        SensorData::Imu(imu) => {
            let m = imu_to_msg(imu);
            let mut buf = Vec::new();
            m.encode(&mut buf);
            ("imu", imu.stamp, buf)
        }
        SensorData::LidarStandard(s) => {
            let m = standard_to_pc(s);
            let mut buf = Vec::new();
            m.encode(&mut buf);
            ("lidar", s.stamp, buf)
        }
        SensorData::LidarAvia(a) => {
            let m = avia_to_pc(a);
            let mut buf = Vec::new();
            m.encode(&mut buf);
            ("lidar", a.stamp, buf)
        }
    }
}

// ---------- 测试（转换 round-trip）----------

#[cfg(test)]
mod tests {
    use super::*;

    fn stamp_ns(s: f64) -> i64 {
        (s * 1e9).round() as i64
    }

    #[test]
    fn imu_roundtrip() {
        let imu = ImuRaw {
            stamp: 1.25,
            acc: [0.0, 0.1, 9.8],
            gyr: [0.01, 0.02, 0.03],
        };
        let again = imu_from_msg(&imu_to_msg(&imu));
        assert_eq!(stamp_ns(again.stamp), stamp_ns(imu.stamp));
        assert_eq!(again.acc, imu.acc);
        assert_eq!(again.gyr, imu.gyr);
    }

    #[test]
    fn standard_roundtrip() {
        let s = StandardMsg {
            stamp: 2.5,
            points: vec![
                StdPointMsg {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                    intensity: 0.5,
                    time: 0.001,
                    ring: 0,
                },
                StdPointMsg {
                    x: 4.0,
                    y: 5.0,
                    z: 6.0,
                    intensity: 1.0,
                    time: 0.002,
                    ring: 1,
                },
            ],
        };
        let pc = standard_to_pc(&s);
        let back = pc_to_standard(&pc).unwrap();
        assert_eq!(stamp_ns(back.stamp), stamp_ns(s.stamp));
        assert_eq!(back.points.len(), s.points.len());
        for (p, q) in back.points.iter().zip(s.points.iter()) {
            assert_eq!(p.x, q.x);
            assert_eq!(p.y, q.y);
            assert_eq!(p.z, q.z);
            assert_eq!(p.intensity, q.intensity);
            assert_eq!(p.time, q.time);
            assert_eq!(p.ring, q.ring);
        }
    }

    #[test]
    fn avia_roundtrip() {
        let a = AviaMsg {
            stamp: 3.0,
            points: vec![
                AviaPointMsg {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                    reflectivity: 200,
                    tag: 0x40,
                    line: 1,
                    offset_time: 123,
                },
                AviaPointMsg {
                    x: 7.0,
                    y: 8.0,
                    z: 9.0,
                    reflectivity: 0,
                    tag: 0x00,
                    line: 0,
                    offset_time: 456,
                },
            ],
        };
        let pc = avia_to_pc(&a);
        // 判型：应识别为 Avia（含 offset_time）
        match sensor_from_pc(&pc) {
            Some(SensorData::LidarAvia(back)) => {
                assert_eq!(stamp_ns(back.stamp), stamp_ns(a.stamp));
                assert_eq!(back.points.len(), a.points.len());
                for (x, y) in back.points.iter().zip(a.points.iter()) {
                    assert_eq!(x.x, y.x);
                    assert_eq!(x.offset_time, y.offset_time);
                    assert_eq!(x.tag, y.tag);
                    assert_eq!(x.reflectivity, y.reflectivity);
                    assert_eq!(x.line, y.line);
                }
            }
            other => panic!("expected LidarAvia, got {other:?}"),
        }
    }
}
