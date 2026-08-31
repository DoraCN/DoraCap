//! `rbag1` 紧凑编码：全小端、无对齐、长度前缀。

use core::fmt;

use crate::types::{Header, Imu, PointCloud, PointField, Time};

#[derive(Debug)]
pub struct DecodeError(pub String);

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "decode: {}", self.0)
    }
}

impl std::error::Error for DecodeError {}

pub type DecodeResult<T> = core::result::Result<T, DecodeError>;

/// 一个规范化消息的可编解码契约。
pub trait Codec: Sized {
    const TYPE_NAME: &'static str;
    /// 用 `rbag1` 编码到 `out`。
    fn encode(&self, out: &mut Vec<u8>);
    /// 从 `rbag1` 字节解码。
    fn decode(buf: &[u8]) -> DecodeResult<Self>;
}

// ---------- 写入辅助 ----------

fn push_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn push_i64(out: &mut Vec<u8>, v: i64) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn push_f64(out: &mut Vec<u8>, v: f64) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn push_f64_arr<const N: usize>(out: &mut Vec<u8>, arr: &[f64; N]) {
    for v in arr {
        push_f64(out, *v);
    }
}

fn push_str(out: &mut Vec<u8>, s: &str) {
    push_u32(out, s.len() as u32);
    out.extend_from_slice(s.as_bytes());
}

fn push_time(out: &mut Vec<u8>, t: &Time) {
    push_i64(out, t.sec);
    push_u32(out, t.nsec);
}

fn push_header(out: &mut Vec<u8>, h: &Header) {
    push_time(out, &h.stamp);
    push_str(out, &h.frame_id);
}

fn push_pointfield(out: &mut Vec<u8>, pf: &PointField) {
    push_str(out, &pf.name);
    push_u32(out, pf.offset);
    out.push(pf.datatype);
    push_u32(out, pf.count);
}

// ---------- 读取辅助 ----------

struct Cursor<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn new(b: &'a [u8]) -> Self {
        Cursor { b, pos: 0 }
    }

    fn take(&mut self, n: usize) -> DecodeResult<&'a [u8]> {
        if self.pos + n > self.b.len() {
            return Err(DecodeError("unexpected end".into()));
        }
        let s = &self.b[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }

    fn u32(&mut self) -> DecodeResult<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }

    fn i64(&mut self) -> DecodeResult<i64> {
        Ok(i64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }

    fn f64(&mut self) -> DecodeResult<f64> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }

    fn f64_arr<const N: usize>(&mut self) -> DecodeResult<[f64; N]> {
        let mut a = [0.0f64; N];
        for x in a.iter_mut() {
            *x = self.f64()?;
        }
        Ok(a)
    }

    fn u8(&mut self) -> DecodeResult<u8> {
        Ok(self.take(1)?[0])
    }

    fn str(&mut self) -> DecodeResult<String> {
        let len = self.u32()? as usize;
        let bytes = self.take(len)?;
        String::from_utf8(bytes.to_vec()).map_err(|_| DecodeError("bad utf8".into()))
    }

    fn bytes(&mut self) -> DecodeResult<Vec<u8>> {
        let len = self.u32()? as usize;
        Ok(self.take(len)?.to_vec())
    }
}

fn dec_time(c: &mut Cursor) -> DecodeResult<Time> {
    Ok(Time {
        sec: c.i64()?,
        nsec: c.u32()?,
    })
}

fn dec_header(c: &mut Cursor) -> DecodeResult<Header> {
    Ok(Header {
        stamp: dec_time(c)?,
        frame_id: c.str()?,
    })
}

fn dec_pointfield(c: &mut Cursor) -> DecodeResult<PointField> {
    Ok(PointField {
        name: c.str()?,
        offset: c.u32()?,
        datatype: c.u8()?,
        count: c.u32()?,
    })
}

macro_rules! impl_codec_msg {
    ($ty:ty, $name:literal, $encode:expr, $decode:expr) => {
        impl Codec for $ty {
            const TYPE_NAME: &'static str = $name;
            fn encode(&self, out: &mut Vec<u8>) {
                $encode(self, out);
            }
            fn decode(buf: &[u8]) -> DecodeResult<Self> {
                $decode(buf)
            }
        }
    };
}

fn encode_pointcloud(pc: &PointCloud, out: &mut Vec<u8>) {
    push_header(out, &pc.header);
    push_u32(out, pc.height);
    push_u32(out, pc.width);
    push_u32(out, pc.fields.len() as u32);
    for f in &pc.fields {
        push_pointfield(out, f);
    }
    out.push(pc.is_bigendian as u8);
    push_u32(out, pc.point_step);
    push_u32(out, pc.row_step);
    push_u32(out, pc.data.len() as u32);
    out.extend_from_slice(&pc.data);
    out.push(pc.is_dense as u8);
}

fn decode_pointcloud(buf: &[u8]) -> DecodeResult<PointCloud> {
    let mut c = Cursor::new(buf);
    let header = dec_header(&mut c)?;
    let height = c.u32()?;
    let width = c.u32()?;
    let nfields = c.u32()? as usize;
    let mut fields = Vec::with_capacity(nfields);
    for _ in 0..nfields {
        fields.push(dec_pointfield(&mut c)?);
    }
    let is_bigendian = c.u8()? != 0;
    let point_step = c.u32()?;
    let row_step = c.u32()?;
    let data = c.bytes()?;
    let is_dense = c.u8()? != 0;
    if c.pos != c.b.len() {
        return Err(DecodeError(format!(
            "trailing bytes: {} != {}",
            c.pos,
            c.b.len()
        )));
    }
    Ok(PointCloud {
        header,
        height,
        width,
        fields,
        is_bigendian,
        point_step,
        row_step,
        data,
        is_dense,
    })
}

fn encode_imu(imu: &Imu, out: &mut Vec<u8>) {
    push_header(out, &imu.header);
    push_f64_arr(out, &imu.orientation);
    push_f64_arr(out, &imu.orientation_cov);
    push_f64_arr(out, &imu.ang_vel);
    push_f64_arr(out, &imu.ang_vel_cov);
    push_f64_arr(out, &imu.lin_acc);
    push_f64_arr(out, &imu.lin_acc_cov);
}

fn decode_imu(buf: &[u8]) -> DecodeResult<Imu> {
    let mut c = Cursor::new(buf);
    let header = dec_header(&mut c)?;
    let orientation = c.f64_arr()?;
    let orientation_cov = c.f64_arr()?;
    let ang_vel = c.f64_arr()?;
    let ang_vel_cov = c.f64_arr()?;
    let lin_acc = c.f64_arr()?;
    let lin_acc_cov = c.f64_arr()?;
    if c.pos != c.b.len() {
        return Err(DecodeError("trailing bytes".into()));
    }
    Ok(Imu {
        header,
        orientation,
        orientation_cov,
        ang_vel,
        ang_vel_cov,
        lin_acc,
        lin_acc_cov,
    })
}

impl_codec_msg!(
    PointCloud,
    "rustbag/PointCloud",
    encode_pointcloud,
    decode_pointcloud
);
impl_codec_msg!(Imu, "rustbag/Imu", encode_imu, decode_imu);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Header, Imu, PointCloud, PointField, Stamped, Time};

    fn ts(sec: i64, nsec: u32) -> Time {
        Time { sec, nsec }
    }

    fn sample_imu() -> Imu {
        Imu {
            header: Header {
                stamp: ts(0, 0),
                frame_id: "imu".into(),
            },
            orientation: [0.0; 4],
            orientation_cov: [0.0; 9],
            ang_vel: [0.0; 3],
            ang_vel_cov: [0.0; 9],
            lin_acc: [0.0; 3],
            lin_acc_cov: [0.0; 9],
        }
    }

    fn sample_pc() -> PointCloud {
        PointCloud {
            header: Header {
                stamp: ts(12, 345678901),
                frame_id: "lidar".into(),
            },
            height: 1,
            width: 2,
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
                PointField {
                    name: "intensity".into(),
                    offset: 12,
                    datatype: 7,
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step: 16,
            row_step: 32,
            data: vec![
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
                23, 24, 25, 26, 27, 28, 29, 30, 31,
            ],
            is_dense: true,
        }
    }

    #[test]
    fn imu_roundtrip() {
        let imu = sample_imu();
        let mut buf = Vec::new();
        imu.encode(&mut buf);
        let back = Imu::decode(&buf).unwrap();
        assert_eq!(imu, back);
        assert_eq!(back.time(), imu.header.stamp);
    }

    #[test]
    fn imu_golden_bytes() {
        let imu = sample_imu();
        let mut buf = Vec::new();
        imu.encode(&mut buf);
        // 冻结参照：rbag1 规则推导，len = 315 字节。
        assert_eq!(buf.len(), 315, "golden length mismatch");
        // 头部 19 字节 = sec(8) + nsec(4) + frame_id 长度/字节(4+3)。
        // 本样例 stamp=(0,0)、frame_id="imu"，故为 "0000...03000000696d75"。
        const HEAD_HEX: &str = "00000000000000000000000003000000696d75";
        let head: Vec<u8> = HEAD_HEX
            .as_bytes()
            .chunks(2)
            .map(|c| u8::from_str_radix(std::str::from_utf8(c).unwrap(), 16).unwrap())
            .collect();
        assert_eq!(&buf[..19], &head[..]);
        // 其余 296 字节（各 f64 数组）全为 0。
        assert!(buf[19..].iter().all(|&b| b == 0));
    }

    #[test]
    fn imu_bad_input() {
        assert!(Imu::decode(&[]).is_err());
        // 截断（只给 header 一部分）
        let mut buf = Vec::new();
        sample_imu().encode(&mut buf);
        let cut = &buf[..buf.len() - 1];
        assert!(Imu::decode(cut).is_err());
    }

    #[test]
    fn pointcloud_roundtrip() {
        let pc = sample_pc();
        let mut buf = Vec::new();
        pc.encode(&mut buf);
        let back = PointCloud::decode(&buf).unwrap();
        assert_eq!(pc, back);
        assert_eq!(back.time(), pc.header.stamp);
    }

    #[test]
    fn pointcloud_bad_input() {
        let mut buf = Vec::new();
        sample_pc().encode(&mut buf);
        let trailing = [buf.as_slice(), b"extra"].concat();
        assert!(PointCloud::decode(&trailing).is_err()); // trailing 字节 → 报错
        assert!(PointCloud::decode(&[]).is_err());
    }
}
