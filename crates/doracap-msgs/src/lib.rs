//! doracap-msgs: 规范的、与来源库无关的建图/导航消息 + `rbag1` 紧凑编码。
//! 依赖 doracap-core 的 `Timestamp`（用于 Header 语义时间）但不反向引用任何领域库。

pub mod codec;
pub mod types;

pub use codec::{Codec, DecodeError};
pub use types::{Header, Imu, PointCloud, PointField, Stamped, Time};
