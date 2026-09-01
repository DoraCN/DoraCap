//! 核心值类型与错误。

use std::fmt;

/// 调度时间戳，单位：纳秒（整数，单调，无浮点漂移）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Timestamp(pub u64);

impl Timestamp {
    pub const fn from_nanos(ns: u64) -> Self {
        Timestamp(ns)
    }

    /// 从秒（f64）转换：乘 1e9 四舍五入到最近的 ns。
    pub fn from_secs_f64(s: f64) -> Self {
        Timestamp((s * 1e9).round() as u64)
    }

    pub fn to_secs_f64(self) -> f64 {
        self.0 as f64 / 1e9
    }

    /// 从 (sec, nsec) 构建；sec 可能为负，但注意 u64 无法表示负时间。
    pub fn from_sec_nsec(sec: i64, nsec: u32) -> Self {
        Timestamp((sec * 1_000_000_000 + nsec as i64) as u64)
    }

    pub fn as_nanos(self) -> u64 {
        self.0
    }
}

/// 消息类型/编码描述。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Schema {
    pub id: u16,
    pub type_name: String,
    pub encoding: String,
}

/// 一个频道（主题）的元信息。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelMeta {
    pub id: u16,
    pub name: String,
    pub schema_id: u16,
}

/// 一个 chunk 在容器里的定位信息（用于按时间 seek / 流式读）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChunkIndex {
    /// chunk 头在文件中的字节偏移。
    pub offset: u64,
    /// 该 chunk 内首条消息的调度时间（纳秒）。
    pub start_stamp: u64,
    /// 该 chunk 内末条消息的调度时间（纳秒）。
    pub end_stamp: u64,
    /// 该 chunk 内的消息条数。
    pub msg_count: u32,
}

/// 游标（只读）消息视图。
#[derive(Clone, Debug)]
pub struct Message<'a> {
    pub channel: &'a str,
    pub stamp: Timestamp,
    pub schema: &'a Schema,
    pub payload: &'a [u8],
}

/// 拥有所有权的消息（读取后端返回）。
#[derive(Clone, Debug)]
pub struct OwnedMessage {
    pub channel: String,
    pub stamp: Timestamp,
    pub schema: Schema,
    pub payload: Vec<u8>,
}

#[derive(Debug)]
pub struct Error(pub String);

impl Error {
    pub fn msg(s: impl Into<String>) -> Self {
        Error(s.into())
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for Error {}

pub type Result<T> = std::result::Result<T, Error>;
