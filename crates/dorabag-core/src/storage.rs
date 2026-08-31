//! 存储插件接口：写端与读端。

use crate::message::{ChannelMeta, OwnedMessage, Result, Schema, Timestamp};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SchemaId(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ChannelId(pub u16);

/// 存储写端。由具体后端（如单文件容器、MCAP）实现。
pub trait StorageWriter {
    fn add_schema(&mut self, schema: &Schema) -> Result<SchemaId>;
    fn add_channel(&mut self, name: &str, schema_id: SchemaId) -> Result<ChannelId>;
    fn write(&mut self, channel: ChannelId, stamp: Timestamp, payload: &[u8]) -> Result<()>;
    fn finish(&mut self) -> Result<()>;
}

/// 存储读端。
pub trait StorageReader {
    fn schemas(&self) -> &[Schema];
    fn channels(&self) -> &[ChannelMeta];
    /// 读出全部消息（最小闭环用；生产可按时间序/索引流式读取）。
    fn read_all(&mut self) -> Result<Vec<OwnedMessage>>;
}

pub mod singlefile;

pub use singlefile::{SingleFileReader, SingleFileWriter};
