//! 存储插件接口：写端与读端。

use crate::message::{ChannelMeta, ChunkIndex, Error, OwnedMessage, Result, Schema, Timestamp};

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

    /// 读出全部消息。默认实现按 chunk 索引逐个解压读取；
    /// 不支持 chunk 的实现（如内存读端）可覆写为直接返回全部。
    fn read_all(&mut self) -> Result<Vec<OwnedMessage>> {
        let mut out = Vec::new();
        let n = self.chunk_index().len();
        for i in 0..n {
            out.extend(self.read_chunk_at(i)?);
        }
        Ok(out)
    }

    /// 暴露 chunk 索引，供按时间 seek / 流式 / 有界内存读取。
    /// 默认返回空（表示该读端不 chunk 化）。
    fn chunk_index(&self) -> &[ChunkIndex] {
        &[]
    }

    /// 解压并读出第 `index` 个 chunk 的全部消息。默认不支持。
    fn read_chunk_at(&mut self, _index: usize) -> Result<Vec<OwnedMessage>> {
        Err(Error::msg("chunked read unsupported by this storage reader"))
    }
}

pub mod singlefile;

pub use singlefile::{SingleFileReader, SingleFileWriter};
