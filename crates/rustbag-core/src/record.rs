//! 录制端门面（类型无关）：把 `(channel, stamp, payload)` 交给任意 `StorageWriter`。

use std::collections::HashMap;

use crate::message::{Error, Result, Schema, Timestamp};
use crate::storage::{ChannelId, SchemaId, StorageWriter};

/// 录制端。持有某个存储写端，负责 schema/channel 注册与按 topic 写入。
pub struct Recorder {
    writer: Box<dyn StorageWriter>,
    schemas: HashMap<String, SchemaId>,
    channels: HashMap<String, ChannelId>,
}

impl Recorder {
    /// 用给定的存储写端创建录制端。调用方负责构造 `StorageWriter`（如 `SingleFileWriter`）。
    pub fn new(writer: Box<dyn StorageWriter>) -> Self {
        Recorder {
            writer,
            schemas: HashMap::new(),
            channels: HashMap::new(),
        }
    }

    fn schema_key(schema: &Schema) -> String {
        format!("{}|{}", schema.type_name, schema.encoding)
    }

    /// 注册或复用某个 schema（按 `type_name+encoding` 去重），返回其 id。
    pub fn register_schema(&mut self, schema: &Schema) -> Result<SchemaId> {
        let key = Self::schema_key(schema);
        if let Some(&id) = self.schemas.get(&key) {
            return Ok(id);
        }
        let id = self.writer.add_schema(schema)?;
        self.schemas.insert(key, id);
        Ok(id)
    }

    /// 注册或复用某个 topic 对应的 schema + channel，返回 channel id。
    pub fn add_channel(&mut self, name: &str, schema: &Schema) -> Result<ChannelId> {
        let schema_id = self.register_schema(schema)?;
        if let Some(&id) = self.channels.get(name) {
            return Ok(id);
        }
        let id = self.writer.add_channel(name, schema_id)?;
        self.channels.insert(name.to_string(), id);
        Ok(id)
    }

    /// 写入一条消息。`payload` 由调用方（glue）负责序列化；core 不解析。
    pub fn write(&mut self, channel: &str, stamp: Timestamp, payload: &[u8]) -> Result<()> {
        let id = self
            .channels
            .get(channel)
            .ok_or_else(|| Error::msg(format!("unknown channel: {channel}")))?;
        self.writer.write(*id, stamp, payload)
    }

    /// 结束录制，把索引/尾部元数据写入存储（使其可被快速读取）。
    pub fn finish(&mut self) -> Result<()> {
        self.writer.finish()
    }
}
