//! 自包含的单文件 `.dcap` 容器后端（纯 std，零外部依赖）。
//!
//! 布局（全小端）：
//!   Header   : magic b"#DCAP"(5) + version u32 + flags u32
//!   Schema   : u32 count; 每条 { u16 id; str type_name; str encoding }
//!   Channel  : u32 count; 每条 { u16 id; str name; u16 schema_id }
//!   Messages : 每条 { u16 channel_id; u64 stamp; u32 len; payload }; `data_offset` 为其起点
//!   Footer   : magic b"DCAP_END"(8) + u64 data_offset + u64 message_count
//!
//! 读取时若尾部存在 Footer 则快速定位；否则线性扫描直到 `DCAP_END` 或截断尾巴。

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::Path;

use crate::message::{ChannelMeta, Error, OwnedMessage, Result, Schema, Timestamp};
use crate::storage::{ChannelId, SchemaId, StorageReader, StorageWriter};

const MAGIC: &[u8; 5] = b"#DCAP";
const VERSION: u32 = 1;
const FLAGS: u32 = 0;
const END_MAGIC: &[u8; 8] = b"DCAP_END";

fn push_str(out: &mut Vec<u8>, s: &str) {
    out.extend_from_slice(&(s.len() as u32).to_le_bytes());
    out.extend_from_slice(s.as_bytes());
}

fn read_u16(d: &[u8], pos: &mut usize) -> Result<u16> {
    if *pos + 2 > d.len() {
        return Err(Error::msg("unexpected end of file"));
    }
    let v = u16::from_le_bytes(d[*pos..*pos + 2].try_into().unwrap());
    *pos += 2;
    Ok(v)
}

fn read_u32(d: &[u8], pos: &mut usize) -> Result<u32> {
    if *pos + 4 > d.len() {
        return Err(Error::msg("unexpected end of file"));
    }
    let v = u32::from_le_bytes(d[*pos..*pos + 4].try_into().unwrap());
    *pos += 4;
    Ok(v)
}

fn read_u64(d: &[u8], pos: &mut usize) -> Result<u64> {
    if *pos + 8 > d.len() {
        return Err(Error::msg("unexpected end of file"));
    }
    let v = u64::from_le_bytes(d[*pos..*pos + 8].try_into().unwrap());
    *pos += 8;
    Ok(v)
}

fn read_str(d: &[u8], pos: &mut usize) -> Result<String> {
    let len = read_u32(d, pos)? as usize;
    if *pos + len > d.len() {
        return Err(Error::msg("unexpected end of file"));
    }
    let s = std::str::from_utf8(&d[*pos..*pos + len])
        .map_err(|_| Error::msg("bad utf8"))?
        .to_string();
    *pos += len;
    Ok(s)
}

/// 单文件 `.dcap` 写端。
pub struct SingleFileWriter {
    file: Option<BufWriter<File>>,
    schemas: Vec<Schema>,
    schema_index: HashMap<(String, String), u16>,
    channels: Vec<ChannelMeta>,
    channel_index: HashMap<String, u16>,
    pos: u64,
    data_offset: u64,
    message_count: u64,
    header_written: bool,
    finished: bool,
}

impl SingleFileWriter {
    /// 打开（创建/截断）一个 `.dcap` 文件用于写入。
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let file = File::create(path).map_err(|e| Error::msg(format!("create {:?}: {e}", path)))?;
        let mut w = BufWriter::new(file);
        w.write_all(MAGIC)
            .and_then(|_| w.write_all(&VERSION.to_le_bytes()))
            .and_then(|_| w.write_all(&FLAGS.to_le_bytes()))
            .map_err(|e| Error::msg(e.to_string()))?;
        Ok(SingleFileWriter {
            file: Some(w),
            schemas: Vec::new(),
            schema_index: HashMap::new(),
            channels: Vec::new(),
            channel_index: HashMap::new(),
            pos: (MAGIC.len() + 4 + 4) as u64,
            data_offset: 0,
            message_count: 0,
            header_written: false,
            finished: false,
        })
    }

    /// 把 schema/channel 声明段写入文件（在首条消息前调用一次）。
    fn ensure_header(&mut self) -> Result<()> {
        if self.header_written {
            return Ok(());
        }
        let mut buf = Vec::new();
        buf.extend_from_slice(&(self.schemas.len() as u32).to_le_bytes());
        for s in &self.schemas {
            buf.extend_from_slice(&s.id.to_le_bytes());
            push_str(&mut buf, &s.type_name);
            push_str(&mut buf, &s.encoding);
        }
        buf.extend_from_slice(&(self.channels.len() as u32).to_le_bytes());
        for c in &self.channels {
            buf.extend_from_slice(&c.id.to_le_bytes());
            push_str(&mut buf, &c.name);
            buf.extend_from_slice(&c.schema_id.to_le_bytes());
        }
        {
            let w = self
                .file
                .as_mut()
                .ok_or_else(|| Error::msg("writer closed"))?;
            w.write_all(&buf).map_err(|e| Error::msg(e.to_string()))?;
        }
        self.pos += buf.len() as u64;
        self.data_offset = self.pos;
        self.header_written = true;
        Ok(())
    }
}

impl StorageWriter for SingleFileWriter {
    fn add_schema(&mut self, schema: &Schema) -> Result<SchemaId> {
        let key = (schema.type_name.clone(), schema.encoding.clone());
        if let Some(&id) = self.schema_index.get(&key) {
            return Ok(SchemaId(id));
        }
        let id = (self.schemas.len() + 1) as u16;
        let mut s = schema.clone();
        s.id = id;
        self.schemas.push(s);
        self.schema_index.insert(key, id);
        Ok(SchemaId(id))
    }

    fn add_channel(&mut self, name: &str, schema_id: SchemaId) -> Result<ChannelId> {
        if let Some(&id) = self.channel_index.get(name) {
            return Ok(ChannelId(id));
        }
        let id = (self.channels.len() + 1) as u16;
        self.channels.push(ChannelMeta {
            id,
            name: name.to_string(),
            schema_id: schema_id.0,
        });
        self.channel_index.insert(name.to_string(), id);
        Ok(ChannelId(id))
    }

    fn write(&mut self, channel: ChannelId, stamp: Timestamp, payload: &[u8]) -> Result<()> {
        self.ensure_header()?;
        let mut buf = Vec::with_capacity(2 + 8 + 4 + payload.len());
        buf.extend_from_slice(&channel.0.to_le_bytes());
        buf.extend_from_slice(&stamp.0.to_le_bytes());
        buf.extend_from_slice(&(payload.len() as u32).to_le_bytes());
        buf.extend_from_slice(payload);
        {
            let w = self
                .file
                .as_mut()
                .ok_or_else(|| Error::msg("writer closed"))?;
            w.write_all(&buf).map_err(|e| Error::msg(e.to_string()))?;
        }
        self.pos += buf.len() as u64;
        self.message_count += 1;
        Ok(())
    }

    fn finish(&mut self) -> Result<()> {
        if self.finished {
            return Ok(());
        }
        self.ensure_header()?;
        let mut buf = Vec::new();
        buf.extend_from_slice(END_MAGIC);
        buf.extend_from_slice(&self.data_offset.to_le_bytes());
        buf.extend_from_slice(&self.message_count.to_le_bytes());
        {
            let w = self
                .file
                .as_mut()
                .ok_or_else(|| Error::msg("writer closed"))?;
            w.write_all(&buf)
                .and_then(|_| w.flush())
                .map_err(|e| Error::msg(e.to_string()))?;
        }
        self.finished = true;
        Ok(())
    }
}

impl Drop for SingleFileWriter {
    fn drop(&mut self) {
        if !self.finished {
            // 尽力写尾部，使文件尽量可读（不掩盖真正的 finish 错误）。
            let _ = self.finish();
        }
    }
}

/// 单文件 `.dcap` 读端。
pub struct SingleFileReader {
    schemas: Vec<Schema>,
    channels: Vec<ChannelMeta>,
    data: Vec<u8>,
    data_offset: usize,
    msg_count: Option<usize>,
}

impl SingleFileReader {
    /// 打开一个 `.dcap` 文件读取。
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let mut f = File::open(path).map_err(|e| Error::msg(format!("open {:?}: {e}", path)))?;
        let mut data = Vec::new();
        f.read_to_end(&mut data)
            .map_err(|e| Error::msg(e.to_string()))?;

        if data.len() < MAGIC.len() || &data[..MAGIC.len()] != MAGIC {
            return Err(Error::msg("not a doracap (.dcap) file"));
        }
        let mut pos = MAGIC.len();
        let version = read_u32(&data, &mut pos)?;
        if version != VERSION {
            return Err(Error::msg(format!("unsupported version {version}")));
        }
        let _flags = read_u32(&data, &mut pos)?;

        let schema_count = read_u32(&data, &mut pos)? as usize;
        let mut schemas = Vec::with_capacity(schema_count);
        for _ in 0..schema_count {
            let id = read_u16(&data, &mut pos)?;
            let type_name = read_str(&data, &mut pos)?;
            let encoding = read_str(&data, &mut pos)?;
            schemas.push(Schema {
                id,
                type_name,
                encoding,
            });
        }

        let channel_count = read_u32(&data, &mut pos)? as usize;
        let mut channels = Vec::with_capacity(channel_count);
        for _ in 0..channel_count {
            let id = read_u16(&data, &mut pos)?;
            let name = read_str(&data, &mut pos)?;
            let schema_id = read_u16(&data, &mut pos)?;
            channels.push(ChannelMeta {
                id,
                name,
                schema_id,
            });
        }
        let data_offset = pos;

        // 尾部 Footer：END_MAGIC(8) + data_offset(8) + msg_count(8)
        let footer_len = END_MAGIC.len() + 8 + 8;
        let mut msg_count = None;
        if data.len() >= footer_len && &data[data.len() - footer_len..data.len() - 16] == END_MAGIC
        {
            let mut fp = data.len() - 16;
            let _footer_offset = read_u64(&data, &mut fp)?;
            let mc = read_u64(&data, &mut fp)?;
            msg_count = Some(mc as usize);
        }

        Ok(SingleFileReader {
            schemas,
            channels,
            data,
            data_offset,
            msg_count,
        })
    }

    fn schema_by_id(&self, id: u16) -> Option<Schema> {
        self.schemas.iter().find(|s| s.id == id).cloned()
    }

    fn channel_by_id(&self, id: u16) -> Option<&ChannelMeta> {
        self.channels.iter().find(|c| c.id == id)
    }
}

impl StorageReader for SingleFileReader {
    fn schemas(&self) -> &[Schema] {
        &self.schemas
    }

    fn channels(&self) -> &[ChannelMeta] {
        &self.channels
    }

    fn read_all(&mut self) -> Result<Vec<OwnedMessage>> {
        let data = &self.data;
        let mut pos = self.data_offset;
        let mut out = Vec::new();
        let mut count = 0usize;

        while pos < data.len() {
            if let Some(mc) = self.msg_count
                && count >= mc
            {
                break;
            }
            if pos + END_MAGIC.len() <= data.len() && &data[pos..pos + END_MAGIC.len()] == END_MAGIC
            {
                break;
            }
            if pos + 2 > data.len() {
                break; // truncate
            }
            let channel_id = u16::from_le_bytes(data[pos..pos + 2].try_into().unwrap());
            pos += 2;
            if pos + 8 > data.len() {
                break;
            }
            let stamp = u64::from_le_bytes(data[pos..pos + 8].try_into().unwrap());
            pos += 8;
            if pos + 4 > data.len() {
                break;
            }
            let payload_len = u32::from_le_bytes(data[pos..pos + 4].try_into().unwrap()) as usize;
            pos += 4;
            if pos + payload_len > data.len() {
                break; // truncated tail
            }
            let payload = data[pos..pos + payload_len].to_vec();
            pos += payload_len;

            let meta = self
                .channel_by_id(channel_id)
                .ok_or_else(|| Error::msg(format!("unknown channel id {channel_id}")))?;
            let schema = self
                .schema_by_id(meta.schema_id)
                .ok_or_else(|| Error::msg(format!("unknown schema id {}", meta.schema_id)))?;
            out.push(OwnedMessage {
                channel: meta.name.clone(),
                stamp: Timestamp(stamp),
                schema,
                payload,
            });
            count += 1;
        }
        Ok(out)
    }
}
