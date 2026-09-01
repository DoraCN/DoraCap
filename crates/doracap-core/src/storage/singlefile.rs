//! 自包含的单文件 `.dcap` 容器后端（v2：chunk 化 + 压缩 + 索引）。
//!
//! 布局（全小端）：
//!   Header   : magic b"#DCAP"(5) + version u32 + flags u32 + compressor u32
//!   Schema   : u32 count; 每条 { u16 id; str type_name; str encoding }
//!   Channel  : u32 count; 每条 { u16 id; str name; u16 schema_id }
//!   Chunk*   : 每条 { CHUNK_HEADER + compressed_records }，见下
//!   Footer   : [index entries] + trailer {END_MAGIC(8)+data_offset(8)+chunk_count(8)+message_count(8)}
//!
//! Chunk Header（50 字节）：
//!   magic "DCAP_CHUNK"(10) + msg_begin u64 + msg_count u32 + start_stamp u64 +
//!   end_stamp u64 + uncompressed_size u32 + compressed_size u32 + crc32 u32
//!
//! records（解压后）= 若干消息记录：{ u16 channel_id; u64 stamp; u32 len; payload }。
//!   `compressor`：0=none，2=deflate(zlib)。1 保留给未来 zstd。
//!
//! 读取时若尾部存在 Footer 则快速定位（chunk 索引可 seek）；否则从 `data_offset`
//! 线性扫描 chunk 头直到截断（仍可读到已完整写入的 chunk）。

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::Path;

use crate::message::{ChannelMeta, ChunkIndex, Error, OwnedMessage, Result, Schema, Timestamp};
use crate::storage::{ChannelId, SchemaId, StorageReader, StorageWriter};

const MAGIC: &[u8; 5] = b"#DCAP";
const VERSION: u32 = 2;
const FLAGS: u32 = 0;
const CHUNK_MAGIC: &[u8; 10] = b"DCAP_CHUNK";
const END_MAGIC: &[u8; 8] = b"DCAP_END";

/// 压缩算法代码。
const COMPRESS_NONE: u32 = 0;
const COMPRESS_ZSTD: u32 = 1; // 保留给未来 zstd（当前未实现）。
const COMPRESS_DEFLATE: u32 = 2;

const CHUNK_HEADER_LEN: usize = 10 + 8 + 4 + 8 + 8 + 4 + 4 + 4; // = 50
const FOOTER_TRAILER_LEN: usize = 8 + 8 + 8 + 8; // = 32
const INDEX_ENTRY_LEN: usize = 8 + 8 + 8 + 4; // = 28

/// 默认 chunk 目标：未压缩 4 MiB（约 1 秒的 Livox 点云流）。
const DEFAULT_CHUNK_TARGET: usize = 4 * 1024 * 1024;

// ---------- 底层读写辅助 ----------

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

// ---------- CRC32（IEEE） ----------

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}

// ---------- 压缩 ----------

fn compress_block(code: u32, data: &[u8]) -> Result<Vec<u8>> {
    match code {
        COMPRESS_NONE => Ok(data.to_vec()),
        COMPRESS_DEFLATE => {
            let mut e = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::new(3));
            e.write_all(data).map_err(|e| Error::msg(e.to_string()))?;
            e.finish().map_err(|e| Error::msg(e.to_string()))
        }
        COMPRESS_ZSTD => Err(Error::msg("zstd compression not built in (reserved)")),
        other => Err(Error::msg(format!("unknown compressor {other}"))),
    }
}

fn decompress_block(code: u32, data: &[u8], expected_len: usize) -> Result<Vec<u8>> {
    match code {
        COMPRESS_NONE => {
            if data.len() != expected_len {
                return Err(Error::msg("none-compressed length mismatch"));
            }
            Ok(data.to_vec())
        }
        COMPRESS_DEFLATE => {
            let mut d = flate2::read::ZlibDecoder::new(data);
            let mut out = Vec::with_capacity(expected_len);
            d.read_to_end(&mut out)
                .map_err(|e| Error::msg(e.to_string()))?;
            if out.len() != expected_len {
                return Err(Error::msg("decompressed length mismatch"));
            }
            Ok(out)
        }
        COMPRESS_ZSTD => Err(Error::msg("zstd decompression not built in (reserved)")),
        other => Err(Error::msg(format!("unknown compressor {other}"))),
    }
}

// ---------- 单文件写端 ----------

/// 单文件 `.dcap` 写端（v2：chunk 化 + 压缩 + 索引）。
pub struct SingleFileWriter {
    file: Option<BufWriter<File>>,
    schemas: Vec<Schema>,
    schema_index: HashMap<(String, String), u16>,
    channels: Vec<ChannelMeta>,
    channel_index: HashMap<String, u16>,
    pos: u64,
    data_offset: u64,
    header_written: bool,
    finished: bool,

    /// 压缩算法：0=none，2=deflate。
    compressor: u32,
    /// 未压缩 chunk 目标字节，超过即 flush。
    chunk_target: usize,

    /// 正在缓冲的当前 chunk 消息。
    pending: Vec<(ChannelId, Timestamp, Vec<u8>)>,
    pending_bytes: usize,
    /// 已落盘 chunk 的索引。
    chunk_index: Vec<ChunkIndex>,
    /// 已落盘消息总数（也用于分配 msg_begin）。
    msg_ordinal: u64,
    message_count: u64,
}

impl SingleFileWriter {
    /// 打开（创建/截断）一个 `.dcap` 文件用于写入（默认 chunk + deflate 压缩）。
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let file = File::create(path).map_err(|e| Error::msg(format!("create {:?}: {e}", path)))?;
        Ok(SingleFileWriter {
            file: Some(BufWriter::new(file)),
            schemas: Vec::new(),
            schema_index: HashMap::new(),
            channels: Vec::new(),
            channel_index: HashMap::new(),
            pos: 0,
            data_offset: 0,
            header_written: false,
            finished: false,
            compressor: COMPRESS_DEFLATE,
            chunk_target: DEFAULT_CHUNK_TARGET,
            pending: Vec::new(),
            pending_bytes: 0,
            chunk_index: Vec::new(),
            msg_ordinal: 0,
            message_count: 0,
        })
    }

    /// 设置压缩算法（须在写入首条消息前调用）。`COMPRESS_*` 常量见文件头。
    pub fn compressor(mut self, code: u32) -> Self {
        debug_assert!(!self.header_written, "compressor must be set before first write");
        self.compressor = code;
        self
    }

    /// 设置 chunk 目标大小（未压缩字节，须在写入前调用）。
    pub fn chunk_target(mut self, bytes: usize) -> Self {
        debug_assert!(!self.header_written, "chunk_target must be set before first write");
        self.chunk_target = bytes.max(1);
        self
    }

    /// 把 schema/channel 声明段写入文件（在首条消息前调用一次）。
    fn ensure_header(&mut self) -> Result<()> {
        if self.header_written {
            return Ok(());
        }
        let mut buf = Vec::new();
        buf.extend_from_slice(MAGIC);
        buf.extend_from_slice(&VERSION.to_le_bytes());
        buf.extend_from_slice(&FLAGS.to_le_bytes());
        buf.extend_from_slice(&self.compressor.to_le_bytes());
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

    /// 把当前挂起的消息编码、压缩成一个 chunk 并落盘，并记录索引。
    fn flush_chunk(&mut self) -> Result<()> {
        if self.pending.is_empty() {
            return Ok(());
        }
        // 按调度时间稳定排序，保证 chunk 内时间有序。
        self.pending.sort_by_key(|m| m.1);
        let start = self.pending.first().map(|m| m.1.0).unwrap_or(0);
        let end = self.pending.last().map(|m| m.1.0).unwrap_or(0);
        let msg_count = self.pending.len() as u32;

        // 编码 records。
        let mut records = Vec::with_capacity(self.pending_bytes);
        for (ch, st, payload) in &self.pending {
            records.extend_from_slice(&ch.0.to_le_bytes());
            records.extend_from_slice(&st.0.to_le_bytes());
            records.extend_from_slice(&(payload.len() as u32).to_le_bytes());
            records.extend_from_slice(payload);
        }
        let records_size = records.len() as u32;
        let compressed = compress_block(self.compressor, &records)?;
        let compressed_size = compressed.len() as u32;
        let crc = crc32(&records);

        let offset = self.pos;
        let mut hdr = Vec::with_capacity(CHUNK_HEADER_LEN + compressed.len());
        hdr.extend_from_slice(CHUNK_MAGIC);
        hdr.extend_from_slice(&self.msg_ordinal.to_le_bytes());
        hdr.extend_from_slice(&msg_count.to_le_bytes());
        hdr.extend_from_slice(&start.to_le_bytes());
        hdr.extend_from_slice(&end.to_le_bytes());
        hdr.extend_from_slice(&records_size.to_le_bytes());
        hdr.extend_from_slice(&compressed_size.to_le_bytes());
        hdr.extend_from_slice(&crc.to_le_bytes());
        hdr.extend_from_slice(&compressed);

        {
            let w = self
                .file
                .as_mut()
                .ok_or_else(|| Error::msg("writer closed"))?;
            w.write_all(&hdr).map_err(|e| Error::msg(e.to_string()))?;
        }
        self.pos += hdr.len() as u64;
        self.chunk_index.push(ChunkIndex {
            offset,
            start_stamp: start,
            end_stamp: end,
            msg_count,
        });
        self.msg_ordinal += msg_count as u64;
        self.message_count += msg_count as u64;
        self.pending.clear();
        self.pending_bytes = 0;
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
        self.pending
            .push((channel, stamp, payload.to_vec()));
        self.pending_bytes += 2 + 8 + 4 + payload.len();
        if self.pending_bytes >= self.chunk_target {
            self.flush_chunk()?;
        }
        Ok(())
    }

    fn finish(&mut self) -> Result<()> {
        if self.finished {
            return Ok(());
        }
        self.ensure_header()?;
        self.flush_chunk()?;
        // 写 index 条目 + footer 尾标。
        let mut buf = Vec::with_capacity(
            self.chunk_index.len() * INDEX_ENTRY_LEN + FOOTER_TRAILER_LEN,
        );
        for ci in &self.chunk_index {
            buf.extend_from_slice(&ci.offset.to_le_bytes());
            buf.extend_from_slice(&ci.start_stamp.to_le_bytes());
            buf.extend_from_slice(&ci.end_stamp.to_le_bytes());
            buf.extend_from_slice(&ci.msg_count.to_le_bytes());
        }
        buf.extend_from_slice(END_MAGIC);
        buf.extend_from_slice(&self.data_offset.to_le_bytes());
        buf.extend_from_slice(&(self.chunk_index.len() as u64).to_le_bytes());
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

// ---------- 单文件读端 ----------

/// 单文件 `.dcap` 读端（兼容 v1 线性格式与 v2 chunk+压缩格式）。
pub struct SingleFileReader {
    schemas: Vec<Schema>,
    channels: Vec<ChannelMeta>,
    data: Vec<u8>,
    data_offset: usize,
    version: u32,
    compressor: u32,
    chunk_index: Vec<ChunkIndex>,
    msg_count: Option<u64>,
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
        if version != 1 && version != 2 {
            return Err(Error::msg(format!("unsupported version {version}")));
        }
        let _flags = read_u32(&data, &mut pos)?;
        // v1 头部没有 compressor 字段；v2 有。
        let compressor = if version >= 2 {
            read_u32(&data, &mut pos)?
        } else {
            COMPRESS_NONE
        };

        // Schemas + Channels 段（v1 / v2 结构一致）。
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

        // 读取 chunk / footer 索引。
        let mut chunk_index = Vec::new();
        let mut msg_count = None;
        if version == 2 {
            // Footer trailer 在文件末尾：END_MAGIC(8) + data_offset(8) + chunk_count(8) + msg_count(8)
            if data.len() >= FOOTER_TRAILER_LEN
                && &data[data.len() - FOOTER_TRAILER_LEN..data.len() - 24] == END_MAGIC
            {
                let mc = read_u64(&data, &mut (data.len() - 8))?;
                let cc = read_u64(&data, &mut (data.len() - 16))? as usize;
                let index_start = data.len() - FOOTER_TRAILER_LEN - cc * INDEX_ENTRY_LEN;
                let mut ip = index_start;
                for _ in 0..cc {
                    let offset = read_u64(&data, &mut ip)?;
                    let start_stamp = read_u64(&data, &mut ip)?;
                    let end_stamp = read_u64(&data, &mut ip)?;
                    let mc = read_u32(&data, &mut ip)?;
                    chunk_index.push(ChunkIndex {
                        offset,
                        start_stamp,
                        end_stamp,
                        msg_count: mc,
                    });
                }
                msg_count = Some(mc);
            }
        } else {
            // v1 footer：END_MAGIC(8) + data_offset(8) + msg_count(8) 在末尾 24 字节。
            if data.len() >= 24 && &data[data.len() - 24..data.len() - 16] == END_MAGIC {
                let _do = read_u64(&data, &mut (data.len() - 16))?;
                let mc = read_u64(&data, &mut (data.len() - 8))?;
                msg_count = Some(mc);
            }
        }

        Ok(SingleFileReader {
            schemas,
            channels,
            data,
            data_offset,
            version,
            compressor,
            chunk_index,
            msg_count,
        })
    }

    /// 找到“应从此处开始读”的 chunk 索引：最后一个 `start_stamp <= target` 的 chunk。
    /// 用于按时间 seek——只需解压这一 chunk，并在其内跳过 `stamp < target` 的消息。
    /// 若 `target` 早于全部分块，返回 0；晚于全部分块，返回最后一个 chunk。
    pub fn seek_chunk(&self, target: u64) -> Option<usize> {
        if self.chunk_index.is_empty() {
            return None;
        }
        let idx = self.chunk_index.partition_point(|c| c.start_stamp <= target);
        Some(idx.saturating_sub(1))
    }

    /// 返回第 `index` 个 chunk 的定位信息。
    pub fn chunk(&self, index: usize) -> Option<&ChunkIndex> {
        self.chunk_index.get(index)
    }

    /// 返回 footer 中记录的**总消息条数**（若文件带完整 footer 且为 v2）。
    pub fn message_count(&self) -> Option<u64> {
        self.msg_count
    }

    /// 返回 (最早, 最晚) 调度时间（纳秒）。chunk 为空时返回 None。
    pub fn time_range(&self) -> Option<(u64, u64)> {
        let first = self.chunk_index.first()?.start_stamp;
        let last = self.chunk_index.last()?.end_stamp;
        Some((first, last))
    }

    fn schema_by_id(&self, id: u16) -> Option<Schema> {
        self.schemas.iter().find(|s| s.id == id).cloned()
    }

    fn channel_by_id(&self, id: u16) -> Option<&ChannelMeta> {
        self.channels.iter().find(|c| c.id == id)
    }

    /// 从一条消息记录解码为 `OwnedMessage`（绑定 channel 名与 schema）。
    fn parse_record(&self, payload: &[u8], channel_id: u16, stamp: u64) -> Result<OwnedMessage> {
        let meta = self
            .channel_by_id(channel_id)
            .ok_or_else(|| Error::msg(format!("unknown channel id {channel_id}")))?;
        let schema = self
            .schema_by_id(meta.schema_id)
            .ok_or_else(|| Error::msg(format!("unknown schema id {}", meta.schema_id)))?;
        Ok(OwnedMessage {
            channel: meta.name.clone(),
            stamp: Timestamp(stamp),
            schema,
            payload: payload.to_vec(),
        })
    }

    /// 将一个 chunk 的 records 块（解压后）解析为消息列表。
    fn parse_records(&self, records: &[u8], expected_count: u32) -> Result<Vec<OwnedMessage>> {
        let mut pos = 0usize;
        let mut out = Vec::with_capacity(expected_count as usize);
        for _ in 0..expected_count {
            if pos + 14 > records.len() {
                return Err(Error::msg("chunk records truncated"));
            }
            let channel_id = u16::from_le_bytes(records[pos..pos + 2].try_into().unwrap());
            pos += 2;
            let stamp = u64::from_le_bytes(records[pos..pos + 8].try_into().unwrap());
            pos += 8;
            let len = u32::from_le_bytes(records[pos..pos + 4].try_into().unwrap()) as usize;
            pos += 4;
            if pos + len > records.len() {
                return Err(Error::msg("chunk records truncated"));
            }
            let payload = &records[pos..pos + len];
            pos += len;
            out.push(self.parse_record(payload, channel_id, stamp)?);
        }
        Ok(out)
    }

    /// 在 `data` 的 `offset` 处读一个 chunk：校验头、解压、校验 CRC、解析消息。
    fn read_chunk_at_offset(&self, offset: u64) -> Result<Vec<OwnedMessage>> {
        let start = offset as usize;
        if start + CHUNK_HEADER_LEN > self.data.len() {
            return Err(Error::msg("chunk header out of bounds"));
        }
        if &self.data[start..start + 10] != CHUNK_MAGIC {
            return Err(Error::msg("bad chunk magic"));
        }
        let mut pos = start + 10;
        let _msg_begin = read_u64(&self.data, &mut pos)?;
        let msg_count = read_u32(&self.data, &mut pos)?;
        let _start_stamp = read_u64(&self.data, &mut pos)?;
        let _end_stamp = read_u64(&self.data, &mut pos)?;
        let uncompressed_size = read_u32(&self.data, &mut pos)? as usize;
        let compressed_size = read_u32(&self.data, &mut pos)? as usize;
        let crc = read_u32(&self.data, &mut pos)?;
        if pos + compressed_size > self.data.len() {
            return Err(Error::msg("chunk data out of bounds"));
        }
        let block = &self.data[pos..pos + compressed_size];
        let records = decompress_block(self.compressor, block, uncompressed_size)?;
        if crc32(&records) != crc {
            return Err(Error::msg("chunk crc mismatch"));
        }
        self.parse_records(&records, msg_count)
    }

    /// 线性扫描所有 chunk（无 footer 的恢复路径），并填充内部索引。
    fn scan_v2_chunks(&mut self) -> Result<Vec<OwnedMessage>> {
        let mut out = Vec::new();
        let mut pos = self.data_offset;
        let mut count = 0usize;
        loop {
            if pos + CHUNK_HEADER_LEN > self.data.len() {
                break;
            }
            if &self.data[pos..pos + 10] != CHUNK_MAGIC {
                break;
            }
            let offset = pos as u64;
            let chunk_msgs = self.read_chunk_at_offset(offset)?;
            let msg_count = chunk_msgs.len() as u32;
            let first = chunk_msgs.first().map(|m| m.stamp.0).unwrap_or(0);
            let last = chunk_msgs.last().map(|m| m.stamp.0).unwrap_or(0);
            self.chunk_index.push(ChunkIndex {
                offset,
                start_stamp: first,
                end_stamp: last,
                msg_count,
            });
            count += chunk_msgs.len();
            // 下一个 chunk 起点 = header 起点 + header_len + compressed_size。
            let mut p = pos + 10;
            let _mb = read_u64(&self.data, &mut p)?;
            let _mc = read_u32(&self.data, &mut p)?;
            let _ss = read_u64(&self.data, &mut p)?;
            let _es = read_u64(&self.data, &mut p)?;
            let _us = read_u32(&self.data, &mut p)?;
            let cs = read_u32(&self.data, &mut p)? as usize;
            let _crc = read_u32(&self.data, &mut p)?;
            pos = p + cs;
            out.extend(chunk_msgs);
        }
        self.msg_count = Some(count as u64);
        Ok(out)
    }

    // ---------- v1（旧版线性格式）读取 ----------

    fn read_v1_all(&self) -> Result<Vec<OwnedMessage>> {
        let data = &self.data;
        let mut pos = self.data_offset;
        let mut out = Vec::new();
        let mut count = 0usize;
        while pos < data.len() {
            if let Some(mc) = self.msg_count
                && count >= mc as usize
            {
                break;
            }
            if pos + END_MAGIC.len() <= data.len() && &data[pos..pos + END_MAGIC.len()] == END_MAGIC
            {
                break;
            }
            if pos + 2 > data.len() {
                break;
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
                break;
            }
            let payload = &data[pos..pos + payload_len];
            pos += payload_len;
            out.push(self.parse_record(payload, channel_id, stamp)?);
            count += 1;
        }
        Ok(out)
    }
}

impl StorageReader for SingleFileReader {
    fn schemas(&self) -> &[Schema] {
        &self.schemas
    }

    fn channels(&self) -> &[ChannelMeta] {
        &self.channels
    }

    fn chunk_index(&self) -> &[ChunkIndex] {
        &self.chunk_index
    }

    fn read_chunk_at(&mut self, index: usize) -> Result<Vec<OwnedMessage>> {
        let ci = self
            .chunk_index
            .get(index)
            .ok_or_else(|| Error::msg(format!("chunk index {index} out of range")))?;
        self.read_chunk_at_offset(ci.offset)
    }

    fn read_all(&mut self) -> Result<Vec<OwnedMessage>> {
        match self.version {
            1 => self.read_v1_all(),
            _ if !self.chunk_index.is_empty() => {
                let mut out = Vec::new();
                for i in 0..self.chunk_index.len() {
                    out.extend(self.read_chunk_at(i)?);
                }
                Ok(out)
            }
            _ => self.scan_v2_chunks(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn msg(channel: &str, stamp: u64, payload: &[u8]) -> (String, Timestamp, Vec<u8>) {
        (channel.to_string(), Timestamp(stamp), payload.to_vec())
    }

    fn write_and_read(chunk_target: usize, compressor: u32) -> (Vec<OwnedMessage>, Vec<ChunkIndex>) {
        let path = std::env::temp_dir().join(format!("doracap_chunk_{compressor}_{}.dcap", std::process::id()));
        let _ = std::fs::remove_file(&path);

        let writer = SingleFileWriter::open(&path)
            .unwrap()
            .chunk_target(chunk_target)
            .compressor(compressor);
        let mut rec = crate::Recorder::new(Box::new(writer));
        rec.add_channel("a", &Schema { id: 0, type_name: "t/a".into(), encoding: "raw".into() })
            .unwrap();
        rec.add_channel("b", &Schema { id: 0, type_name: "t/b".into(), encoding: "raw".into() })
            .unwrap();

        let samples = vec![
            msg("a", 1, &[0x11; 40]),
            msg("b", 2, &[0x22; 80]),
            msg("a", 3, &[0x33; 160]),
            msg("b", 4, &[0x44; 320]),
            msg("a", 5, &[0x55; 640]),
        ];
        for (ch, st, pay) in &samples {
            rec.write(ch, *st, pay).unwrap();
        }
        rec.finish().unwrap();

        let mut reader = SingleFileReader::open(&path).unwrap();
        let msgs = reader.read_all().unwrap();
        let idx = reader.chunk_index().to_vec();
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(&bytes[bytes.len() - 32..bytes.len() - 24], b"DCAP_END");
        let _ = std::fs::remove_file(&path);
        (msgs, idx)
    }

    #[test]
    fn chunk_roundtrip_deflate() {
        let (msgs, idx) = write_and_read(200, COMPRESS_DEFLATE);
        assert_eq!(msgs.len(), 5, "all messages round-trip");
        assert!(idx.len() >= 2, "small chunk_target forces multiple chunks");
        // 按 stamp 升序，payload 一致。
        let stamps: Vec<u64> = msgs.iter().map(|m| m.stamp.0).collect();
        assert_eq!(stamps, vec![1, 2, 3, 4, 5]);
        assert_eq!(msgs[0].payload, vec![0x11; 40]);
        assert_eq!(msgs[4].payload, vec![0x55; 640]);
        // chunk 索引 start/end 覆盖各自范围。
        assert!(idx[0].start_stamp <= idx[0].end_stamp);
    }

    #[test]
    fn chunk_roundtrip_none() {
        let (msgs, _idx) = write_and_read(200, COMPRESS_NONE);
        assert_eq!(msgs.len(), 5);
        assert_eq!(msgs[1].payload, vec![0x22; 80]);
    }

    #[test]
    fn crc32_basic() {
        // 空串
        assert_eq!(crc32(b""), 0);
        // "123456789" 的 IEEE CRC32 = 0xCBF43926
        assert_eq!(crc32(b"123456789"), 0xCBF43926);
    }

    #[test]
    fn seek_chunk_finds_containing_range() {
        let path = std::env::temp_dir().join(format!("doracap_seek_{}.dcap", std::process::id()));
        let _ = std::fs::remove_file(&path);

        let writer = SingleFileWriter::open(&path).unwrap().chunk_target(160);
        let mut rec = crate::Recorder::new(Box::new(writer));
        rec.add_channel(
            "a",
            &Schema {
                id: 0,
                type_name: "t/a".into(),
                encoding: "raw".into(),
            },
        )
        .unwrap();
        // 造 8 条消息、stamp 递增、payload 递增大，迫使多 chunk。
        for i in 0..8u64 {
            rec.write("a", Timestamp(i * 100), &vec![0xAA; (i as usize + 1) * 130])
                .unwrap();
        }
        rec.finish().unwrap();

        let reader = SingleFileReader::open(&path).unwrap();
        let idx = reader.chunk_index().to_vec();
        assert!(idx.len() >= 2, "expected multiple chunks, got {}", idx.len());
        // 对每个 chunk 的中间时间，seek_chunk 应命中该 chunk（范围含 target）。
        for ci in &idx {
            let mid = ci.start_stamp + (ci.end_stamp - ci.start_stamp) / 2;
            let hit = reader.seek_chunk(mid).unwrap();
            let c = reader.chunk(hit).unwrap();
            assert!(
                c.start_stamp <= mid && mid <= c.end_stamp,
                "chunk {hit} range {:?} should contain {mid}",
                (c.start_stamp, c.end_stamp)
            );
        }
        // 目标早于所有 chunk 时返回 0。
        assert_eq!(reader.seek_chunk(0).unwrap(), 0);
        let _ = std::fs::remove_file(&path);
        // 引用 idx 以消除未使用告警（上面用到了）。
        assert!(!idx.is_empty());
    }
}
