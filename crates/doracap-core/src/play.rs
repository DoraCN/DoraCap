//! 回放端门面（类型无关）：按时间序、受 `PlayOptions` 调度地吐消息。
//!
//! v2 容器（chunk 索引）采用**惰性流式**读取：按需解压目标 chunk，内存有界；
//! v1（无 chunk 索引）则回退为全量 `read_all` + 排序。两种都通过同一个 `Player` 消费。

use std::time::{Duration, Instant};

use crate::message::{ChunkIndex, OwnedMessage, Result, Timestamp};
use crate::storage::StorageReader;

/// 回放选项。
#[derive(Clone, Copy, Debug)]
pub struct PlayOptions {
    /// 播放倍速。`0.0` 表示不限速（尽量快地吐），`1.0` 表示实时。
    pub rate: f64,
    /// 是否循环播放（末尾回绕）。
    pub loop_: bool,
}

impl Default for PlayOptions {
    fn default() -> Self {
        PlayOptions {
            rate: 0.0,
            loop_: false,
        }
    }
}

impl PlayOptions {
    pub fn rate(mut self, r: f64) -> Self {
        self.rate = r;
        self
    }
    pub fn looped(mut self, l: bool) -> Self {
        self.loop_ = l;
        self
    }
}

/// `try_next` 的非阻塞三种状态。
#[derive(Debug)]
pub enum TryNext {
    Ready(OwnedMessage),
    /// 下一条尚未到发布时间（rate > 0 时）。
    NonBlocking,
    /// 已到末尾（且未开启循环）。
    End,
}

/// 回放端。持有某个存储读端，按时间轴调度返回消息。
pub struct Player {
    #[allow(dead_code)]
    reader: Box<dyn StorageReader>,
    options: PlayOptions,
    /// chunk 索引（v2），按时间 seek 用；v1 为空。
    chunks: Vec<ChunkIndex>,
    /// 每个 chunk 起始的**全局消息序号**（累计 msg_count）。
    chunk_starts: Vec<usize>,
    /// 总消息条数。
    total: usize,
    /// all = Some 表示已物化（v1 必为 Some；v2 仅在调用 `messages()` 后）。
    all: Option<Vec<OwnedMessage>>,
    /// 当前已解压的 chunk：(chunk 索引, 消息列表)。用于 v2 流式。
    loaded: Option<(usize, Vec<OwnedMessage>)>,
    pos: usize,
    start: Option<Instant>,
    first_ts: Option<Timestamp>,
}

impl Player {
    /// 打开回放。
    ///
    /// - 若读端有 chunk 索引（v2）：走惰性流式，仅按需解压 chunk，内存有界。
    /// - 若读端无 chunk 索引（v1 / 截断）：全量 `read_all` 后按 `stamp` 稳定排序。
    pub fn open(mut reader: Box<dyn StorageReader>, options: PlayOptions) -> Result<Self> {
        let chunks = reader.chunk_index().to_vec();
        let mut chunk_starts = Vec::with_capacity(chunks.len());
        let mut total = 0usize;
        for c in &chunks {
            chunk_starts.push(total);
            total += c.msg_count as usize;
        }

        let all = if chunks.is_empty() {
            let mut messages = reader.read_all()?;
            messages.sort_by_key(|m| m.stamp);
            total = messages.len();
            Some(messages)
        } else {
            None
        };

        Ok(Player {
            reader,
            options,
            chunks,
            chunk_starts,
            total,
            all,
            loaded: None,
            pos: 0,
            start: None,
            first_ts: None,
        })
    }

    /// 物化所有消息并返回（会解压全部 chunk）。大文件播放请勿调用，走 `next_message` 流式读。
    pub fn messages(&mut self) -> &[OwnedMessage] {
        if self.all.is_none() {
            let mut v = Vec::with_capacity(self.total);
            for i in 0..self.chunks.len() {
                if let Ok(mut msgs) = self.reader.read_chunk_at(i) {
                    v.append(&mut msgs);
                }
            }
            v.sort_by_key(|m| m.stamp);
            self.all = Some(v);
            self.loaded = None;
        }
        self.all.as_ref().unwrap()
    }

    /// 当前所有消息（供 info / 查询）。
    pub fn message_count(&self) -> usize {
        self.total
    }

    /// 首条消息的调度时间。
    pub fn first_stamp(&self) -> Option<Timestamp> {
        if let Some(all) = &self.all {
            return all.first().map(|m| m.stamp);
        }
        self.chunks.first().map(|c| Timestamp(c.start_stamp))
    }

    /// 末条消息的调度时间。
    pub fn last_stamp(&self) -> Option<Timestamp> {
        if let Some(all) = &self.all {
            return all.last().map(|m| m.stamp);
        }
        self.chunks.last().map(|c| Timestamp(c.end_stamp))
    }

    /// 整段回放的时间跨度（`last - first`）。
    pub fn duration(&self) -> Timestamp {
        match (self.first_stamp(), self.last_stamp()) {
            (Some(a), Some(b)) => Timestamp(b.0.saturating_sub(a.0)),
            _ => Timestamp(0),
        }
    }

    /// 取第 `pos` 条消息（克隆）。v2 惰性解压所在 chunk。
    fn message_at(&mut self, pos: usize) -> Result<OwnedMessage> {
        if let Some(all) = &self.all {
            return all
                .get(pos)
                .cloned()
                .ok_or_else(|| crate::message::Error::msg(format!("ordinal {pos} out of range")));
        }
        if self.chunks.is_empty() {
            return Err(crate::message::Error::msg("no stream data"));
        }
        let c = self.chunk_containing(pos)?;
        self.ensure_loaded(c)?;
        let local = pos - self.chunk_starts[c];
        let msgs = &self.loaded.as_ref().unwrap().1;
        msgs.get(local)
            .cloned()
            .ok_or_else(|| crate::message::Error::msg(format!("ordinal {pos} out of range")))
    }

    /// 返回第 `pos` 条消息的调度时间（纳米）。
    fn stamp_at(&mut self, pos: usize) -> Option<u64> {
        if let Some(all) = &self.all {
            return all.get(pos).map(|m| m.stamp.0);
        }
        if self.chunks.is_empty() {
            return None;
        }
        let c = self.chunk_containing(pos).ok()?;
        self.ensure_loaded(c).ok()?;
        let local = pos - self.chunk_starts[c];
        self.loaded.as_ref()?.1.get(local).map(|m| m.stamp.0)
    }

    /// 定位包含第 `pos` 条消息的 chunk 下标。
    fn chunk_containing(&self, pos: usize) -> Result<usize> {
        let c = self
            .chunk_starts
            .partition_point(|&s| s <= pos)
            .checked_sub(1)
            .ok_or_else(|| crate::message::Error::msg("pos before first chunk"))?;
        Ok(c.min(self.chunks.len() - 1))
    }

    /// 确保第 `c` 个 chunk 已解压并缓存（含防御性稳定排序）。
    fn ensure_loaded(&mut self, c: usize) -> Result<()> {
        if let Some((i, _)) = &self.loaded {
            if *i == c {
                return Ok(());
            }
        }
        let mut msgs = self.reader.read_chunk_at(c)?;
        msgs.sort_by_key(|m| m.stamp);
        self.loaded = Some((c, msgs));
        Ok(())
    }

    /// 在 v2 流式下，定位第一条 `stamp >= t` 的全局消息序号。
    fn ordinal_for_stamp(&mut self, t: u64) -> Result<usize> {
        if let Some(all) = &self.all {
            return Ok(all.partition_point(|m| m.stamp.0 < t));
        }
        if self.chunks.is_empty() {
            return Ok(0);
        }
        if self.chunks.last().unwrap().end_stamp < t {
            return Ok(self.total);
        }
        let c = self
            .chunks
            .partition_point(|ci| ci.end_stamp < t)
            .min(self.chunks.len() - 1);
        self.ensure_loaded(c)?;
        let local = self.loaded.as_ref().unwrap().1.partition_point(|m| m.stamp.0 < t);
        Ok(self.chunk_starts[c] + local)
    }

    /// 初始化时间锚点（首次调用时）。
    fn init_time(&mut self) {
        if self.start.is_none() {
            self.start = Some(Instant::now());
            self.first_ts = self
                .stamp_at(self.pos)
                .map(Timestamp)
                .or(Some(Timestamp(0)));
        }
    }

    /// 第 `pos` 条消息应到达的墙钟时刻（rate <= 0 返回 None，即不节流）。
    fn due_for(&self, stamp: u64) -> Option<Instant> {
        if self.options.rate <= 0.0 {
            return None;
        }
        let first = self.first_ts?;
        let delta = (stamp as i64 - first.0 as i64) as f64;
        let ns = (delta / self.options.rate).round().max(0.0) as u64;
        self.start.map(|s| s + Duration::from_nanos(ns))
    }

    /// 阻塞取出下一条消息；rate 为 0 时不节流，rate>0 时按倍速 sleep。
    pub fn next_message(&mut self) -> Option<OwnedMessage> {
        if self.total == 0 {
            return None;
        }
        if self.pos >= self.total {
            if self.options.loop_ {
                self.seek_to_start();
                return self.next_message();
            }
            return None;
        }
        self.init_time();
        let stamp = self.stamp_at(self.pos)?;
        if let Some(due) = self.due_for(stamp) {
            let now = Instant::now();
            if due > now {
                std::thread::sleep(due - now);
            }
        }
        let m = self.message_at(self.pos).ok()?;
        self.pos += 1;
        Some(m)
    }

    /// 非阻塞取出下一条消息；未到发布时刻返回 `NonBlocking`。
    pub fn try_next(&mut self) -> TryNext {
        if self.total == 0 {
            return TryNext::End;
        }
        if self.pos >= self.total {
            if self.options.loop_ {
                self.seek_to_start();
                return self.try_next();
            }
            return TryNext::End;
        }
        self.init_time();
        let stamp = match self.stamp_at(self.pos) {
            Some(s) => s,
            None => return TryNext::End,
        };
        if let Some(due) = self.due_for(stamp) {
            if Instant::now() < due {
                return TryNext::NonBlocking;
            }
        }
        let m = match self.message_at(self.pos) {
            Ok(m) => m,
            Err(_) => return TryNext::End,
        };
        self.pos += 1;
        TryNext::Ready(m)
    }

    /// 当前模拟时间（rate 为 0 时始终等于首条时间）。
    pub fn now(&self) -> Timestamp {
        let first = self.first_ts.unwrap_or(Timestamp(0));
        if self.options.rate <= 0.0 {
            return first;
        }
        match self.start {
            Some(s) => {
                let el = s.elapsed().as_nanos() as f64;
                Timestamp((first.0 as f64 + el * self.options.rate).round() as u64)
            }
            None => first,
        }
    }

    /// 把播放头跳到**首个 `stamp >= target`** 的消息。支持任意拖动。
    pub fn seek(&mut self, target: Timestamp) {
        let ord = self.ordinal_for_stamp(target.0).unwrap_or(self.total);
        self.pos = ord.min(self.total);
        let anchor = self
            .stamp_at(self.pos)
            .map(Timestamp)
            .unwrap_or(target);
        self.first_ts = Some(anchor);
        self.start = Some(Instant::now());
    }

    /// 按比例（0.0..=1.0）把播放头跳到时间轴上对应位置。
    pub fn seek_ratio(&mut self, ratio: f64) {
        let first = self.first_stamp().unwrap_or(Timestamp(0)).0;
        let last = self.last_stamp().unwrap_or(Timestamp(0)).0;
        let dur = last.saturating_sub(first);
        let target = first + (dur as f64 * ratio.clamp(0.0, 1.0)) as u64;
        self.seek(Timestamp(target));
    }

    /// 调整播放倍速（0 = 不限速；>0 = 实时倍率）。重新锚定"现在=当前播放头"。
    pub fn set_rate(&mut self, rate: f64) {
        self.options.rate = rate;
        let cur = self
            .stamp_at(self.pos)
            .map(Timestamp)
            .unwrap_or_else(|| self.first_ts.unwrap_or(Timestamp(0)));
        self.first_ts = Some(cur);
        self.start = Some(Instant::now());
    }

    /// 回到开头（循环 / 重新开始）。
    pub fn seek_to_start(&mut self) {
        self.pos = 0;
        self.start = None;
        self.first_ts = None;
        self.init_time();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::{ChannelMeta, OwnedMessage, Schema};
    use crate::storage::singlefile::{SingleFileReader, SingleFileWriter};
    use crate::storage::StorageReader;

    /// 简单的内存读端，用于测排序/seek，而不依赖具体文件（无 chunk 索引 → 走 v1 全量路径）。
    struct MemReader(Vec<OwnedMessage>);
    impl StorageReader for MemReader {
        fn schemas(&self) -> &[Schema] {
            &[]
        }
        fn channels(&self) -> &[ChannelMeta] {
            &[]
        }
        fn read_all(&mut self) -> Result<Vec<OwnedMessage>> {
            Ok(std::mem::take(&mut self.0))
        }
    }

    fn msg(ns: u64) -> OwnedMessage {
        OwnedMessage {
            channel: "c".into(),
            stamp: Timestamp(ns),
            schema: Schema {
                id: 0,
                type_name: "t".into(),
                encoding: "e".into(),
            },
            payload: Vec::new(),
        }
    }

    #[test]
    fn sorts_by_stamp_then_seeks() {
        let reader = MemReader(vec![msg(30), msg(10), msg(20)]);
        let mut player = Player::open(Box::new(reader), PlayOptions::default()).unwrap();

        // 打开后按时间有序
        assert_eq!(player.messages()[0].stamp, Timestamp(10));
        assert_eq!(player.messages()[2].stamp, Timestamp(30));
        assert_eq!(player.duration(), Timestamp(20));

        // 跳转到 20ns
        player.seek(Timestamp(20));
        assert_eq!(player.next_message().unwrap().stamp, Timestamp(20));
        assert_eq!(player.next_message().unwrap().stamp, Timestamp(30));
        assert!(player.next_message().is_none());

        // 按比例拖动到末尾
        player.seek_ratio(1.0);
        assert_eq!(player.next_message().unwrap().stamp, Timestamp(30));
    }

    #[test]
    fn streams_chunked_v2_with_seek() {
        let path = std::env::temp_dir().join(format!("doracap_play_v2_{}.dcap", std::process::id()));
        let _ = std::fs::remove_file(&path);

        let writer = SingleFileWriter::open(&path).unwrap().chunk_target(120);
        let mut rec = crate::Recorder::new(Box::new(writer));
        rec.add_channel(
            "c",
            &Schema {
                id: 0,
                type_name: "t/c".into(),
                encoding: "raw".into(),
            },
        )
        .unwrap();
        for i in 0..12u64 {
            rec.write("c", Timestamp(i * 10), &vec![0xBB; 60])
                .unwrap();
        }
        rec.finish().unwrap();

        let reader = SingleFileReader::open(&path).unwrap();
        let mut player = Player::open(Box::new(reader), PlayOptions::default()).unwrap();
        assert_eq!(player.message_count(), 12);
        // 全量流式：跨 chunk 时间序。
        let mut stamps = Vec::new();
        while let Some(m) = player.next_message() {
            stamps.push(m.stamp.0);
        }
        assert_eq!(stamps, (0..12).map(|i| i * 10).collect::<Vec<_>>());
        // 跳到第一条 >= 55ns（应为 60）。seek 在不同 chunk 间边界定位。
        player.seek(Timestamp(55));
        assert_eq!(player.next_message().unwrap().stamp.0, 60);
        // 按比例到末尾。
        player.seek_ratio(1.0);
        assert_eq!(player.next_message().unwrap().stamp.0, 110);
        let _ = std::fs::remove_file(&path);
    }
}
