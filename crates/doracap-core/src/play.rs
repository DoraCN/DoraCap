//! 回放端门面（类型无关）：按时间序、受 `PlayOptions` 调度地吐消息。

use std::time::{Duration, Instant};

use crate::message::{OwnedMessage, Result, Timestamp};
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

/// 回放端。持有某个存储读端，加载全部消息后按时间轴调度返回。
pub struct Player {
    #[allow(dead_code)]
    reader: Box<dyn StorageReader>,
    options: PlayOptions,
    messages: Vec<OwnedMessage>,
    pos: usize,
    start: Option<Instant>,
    first_ts: Option<Timestamp>,
}

impl Player {
    /// 打开回放。加载全部消息，并按回放选项设置调度基准。
    pub fn open(reader: Box<dyn StorageReader>, options: PlayOptions) -> Result<Self> {
        let mut reader = reader;
        let mut messages = reader.read_all()?;
        // 回放必须按时间轴有序。录制端可能因“位姿晚于传感器几毫秒”或真机多线程乱序
        // 写入文件，因此在加载时按 `stamp` 稳定排序，保证播放/seek 正确。
        messages.sort_by_key(|a| a.stamp);
        Ok(Player {
            reader,
            options,
            messages,
            pos: 0,
            start: None,
            first_ts: None,
        })
    }

    /// 当前所有消息（供 info / 查询）。
    pub fn messages(&self) -> &[OwnedMessage] {
        &self.messages
    }

    /// 首条消息的调度时间。
    pub fn first_stamp(&self) -> Option<Timestamp> {
        self.messages.first().map(|m| m.stamp)
    }

    /// 末条消息的调度时间。
    pub fn last_stamp(&self) -> Option<Timestamp> {
        self.messages.last().map(|m| m.stamp)
    }

    /// 整段回放的时间跨度（`last - first`）。
    pub fn duration(&self) -> Timestamp {
        match (self.first_stamp(), self.last_stamp()) {
            (Some(a), Some(b)) => Timestamp(b.0.saturating_sub(a.0)),
            _ => Timestamp(0),
        }
    }

    /// 把播放头跳到**首个 `stamp >= target`** 的消息。支持任意拖动。
    pub fn seek(&mut self, target: Timestamp) {
        let idx = self.messages.partition_point(|m| m.stamp < target);
        self.pos = idx.min(self.messages.len());
        // 重新锚定时间轴：让读取端把“现在”当作目标时刻，rate>0 时从该点继续节流。
        let anchor = self
            .messages
            .get(self.pos)
            .map(|m| m.stamp)
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

    /// 调整播放倍速（0 = 不限速；>0 = 实时倍率）。重新锚定“现在=当前播放头”。
    pub fn set_rate(&mut self, rate: f64) {
        self.options.rate = rate;
        let cur = self
            .messages
            .get(self.pos)
            .map(|m| m.stamp)
            .unwrap_or_else(|| self.first_ts.unwrap_or(Timestamp(0)));
        self.first_ts = Some(cur);
        self.start = Some(Instant::now());
    }

    fn init_time(&mut self) {
        if self.start.is_none() {
            self.start = Some(Instant::now());
            self.first_ts = self
                .messages
                .first()
                .map(|m| m.stamp)
                .or(Some(Timestamp(0)));
        }
    }

    /// 第 i 条消息应到达的墙钟时刻（rate <= 0 返回 None，即不节流）。
    fn due_for(&self, i: usize) -> Option<Instant> {
        if self.options.rate <= 0.0 {
            return None;
        }
        let first = self.first_ts?;
        let delta = (self.messages[i].stamp.0 as i64 - first.0 as i64) as f64;
        let ns = (delta / self.options.rate).round().max(0.0) as u64;
        self.start.map(|s| s + Duration::from_nanos(ns))
    }

    /// 阻塞取出下一条消息；rate 为 0 时不节流，rate>0 时按倍速 sleep。
    pub fn next_message(&mut self) -> Option<OwnedMessage> {
        if self.messages.is_empty() {
            return None;
        }
        if self.pos >= self.messages.len() {
            if self.options.loop_ {
                self.seek_to_start();
                return self.next_message();
            }
            return None;
        }
        self.init_time();
        if let Some(due) = self.due_for(self.pos) {
            let now = Instant::now();
            if due > now {
                std::thread::sleep(due - now);
            }
        }
        let m = self.messages[self.pos].clone();
        self.pos += 1;
        Some(m)
    }

    /// 非阻塞取出下一条消息；未到发布时刻返回 `NonBlocking`。
    pub fn try_next(&mut self) -> TryNext {
        if self.messages.is_empty() {
            return TryNext::End;
        }
        if self.pos >= self.messages.len() {
            if self.options.loop_ {
                self.seek_to_start();
                return self.try_next();
            }
            return TryNext::End;
        }
        self.init_time();
        if let Some(due) = self.due_for(self.pos)
            && Instant::now() < due
        {
            return TryNext::NonBlocking;
        }
        let m = self.messages[self.pos].clone();
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
    use crate::storage::StorageReader;

    /// 简单的内存读端，用于测排序/seek，而不依赖具体文件。
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
}
