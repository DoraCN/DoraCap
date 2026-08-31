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
        let messages = reader.read_all()?;
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
