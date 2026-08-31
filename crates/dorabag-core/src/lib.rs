//! dorabag-core: 与具体存储/消息无关的核心值类型与存储插件接口。
//! 只依赖 std，不依赖任何领域库。

pub mod message;
pub mod play;
pub mod record;
pub mod storage;

pub use message::{ChannelMeta, Message, OwnedMessage, Result, Schema, Timestamp};
pub use play::{PlayOptions, Player, TryNext};
pub use record::Recorder;
pub use storage::{
    ChannelId, SchemaId, SingleFileReader, SingleFileWriter, StorageReader, StorageWriter,
};
