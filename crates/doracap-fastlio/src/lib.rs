//! doracap-fastlio：把 FAST-LIO 的传感器流接入 doracap。
//! - 录制：任意 `fast_lio::data_source::DataSource` → `.dcap`（经 `doracap-msgs` 规范消息）。
//! - 回放：`.dcap` → `fast_lio::data_source::DataSource`，供 FAST-LIO 直接消费。

pub mod conv;

use fast_lio::data_source::{DataSource, NonBlocking};
use fast_lio::types::SensorData;
use doracap_core::{
    PlayOptions, Player, Recorder, Result, Schema, SingleFileReader, StorageWriter, Timestamp,
    TryNext,
};
use doracap_msgs::Codec;

/// 把某个数据源录制进一个 `.dcap`。
pub fn record_source<W: StorageWriter + 'static>(
    writer: W,
    source: &mut dyn DataSource,
) -> Result<()> {
    let mut rec = Recorder::new(Box::new(writer));
    rec.add_channel("imu", &schema_of::<doracap_msgs::Imu>())?;
    rec.add_channel("lidar", &schema_of::<doracap_msgs::PointCloud>())?;
    while let Some(data) = source.next() {
        let (channel, ts, buf) = conv::encode(&data);
        rec.write(channel, Timestamp::from_secs_f64(ts), &buf)?;
    }
    rec.finish()
}

/// 词句：`doracap-fastlio` 基于 `.dcap` 的 FAST-LIO `DataSource`。
pub struct BagDataSource {
    player: Player,
}

impl BagDataSource {
    pub fn open(path: impl AsRef<std::path::Path>, options: PlayOptions) -> Result<Self> {
        let reader = SingleFileReader::open(path)?;
        let player = Player::open(Box::new(reader), options)?;
        Ok(BagDataSource { player })
    }
}

impl DataSource for BagDataSource {
    fn next(&mut self) -> Option<SensorData> {
        while let Some(m) = self.player.next_message() {
            if let Some(d) = conv::decode_message(&m) {
                return Some(d);
            }
        }
        None
    }

    fn try_next(&mut self) -> std::result::Result<Option<SensorData>, NonBlocking> {
        loop {
            match self.player.try_next() {
                TryNext::Ready(m) => {
                    if let Some(d) = conv::decode_message(&m) {
                        return Ok(Some(d));
                    }
                }
                TryNext::NonBlocking => return Err(NonBlocking),
                TryNext::End => return Ok(None),
            }
        }
    }
}

fn schema_of<T: Codec>() -> Schema {
    Schema {
        id: 0,
        type_name: T::TYPE_NAME.to_string(),
        encoding: "rbag1".to_string(),
    }
}
