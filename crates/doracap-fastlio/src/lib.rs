//! doracap-fastlio：把 FAST-LIO 的传感器流接入 doracap。
//! - 录制：任意 `fast_lio::data_source::DataSource` → `.dcap`（经 `doracap-msgs` 规范消息）。
//! - 回放：`.dcap` → `fast_lio::data_source::DataSource`，供 FAST-LIO 直接消费。

pub mod conv;

use fast_lio::data_source::{DataSource, NonBlocking};
use fast_lio::laser_mapping::{LaserMapping, LioConfig};
use fast_lio::types::SensorData;
use doracap_core::{
    PlayOptions, Player, Recorder, Result, Schema, SingleFileReader, StorageWriter, Timestamp,
    TryNext,
};
use doracap_msgs::{Codec, Header, PoseStamped};

use crate::conv::sec_nsec;

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

/// 一边录制、一边建图的录制端。
///
/// `push` 每来一条传感器样本就把 imu/lidar 写进 `.dcap`，并同步喂给 FAST-LIO；
/// 每处理完一帧点云、`run_once()` 出一个位姿时，就以「该帧时间戳」把位姿作为
/// `doracap/PoseStamped` 通道写进**同一个** `.dcap`。
///
/// 这样产出的 `.dcap` 是**自洽的建图过程回放源**：既有原始传感器帧，又有把每一帧
/// 摆回世界系的位姿，外部可视化工具无需重跑 SLAM 即可播放/暂停/拖动“看到建图过程”。
pub struct LioRecorder {
    rec: Recorder,
    mapping: LaserMapping,
}

impl LioRecorder {
    pub fn new(writer: Box<dyn StorageWriter>, cfg: &LioConfig) -> Result<Self> {
        let mut rec = Recorder::new(writer);
        rec.add_channel("imu", &schema_of::<doracap_msgs::Imu>())?;
        rec.add_channel("lidar", &schema_of::<doracap_msgs::PointCloud>())?;
        rec.add_channel("pose", &schema_of::<PoseStamped>())?;
        Ok(LioRecorder {
            rec,
            mapping: LaserMapping::new(cfg),
        })
    }

    /// 写入一条传感器样本；若是点云帧，随后把 FAST-LIO 算出的位姿也写入 `.dcap`。
    pub fn push(&mut self, data: &SensorData) -> Result<()> {
        match data {
            SensorData::Imu(imu) => {
                let (channel, ts, buf) = conv::encode(data);
                self.rec.write(channel, Timestamp::from_secs_f64(ts), &buf)?;
                self.mapping.add_imu(imu);
            }
            SensorData::LidarStandard(s) => {
                let (channel, ts, buf) = conv::encode(data);
                self.rec.write(channel, Timestamp::from_secs_f64(ts), &buf)?;
                self.mapping.add_lidar_standard(s);
                self.emit_pose_if_ready()?;
            }
            SensorData::LidarAvia(a) => {
                let (channel, ts, buf) = conv::encode(data);
                self.rec.write(channel, Timestamp::from_secs_f64(ts), &buf)?;
                self.mapping.add_lidar_avia(a);
                self.emit_pose_if_ready()?;
            }
        }
        Ok(())
    }

    fn emit_pose_if_ready(&mut self) -> Result<()> {
        if !self.mapping.has_data() {
            return Ok(());
        }
        if let Some(res) = self.mapping.run_once() {
            let msg = PoseStamped {
                header: Header {
                    stamp: sec_nsec(res.time),
                    frame_id: "map".into(),
                },
                position: [res.pos[0], res.pos[1], res.pos[2]],
                orientation: res.quat,
            };
            let mut buf = Vec::new();
            msg.encode(&mut buf);
            self.rec.write("pose", Timestamp::from_secs_f64(res.time), &buf)?;
        }
        Ok(())
    }

    /// 结束录制，写出 `.dcap` 尾部索引。
    pub fn finish(&mut self) -> Result<()> {
        self.rec.finish()
    }

    /// 访问内部建图器（用于导出地图点等）。
    pub fn mapping(&self) -> &LaserMapping {
        &self.mapping
    }
}

fn schema_of<T: Codec>() -> Schema {
    Schema {
        id: 0,
        type_name: T::TYPE_NAME.to_string(),
        encoding: "rbag1".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use doracap_core::{SingleFileReader, SingleFileWriter, StorageReader};
    use fast_lio::data_source::{SimParams, SimSource};
    use fast_lio::types::LidarType;

    #[test]
    fn lio_recorder_bakes_pose_channel() {
        let path =
            std::env::temp_dir().join(format!("doracap_lio_{}.dcap", std::process::id()));
        let _ = std::fs::remove_file(&path);

        let params = SimParams {
            duration: 3.0,
            points_per_scan: 200,
            ..Default::default()
        };
        let mut source = SimSource::new(&params);
        let cfg = LioConfig {
            lidar_type: LidarType::Velo16,
            ..Default::default()
        };

        let writer = SingleFileWriter::open(&path).unwrap();
        let mut rec = LioRecorder::new(Box::new(writer), &cfg).unwrap();
        while let Some(data) = source.next() {
            rec.push(&data).unwrap();
        }
        rec.finish().unwrap();

        let mut reader = SingleFileReader::open(&path).unwrap();
        let msgs = reader.read_all().unwrap();
        let pose = msgs.iter().filter(|m| m.channel == "pose").count();
        let imu = msgs.iter().filter(|m| m.channel == "imu").count();
        let lidar = msgs.iter().filter(|m| m.channel == "lidar").count();
        assert!(imu > 0 && lidar > 0, "sensor channels missing");
        assert!(pose > 0, "pose channel (trajectory) missing from .dcap");

        let _ = std::fs::remove_file(&path);
    }
}
