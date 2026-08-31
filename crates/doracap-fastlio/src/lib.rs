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
use doracap_msgs::{ChannelRole, Codec, Header, PoseStamped, SceneMeta};

use crate::conv::sec_nsec;

/// 把某个数据源录制进一个 `.dcap`。
pub fn record_source<W: StorageWriter + 'static>(
    writer: W,
    source: &mut dyn DataSource,
) -> Result<()> {
    let mut rec = Recorder::new(Box::new(writer));
    rec.add_channel("imu", &schema_of::<doracap_msgs::Imu>())?;
    rec.add_channel("lidar", &schema_of::<doracap_msgs::PointCloud>())?;
    // 自描述：声明世界系与通道角色，让第三方 viz 读单文件即可回放建图。
    write_scene(&mut rec, "map", false)?;
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
    /// 录制期间每帧位姿（time, position, quaternion[w,x,y,z]），用于直接导出轨迹。
    poses: Vec<(f64, [f64; 3], [f64; 4])>,
}

impl LioRecorder {
    pub fn new(writer: Box<dyn StorageWriter>, cfg: &LioConfig) -> Result<Self> {
        let mut rec = Recorder::new(writer);
        rec.add_channel("imu", &schema_of::<doracap_msgs::Imu>())?;
        rec.add_channel("lidar", &schema_of::<doracap_msgs::PointCloud>())?;
        rec.add_channel("pose", &schema_of::<PoseStamped>())?;
        // 自描述：建图回放源，声明 world_frame + 各通道角色（含 pose 通道）。
        write_scene(&mut rec, "map", true)?;
        Ok(LioRecorder {
            rec,
            mapping: LaserMapping::new(cfg),
            poses: Vec::new(),
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
            self.poses.push((res.time, [res.pos[0], res.pos[1], res.pos[2]], res.quat));
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

    pub fn mapping_mut(&mut self) -> &mut LaserMapping {
        &mut self.mapping
    }

    /// 录制期间产生的每帧位姿，可直接写成轨迹文件（无需重新回放）。
    pub fn poses(&self) -> &[(f64, [f64; 3], [f64; 4])] {
        &self.poses
    }
}

fn schema_of<T: Codec>() -> Schema {
    Schema {
        id: 0,
        type_name: T::TYPE_NAME.to_string(),
        encoding: "rbag1".to_string(),
    }
}

/// 构造一份建图场景元数据，写入 `.dcap` 使文件自描述。
fn scene_meta(world_frame: &str, has_pose: bool) -> SceneMeta {
    let mut channels = vec![
        ChannelRole {
            name: "imu".into(),
            role: "imu".into(),
            frame_id: "imu".into(),
        },
        ChannelRole {
            name: "lidar".into(),
            role: "lidar".into(),
            frame_id: "lidar".into(),
        },
    ];
    if has_pose {
        channels.push(ChannelRole {
            name: "pose".into(),
            role: "pose".into(),
            frame_id: world_frame.into(),
        });
    }
    SceneMeta {
        world_frame: world_frame.into(),
        channels,
    }
}

/// 把场景元数据作为 `doracap/SceneMeta` 通道写入 `.dcap`（单条、时间戳 0）。
fn write_scene(rec: &mut Recorder, world_frame: &str, has_pose: bool) -> Result<()> {
    rec.add_channel("scene", &schema_of::<SceneMeta>())?;
    let meta = scene_meta(world_frame, has_pose);
    let mut buf = Vec::new();
    meta.encode(&mut buf);
    rec.write("scene", Timestamp(0), &buf)
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
        let scene = msgs.iter().filter(|m| m.channel == "scene").count();
        assert!(imu > 0 && lidar > 0, "sensor channels missing");
        assert!(pose > 0, "pose channel (trajectory) missing from .dcap");
        assert_eq!(scene, 1, "scene metadata should be written once");
        // 自描述：能解码出 world_frame 与通道角色。
        let meta = msgs
            .iter()
            .find(|m| m.channel == "scene")
            .and_then(|m| doracap_msgs::SceneMeta::decode(&m.payload).ok())
            .expect("SceneMeta decodes");
        assert_eq!(meta.world_frame, "map");
        assert!(meta.channels.iter().any(|c| c.role == "pose"));

        let _ = std::fs::remove_file(&path);
    }
}
