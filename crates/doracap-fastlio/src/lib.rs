//! doracap-fastlio：把 FAST-LIO 的传感器流接入 doracap。
//! - 录制：任意 `fast_lio::data_source::DataSource` → `.dcap`（经 `doracap-msgs` 规范消息）。
//! - 回放：`.dcap` → `fast_lio::data_source::DataSource`，供 FAST-LIO 直接消费。

pub mod conv;

use doracap_core::{
    PlayOptions, Player, Recorder, Result, Schema, SingleFileReader, StorageWriter, Timestamp,
    TryNext,
};
use doracap_msgs::{ChannelRole, Codec, Header, PoseStamped, SceneMeta};
use fast_lio::data_source::{DataSource, NonBlocking};
use fast_lio::laser_mapping::{LaserMapping, LioConfig};
use fast_lio::types::SensorData;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::{self, JoinHandle};

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
/// `push` 立即把 imu/lidar 写入 `.dcap`（快路径，不再被建图拖慢），再把该帧交给一个
/// 后台线程跑 FAST-LIO；后台每算出一帧位姿就通过通道回传，由 `push` / `finish_*` 把位姿
/// 也写进**同一个** `.dcap`。
///
/// 这样产出的 `.dcap` 是**自洽的建图过程回放源**：既有原始传感器帧，又有把每一帧
/// 摆回世界系的位姿。录制速率与扫描率解耦：即使 FAST-LIO 比扫描慢，sensor 也不会丢帧，
/// 姿态通道覆盖“建图线程实际赶到”的部分。
pub struct LioRecorder {
    rec: Recorder,
    /// 后台建图线程归还的映射器（`finish` / `finish_now` 之后可用）。
    mapping: Option<LaserMapping>,
    /// 给后台建图线程投递 sensor 数据的通道发送端。
    map_tx: Option<Sender<SensorData>>,
    /// 用于在 `finish_now` 时让后台线程立即停止（丢弃积压）。
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<LaserMapping>>,
    pose_rx: Receiver<PoseMsg>,
    /// 录制期间每帧位姿（time, position, quaternion[w,x,y,z]），用于直接导出轨迹。
    poses: Vec<(f64, [f64; 3], [f64; 4])>,
}

/// 后台建图线程产出的位姿。
struct PoseMsg {
    time: f64,
    pos: [f64; 3],
    quat: [f64; 4],
}

impl LioRecorder {
    pub fn new(writer: Box<dyn StorageWriter>, cfg: &LioConfig) -> Result<Self> {
        let mut rec = Recorder::new(writer);
        rec.add_channel("imu", &schema_of::<doracap_msgs::Imu>())?;
        rec.add_channel("lidar", &schema_of::<doracap_msgs::PointCloud>())?;
        rec.add_channel("pose", &schema_of::<PoseStamped>())?;
        // 自描述：建图回放源，声明 world_frame + 各通道角色（含 pose 通道）。
        write_scene(&mut rec, "map", true)?;

        let (map_tx, pose_rx, worker, stop) = spawn_map_worker(LaserMapping::new(cfg));
        Ok(LioRecorder {
            rec,
            mapping: None,
            map_tx: Some(map_tx),
            stop,
            worker: Some(worker),
            pose_rx,
            poses: Vec::new(),
        })
    }

    /// 立即把一条传感器样本写入 `.dcap`，并交给后台建图线程。
    ///
    /// 写入是快路径（编码 + 落盘），不等待 FAST-LIO；建图在后台线程并行推进，避免
    /// 算法比扫描慢时拖垮录制、导致 sensor 帧在缓冲区里被丢弃。
    pub fn push(&mut self, data: &SensorData) -> Result<()> {
        let (channel, ts, buf) = conv::encode(data);
        self.rec
            .write(channel, Timestamp::from_secs_f64(ts), &buf)?;
        // 后台线程要一份数据拷贝（点云较大，但建图线程才是瓶颈，不会因此拖慢录制）。
        if let Some(tx) = &self.map_tx {
            let _ = tx.send(data.clone());
        }
        self.drain_poses()
    }

    /// 把后台线程已产出的所有位姿写入 `.dcap`。
    fn drain_poses(&mut self) -> Result<()> {
        while let Ok(p) = self.pose_rx.try_recv() {
            self.write_pose(p)?;
        }
        Ok(())
    }

    fn write_pose(&mut self, p: PoseMsg) -> Result<()> {
        let msg = PoseStamped {
            header: Header {
                stamp: sec_nsec(p.time),
                frame_id: "map".into(),
            },
            position: p.pos,
            orientation: p.quat,
        };
        let mut buf = Vec::new();
        msg.encode(&mut buf);
        self.rec
            .write("pose", Timestamp::from_secs_f64(p.time), &buf)?;
        self.poses.push((p.time, p.pos, p.quat));
        Ok(())
    }

    /// 结束录制：停掉后台线程并写出 `.dcap` 尾部索引。
    ///
    /// 会**允许后台线程把已收到的全部帧建完**（`drain=true`），使姿态通道最完整；但若
    /// 建图明显慢于扫描，`finish` 会一直等到它赶完（可能耗时较长）。
    pub fn finish(&mut self) -> Result<()> {
        self.end_mapping(true)
    }

    /// 结束录制并**立即停止**：丢弃建图线程尚未处理的积压，保住已写入的 sensor 数据。
    ///
    /// `.dcap` 中 sensor 完整，`pose` 通道只覆盖建图线程已完成的部分。用于“想尽快停下”。
    pub fn finish_now(&mut self) -> Result<()> {
        self.end_mapping(false)
    }

    fn end_mapping(&mut self, drain: bool) -> Result<()> {
        // 关闭给建图线程投递数据的管道，让它处理完（或基于 stop 立即退出）。
        if let Some(tx) = self.map_tx.take() {
            drop(tx);
        }
        if let Some(handle) = self.worker.take() {
            if !drain {
                self.stop.store(true, Ordering::Relaxed);
            }
            let mapping = handle
                .join()
                .map_err(|_| doracap_core::message::Error::msg("mapping worker panicked"))?;
            self.mapping = Some(mapping);
        }
        self.drain_poses()?;
        self.rec.finish()
    }

    /// 访问内部建图器（用于导出地图点等）。须在 `finish` / `finish_now` 之后调用。
    pub fn mapping(&self) -> &LaserMapping {
        self.mapping
            .as_ref()
            .expect("mapping is available after finish()/finish_now()")
    }

    pub fn mapping_mut(&mut self) -> &mut LaserMapping {
        self.mapping
            .as_mut()
            .expect("mapping is available after finish()/finish_now()")
    }

    /// 录制期间产生的每帧位姿，可直接写成轨迹文件（无需重新回放）。
    pub fn poses(&self) -> &[(f64, [f64; 3], [f64; 4])] {
        &self.poses
    }
}

impl Drop for LioRecorder {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        self.map_tx.take();
        if let Some(h) = self.worker.take() {
            let _ = h.join();
        }
    }
}

/// 启动后台建图线程。
fn spawn_map_worker(
    mut mapping: LaserMapping,
) -> (
    Sender<SensorData>,
    Receiver<PoseMsg>,
    JoinHandle<LaserMapping>,
    Arc<AtomicBool>,
) {
    let (tx, rx) = mpsc::channel::<SensorData>();
    let (pose_tx, pose_rx) = mpsc::channel::<PoseMsg>();
    let stop = Arc::new(AtomicBool::new(false));
    let stop2 = Arc::clone(&stop);
    let handle = thread::spawn(move || {
        while !stop2.load(Ordering::Relaxed) {
            match rx.recv() {
                Ok(data) => map_one(&mut mapping, data, &pose_tx),
                Err(_) => break,
            }
        }
        mapping
    });
    (tx, pose_rx, handle, stop)
}

fn map_one(mapping: &mut LaserMapping, data: SensorData, pose_tx: &Sender<PoseMsg>) {
    match data {
        SensorData::Imu(imu) => mapping.add_imu(&imu),
        SensorData::LidarStandard(s) => {
            mapping.add_lidar_standard(&s);
            maybe_emit_pose(mapping, pose_tx);
        }
        SensorData::LidarAvia(a) => {
            mapping.add_lidar_avia(&a);
            maybe_emit_pose(mapping, pose_tx);
        }
    }
}

fn maybe_emit_pose(mapping: &mut LaserMapping, pose_tx: &Sender<PoseMsg>) {
    if !mapping.has_data() {
        return;
    }
    if let Some(res) = mapping.run_once() {
        let _ = pose_tx.send(PoseMsg {
            time: res.time,
            pos: [res.pos[0], res.pos[1], res.pos[2]],
            quat: res.quat,
        });
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
        let path = std::env::temp_dir().join(format!("doracap_lio_{}.dcap", std::process::id()));
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
