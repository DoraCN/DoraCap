//! 规范消息结构体。

use core::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Time {
    pub sec: i64,
    pub nsec: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PointCloud {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Imu {
    pub header: Header,
    pub orientation: [f64; 4],
    pub orientation_cov: [f64; 9],
    pub ang_vel: [f64; 3],
    pub ang_vel_cov: [f64; 9],
    pub lin_acc: [f64; 3],
    pub lin_acc_cov: [f64; 9],
}

/// 带头的位姿消息（几何：position + orientation[w,x,y,z]）。
///
/// 这正是“建图过程回放”所需要的**每帧位姿**：可视化工具用它与原始点云帧配合，
/// 把每一帧从机体坐标变换到世界坐标，从而累加出正在生长的地图/轨迹。
#[derive(Clone, Debug, PartialEq)]
pub struct PoseStamped {
    pub header: Header,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// 能提供语义（传感器）时间的规范消息。
///
/// doracap 顶层 `Recorder` 依赖它从消息的 `Header` 自动提取调度时间戳，
/// 而不是要求调用方显式传时间。doracap 核心仍是类型无关的，只会在
/// `doracap-msgs` 这一层通过本 trait 做“消息 → 时间”的映射。
pub trait Stamped {
    /// 返回本消息在 [`Header`] 中的语义时间（传感器时间）。
    fn time(&self) -> Time;
}

impl Stamped for PointCloud {
    fn time(&self) -> Time {
        self.header.stamp
    }
}

impl Stamped for Imu {
    fn time(&self) -> Time {
        self.header.stamp
    }
}

impl Stamped for PoseStamped {
    fn time(&self) -> Time {
        self.header.stamp
    }
}

impl fmt::Display for Time {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{:09}", self.sec, self.nsec)
    }
}
