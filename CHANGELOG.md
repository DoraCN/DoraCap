# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- P0 最小闭环：`doracap-core` 核心类型、存储 trait、单文件 `.dcap` 后端，`Recorder`/`Player`。
- `doracap-msgs`：`Header`/`Time`/`PointCloud`/`Imu` + `rbag1` 编解码（含 golden/round-trip 测试）。
- `doracap` CLI：`selftest` / `info` / `play`。
- `play` 支持 `--rate`/`--loop`/`--json`/`--show <cmd>`：实时回放并以 JSON 流输出/管道给外部 viz。
- `doracap-fastlio`：`SensorData ⇄ doracap-msgs` 胶水，把 `fast_lio::data_source::DataSource`
  录制进 `.dcap`、从 `.dcap` 回放成 `DataSource`；`glue_demo` 验证 round-trip 并能驱动 FAST-LIO 建图。
- `doracap` example `record_play_lio`：端到端验收测试（数据源 → 录制 `.dcap` → 回放 → FAST-LIO
  建图 → 导出 `map.pcd` + `pos_log.txt`）。仿真无需硬件（`--features fastlio`）；真机 Livox
  用 `--features livox`（optional 依赖 `fast-lio-driver@0.1` + `livox-sdk2`，需 cmake + C++）。
  真机路径 `--duration` 默认 0（不限时长），支持 Ctrl+C 优雅收尾后继续建图导出。
- `doracap-msgs`：新增 `PoseStamped`（header + position + orientation[w,x,y,z]）+ `rbag1` 编解码。
- `doracap-fastlio::LioRecorder`：**一遍录制**——边采集边喂 FAST-LIO，把每帧位姿作为
  `doracap/PoseStamped` 通道写进**同一个 `.dcap`**；产出的 `.dcap` 是自洽的建图过程回放源。
- `doracap-core::Player`：打开时按 `stamp` 排序；新增 `seek` / `seek_ratio` / `set_rate` /
  `duration` / `first_stamp` / `last_stamp`，支持播放/暂停/拖动。
- `doracap` CLI `play`：新增 `--seek <secs>` / `--seek-ratio <r>`，支持跳到时间轴任意位置。
- `record_play_lio`：新增 `--discover`（真机先探测 Livox 设备是否可达）与可编辑配置模板
  `configs/livox/mid360_config.json`（需把 `host_ip` 改成机器人网卡 IP）。
- `record_play_lio`：真机路径 Ctrl+C 即**停止并保存后退出**（不再自动进入耗时建图）；建图改为
  显式 `--map`，且建图阶段可再次 Ctrl+C 中断并导出已积累结果。
- `record_play_lio`/`LioRecorder`：`--map` 改为**直接从录制时那次 FAST-LIO 的内存态导出**
  `map.pcd` + `pos_log.txt`，不再重放 `.dcap`（单次完成，避免对大文件二次跑）。
- 设计文档 `docs/doracap-design.md`（§0–§17）。
- `doracap-msgs`：新增 `SceneMeta`（`world_frame` + 通道角色 `ChannelRole`）+ `rbag1` 编解码，使
  `.dcap` **单文件自描述**（声明世界系与 lidar/imu/pose 角色，第三方 viz 无需重跑 SLAM 即可回放建图）。
- `doracap-fastlio`：`LioRecorder` / `record_source` 录制时写入 `doracap/SceneMeta`；`BagDataSource`
  自动跳过该通道，round-trip 不受影响。
- `doracap`：`info` / `play --json` 展示场景元信息（world frame + 通道角色）。
- 新增 `.dcap` **公开发行格式规范** `docs/doracap-format.md`：字节级布局、读取端独立伪代码、
  写端契约、建图回放行为契约（配帧 / 四元数→旋转矩阵 / seek 重建）、第三方自检清单、跨语言
  golden 样例 `crates/doracap/examples/gen_golden.rs`。
