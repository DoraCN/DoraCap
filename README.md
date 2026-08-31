# DoraCap

一个**类型无关、与具体建图/导航库解耦**的录制与回放库（Rust）。

目标：把传感器采集 + 位姿/外参等数据录进**一个 `.dcap` 单文件**；回放读取该文件后，可交给独立的
RViz 类可视化工具进行数据回放。详见 [docs/doracap-design.md](docs/doracap-design.md)。

**第三方可视化工具**只需遵守 [`.dcap` 文件格式规范](docs/doracap-format.md)，即可读取**单个
`.dcap` 文件**并按时间轴（播放 / 暂停 / 拖动 / 倍速 / 循环）逐帧回放建图过程，无需重跑 SLAM。

## 现状：P0 最小闭环

已实现并验证：

```
record(Imu + PointCloud + PoseStamped) → 单文件 .dcap → play(排序/seek) → decode
```

- `doracap-core`：核心值类型（`Timestamp`/`Schema`/`Message`）、存储 trait
  （`StorageWriter`/`StorageReader`）、单文件 `.dcap` 后端（`SingleFileWriter`/`SingleFileReader`）、
  `Recorder` / `Player`。`Player` 打开时**按时间排序**，支持 `rate`/`loop`/`now`/`seek`/`seek_ratio`
  /`set_rate`/`duration`（对外提供播放/暂停/拖动所需的时间轴 API）。
- `doracap-msgs`：规范消息 `Header`/`Time`/`PointCloud`/`Imu`/`PoseStamped` + `rbag1` 紧凑编码
  （含 golden 与 round-trip 测试）。
- `doracap`：命令行（`selftest` / `info` / `play`）。`play` 支持 `--rate`/`--loop`，可输出
  单行 JSON（`--json`）、按时间跳到任意点（`--seek <secs>` / `--seek-ratio <r>`），或实时管道给
  外部可视化进程（`--show <cmd>`）。
- `doracap` example `record_play_lio`：端到端验收测试（数据源 → 录制 `.dcap` → 回放 →
  FAST-LIO 建图 → 导出 `map.pcd` + `pos_log.txt`）。录制时**一遍完成**：边采集边喂 FAST-LIO，
  把每帧位姿作为 `doracap/PoseStamped` 通道写进**同一个 `.dcap`**，因此该 `.dcap` 已是
  **自洽的建图过程回放源**（原始帧 + 位姿），外部可视化工具无需重跑 SLAM 即可播放/暂停/拖动。
  默认仿真（`--features fastlio`，无需硬件）；
  真机 Livox 用 `--features livox`（需 cmake + C++，且与雷达同网络）。

## 快速开始

```bash
# 构建
cargo build

# 自检：录制 → 单文件 .dcap → 回放 → 解码 → round-trip
cargo run -p doracap -- selftest

# 查看某个 .dcap 的信息
cargo run -p doracap -- info <file.dcap>

# 回放（默认摘要 / JSON 流 / 管道给外部 viz）
cargo run -p doracap -- play <file.dcap> [--rate 1] [--loop] [--json]
cargo run -p doracap -- play <file.dcap> --json --show '<your-viz-cmd>'
# 跳到时间轴任意位置（“拖动/暂停点”）
cargo run -p doracap -- play <file.dcap> --seek 2.5 --json
cargo run -p doracap -- play <file.dcap> --seek-ratio 0.5

# FAST-LIO 胶水：SimSource → .dcap → 回放(round-trip) → 驱动 FAST-LIO 建图
cargo run -p doracap-fastlio --bin glue_demo          # 录制+回放 round-trip 校验
cargo run -p doracap-fastlio --bin glue_demo -- --lio  # 额外跑一次 FAST-LIO

# 端到端验收（仿真，无需硬件）：录制 .dcap → 回放 → FAST-LIO 建图 → 导出 map.pcd + pos_log.txt
cargo run -p doracap --example record_play_lio --features fastlio \
  -- --out /tmp/lio_sim.dcap --pcd /tmp/map_sim.pcd --pos /tmp/pos_sim.txt --duration 5
# 生成的 /tmp/lio_sim.dcap 含 imu / lidar / pose 三个通道；用 play --json 可看到每帧位姿

# 端到端验收（真机 Livox，mid360_config.json 为官方 Livox SDK 配置）
cargo run --release -p doracap --example record_play_lio --features livox \
  -- --live --config mid360_config.json --scan-hz 10 [--duration 30] \
     --out /tmp/live.dcap --pcd /tmp/map_live.pcd --pos /tmp/pos_live.txt
# 验收：pcl_viewer /tmp/map_live.pcd 与 FAST-LIO 官方直跑产出的 map.pcd 对照
```

## 真机验收方法（Livox）

依赖：`cmake` + `g++`（构建 vendored Livox SDK2），**机器人**与雷达同网络，准备一份有效的 Livox
配置。仓库里给了一份可直接改的模板 [`configs/livox/mid360_config.json`](configs/livox/mid360_config.json)：
把它拷到机器人上，把里面的 `host_ip` 改成**机器人网卡在雷达网段的 IP**（通常保持
`multicast_ip` 为 `224.1.1.5` 和默认端口即可；这也正是 Livox Viewer / driver2 用的同一份配置）。

0. **先确认雷达可达**（可选，但推荐——真机挂起/网络不对一测便知）：
   ```bash
   cargo run --release -p doracap --example record_play_lio --features livox \
     -- --discover --config mid360_config.json
   # 预期输出一行：device: handle=.. type=.. (..) SN=.. IP=..
   ```

1. **采集 + 建图**（一个命令完成：真机采集 → 录成 `.dcap` → 回放 → FAST-LIO 建图 → 导出）：
   ```bash
   cargo run --release -p doracap --example record_play_lio --features livox \
     -- --live --config mid360_config.json --scan-hz 10 --duration 30 \
        --out /tmp/live.dcap --pcd /tmp/map_live.pcd --pos /tmp/pos_live.txt
   ```
   实际上真机上更常用**只录不建图**（`Ctrl+C` 即停止并保存，立即退出），需要时再加 `--map`：
   ```bash
   cargo run --release -p doracap --example record_play_lio --features livox \
     -- --live --config mid360_config.json --scan-hz 10 --out live.dcap
   # Ctrl+C -> 落盘 live.dcap 后退出，不再耗时重放建图
   ```
   `--duration` 默认 `0`（不限时长）：录制会一直进行，手动 **Ctrl+C** 会**优雅收尾**（先把已录数据
  完整落盘、写出尾部索引，然后退出）。想定时结束传 `--duration 30`。要**一并回放+建图导出**就加
   `--map`——它会**直接从录制时那次 FAST-LIO 的内存态导出** `map.pcd` + `pos_log.txt`，
   **不再重放 `.dcap`**。产出的 `.dcap` 自带 `imu / lidar / pose` 三通道，是**自洽的建图过程回放源**
   （无需重跑 SLAM）。
2. **判据**：控制台打印 `[live] recorded Stats { imu: ..., lidar: ... }` 与 `[mapping] frames=...`；
   `frames` 应非 0（否则说明回放未驱动起建图）。用 `pcl_viewer /tmp/map_live.pcd` 看到与
   FAST-LIO 直跑一致的地图，即验收通过。
3. **产物**：`/tmp/live.dcap`（可复回放/可视化的单文件）、`/tmp/map_live.pcd`（地图点）、
   `/tmp/pos_live.txt`（每帧 `time qw qx qy qz pos_x pos_y pos_z`）。

## 仓库结构

```
crates/
  doracap-core/   # 值类型 + 存储 trait + 单文件后端 + Recorder/Player
  doracap-msgs/   # 规范消息 + rbag1 编解码
  doracap/        # 命令行
  doracap-fastlio/ # FAST-LIO 胶水：SensorData ⇄ 规范消息，录/放适配 DataSource
docs/
  doracap-design.md  # 完整设计文档（§0–§17）
  doracap-format.md  # .dcap 公开发行格式规范（第三方 viz 实现依据）
```

## 约定

本 crate 遵循 [Conventional Commits](https://www.conventionalcommits.org/)。

## License

MIT OR Apache-2.0
