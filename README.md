# DoraCap

一个**类型无关、与具体建图/导航库解耦**的录制与回放库（Rust）。

目标：把传感器采集 + 位姿/外参等数据录进**一个 `.dcap` 单文件**；回放读取该文件后，可交给独立的
RViz 类可视化工具进行数据回放。详见 [docs/doracap-design.md](docs/doracap-design.md)。

## 现状：P0 最小闭环

已实现并验证：

```
record(Imu + PointCloud) → 单文件 .dcap → play → decode → round-trip
```

- `doracap-core`：核心值类型（`Timestamp`/`Schema`/`Message`）、存储 trait
  （`StorageWriter`/`StorageReader`）、单文件 `.dcap` 后端（`SingleFileWriter`/`SingleFileReader`）、
  `Recorder` / `Player`（含 rate/loop/非阻塞 `next`/`try_next`/`now`）。
- `doracap-msgs`：规范消息 `Header`/`Time`/`PointCloud`/`Imu` + `rbag1` 紧凑编码
  （含 golden 与 round-trip 测试）。
- `doracap`：命令行（`selftest` / `info` / `play`）。`play` 支持 `--rate`/`--loop`，可输出
  单行 JSON（`--json`）或实时管道给外部可视化进程（`--show <cmd>`）。

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

# FAST-LIO 胶水：SimSource → .dcap → 回放(round-trip) → 驱动 FAST-LIO 建图
cargo run -p doracap-fastlio --bin glue_demo          # 录制+回放 round-trip 校验
cargo run -p doracap-fastlio --bin glue_demo -- --lio  # 额外跑一次 FAST-LIO
```

## 仓库结构

```
crates/
  doracap-core/   # 值类型 + 存储 trait + 单文件后端 + Recorder/Player
  doracap-msgs/   # 规范消息 + rbag1 编解码
  doracap/        # 命令行
  doracap-fastlio/ # FAST-LIO 胶水：SensorData ⇄ 规范消息，录/放适配 DataSource
docs/
  doracap-design.md  # 完整设计文档（§0–§17）
```

## 约定

本 crate 遵循 [Conventional Commits](https://www.conventionalcommits.org/)。

## License

MIT OR Apache-2.0
