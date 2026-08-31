# rustbag

一个**类型无关、与具体建图/导航库解耦**的录制与回放库（Rust）。

目标：把传感器采集 + 位姿/外参等数据录进**一个 `.rbag` 单文件**；回放读取该文件后，可交给独立的
RViz 类可视化工具进行数据回放。详见 [docs/rustbag-design.md](docs/rustbag-design.md)。

## 现状：P0 最小闭环

已实现并验证：

```
record(Imu + PointCloud) → 单文件 .rbag → play → decode → round-trip
```

- `rustbag-core`：核心值类型（`Timestamp`/`Schema`/`Message`）、存储 trait
  （`StorageWriter`/`StorageReader`）、单文件 `.rbag` 后端（`SingleFileWriter`/`SingleFileReader`）、
  `Recorder` / `Player`（含 rate/loop/非阻塞 `next`/`try_next`/`now`）。
- `rustbag-msgs`：规范消息 `Header`/`Time`/`PointCloud`/`Imu` + `rbag1` 紧凑编码
  （含 golden 与 round-trip 测试）。
- `rustbag`：命令行（`selftest` / `info`）。

## 快速开始

```bash
# 构建
cargo build

# 自检：录制 → 单文件 .rbag → 回放 → 解码 → round-trip
cargo run -p rustbag -- selftest

# 查看某个 .rbag 的信息
cargo run -p rustbag -- info <file.rbag>
```

## 仓库结构

```
crates/
  rustbag-core/   # 值类型 + 存储 trait + 单文件后端 + Recorder/Player
  rustbag-msgs/   # 规范消息 + rbag1 编解码
  rustbag/        # 命令行
docs/
  rustbag-design.md  # 完整设计文档（§0–§17）
```

## 约定

本 crate 遵循 [Conventional Commits](https://www.conventionalcommits.org/)。

## License

MIT OR Apache-2.0

