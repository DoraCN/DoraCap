# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- P0 最小闭环：`dorabag-core` 核心类型、存储 trait、单文件 `.rbag` 后端，`Recorder`/`Player`。
- `dorabag-msgs`：`Header`/`Time`/`PointCloud`/`Imu` + `rbag1` 编解码（含 golden/round-trip 测试）。
- `dorabag` CLI：`selftest` / `info` / `play`。
- `play` 支持 `--rate`/`--loop`/`--json`/`--show <cmd>`：实时回放并以 JSON 流输出/管道给外部 viz。
- `dorabag-fastlio`：`SensorData ⇄ dorabag-msgs` 胶水，把 `fast_lio::data_source::DataSource`
  录制进 `.rbag`、从 `.rbag` 回放成 `DataSource`；`glue_demo` 验证 round-trip 并能驱动 FAST-LIO 建图。
- 设计文档 `docs/dorabag-design.md`（§0–§17）。
