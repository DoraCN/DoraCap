//! doracap 的「数据源 → 录制 → 回放 → FAST-LIO 建图 → 导出」一体化示例。
//!
//! 为什么放在 example：这是面向真机/仿真的**端到端验收测试**，不是独立可发布 crate。
//! 它把 doracap 各个 crate 串起来，并验证录制出的 `.dcap` 能被回放后驱动 FAST-LIO。
//!
//! # 数据源
//! - **仿真（默认，`--features fastlio`）**：使用 `fast_lio::data_source::SimSource`，
//!   纯 Rust、不需要硬件/CMake，用于在任何机器上验证整条链路。
//! - **真机 Livox（`--features livox`）**：使用 `fast_lio_driver` 连接 HAP / Mid-360；
//!   需要 `cmake` + C++ 编译器构建 vendored Livox SDK2，且与雷达同网络。
//!
//! # 产物
//! - `map.pcd`：FAST-LIO 增量地图点，可用 `pcl_viewer map.pcd` 打开。
//! - `pos_log.txt`：每帧 `time qw qx qy qz pos_x pos_y pos_z`（WXYZ + 位置）。
//!
//! # 运行
//! ```bash
//! # 仿真（无需硬件）
//! cargo run --release -p doracap --example record_play_lio --features fastlio \
//!   -- --out /tmp/lio.dcap --pcd /tmp/map.pcd --pos /tmp/pos_log.txt --duration 5
//!
//! # 真机 Livox（需与雷达同网络，且 config 为官方 mid360_config.json）
//! cargo run --release -p doracap --example record_play_lio --features livox \
//!   -- --live --config mid360_config.json --scan-hz 10 --duration 30 \
//!      --out /tmp/live.dcap --pcd /tmp/map.pcd --pos /tmp/pos_log.txt
//!
//! 真机路径下 `--duration` 默认 0（不限时长）：录制会一直进行，直到
//! 到达 `--duration` 秒，或手动 **Ctrl+C**（SIGINT）。Ctrl+C 会**优雅收尾**：
//! 先把已录数据完整落盘（写出尾部 `DCAP_END` 索引，`.dcap` 可被正常回放），
//! 然后**立即退出**（不进入耗时的重放建图）。若要同时建图导出 `map.pcd`，
//! 加 `--map`：直接从**录制时**那次 FAST-LIO 的内存态导出地图与轨迹，**不再重放 `.dcap`**。
//!
//! 先可跑 `--discover` 确认雷达可达：
//!   cargo run --release -p doracap --example record_play_lio --features livox \
//!     -- --discover --config mid360_config.json
//! 采集+保存（不建图）：
//!   cargo run --release -p doracap --example record_play_lio --features livox \
//!     -- --live --config mid360_config.json --scan-hz 10 --out live.dcap
//! 采集+保存+建图导出：
//!   cargo run --release -p doracap --example record_play_lio --features livox \
//!     -- --live --config mid360_config.json --scan-hz 10 --out live.dcap --map \
//!        --pcd map_live.pcd --pos pos_live.txt
//!
//! 录制中每隔 0.5s 在 stderr 打印一行实时状态（wall / data span / 实时占比 /
//! imu·lidar·pose 速率 / 当前位姿 / idle），数据源停滞(>2s)会打出告警。
//! 加 `--quiet` 可关闭实时状态输出。
//! ```

#[cfg(feature = "fastlio")]
mod app {
    use std::io::IsTerminal;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::{Duration, Instant};

    use fast_lio::data_source::{DataSource, NonBlocking, SimParams, SimSource};
    use fast_lio::laser_mapping::LioConfig;
    use fast_lio::types::{LidarType, PointType, SensorData, TimeUnit};

    use doracap_core::SingleFileWriter;
    use doracap_fastlio::LioRecorder;

    /// 一次录制的统计。
    #[derive(Debug)]
    struct Stats {
        imu: usize,
        lidar: usize,
    }

    impl Stats {
        fn new() -> Self {
            Stats { imu: 0, lidar: 0 }
        }
        fn total(&self) -> usize {
            self.imu + self.lidar
        }
        fn bump_sensor(&mut self, d: &SensorData) {
            match d {
                SensorData::Imu(_) => self.imu += 1,
                SensorData::LidarStandard(_) | SensorData::LidarAvia(_) => self.lidar += 1,
            }
        }
    }

    /// 取一条传感器样本的时间戳（秒）作为“录制时钟”。
    fn msg_time(d: &SensorData) -> f64 {
        match d {
            SensorData::Imu(i) => i.stamp,
            SensorData::LidarStandard(s) => s.stamp,
            SensorData::LidarAvia(a) => a.stamp,
        }
    }

    /// 把秒数格式化为 `m:ss.s`。
    fn fmt_secs(s: f64) -> String {
        let m = (s / 60.0).floor() as i64;
        let sec = s - m as f64 * 60.0;
        format!("{m}:{sec:04.1}")
    }

    pub fn run() -> Result<(), String> {
        let args: Vec<String> = std::env::args().collect();
        let live = args.iter().any(|a| a == "--live");
        let discover = args.iter().any(|a| a == "--discover");

        if discover {
            #[cfg(feature = "livox")]
            {
                return run_discover(&args);
            }
            #[cfg(not(feature = "livox"))]
            {
                return Err(
                    "--discover requires the `livox` feature: cargo run --example record_play_lio --features livox"
                        .into(),
                );
            }
        }

        if live {
            #[cfg(feature = "livox")]
            {
                return run_live(&args);
            }
            #[cfg(not(feature = "livox"))]
            {
                return Err(
                    "--live requires the `livox` feature: cargo run --example record_play_lio --features livox"
                        .into(),
                );
            }
        }

        run_sim(&args)
    }

    // ---------- 仿真路径 ----------

    fn run_sim(args: &[String]) -> Result<(), String> {
        let out = arg(args, "--out").unwrap_or_else(|| "/tmp/lio_sim.dcap".into());
        let pcd = arg(args, "--pcd").unwrap_or_else(|| "/tmp/map_sim.pcd".into());
        let pos = arg(args, "--pos").unwrap_or_else(|| "/tmp/pos_sim.txt".into());
        let duration = argf(args, "--duration", 5.0);
        let progress = !args.iter().any(|a| a == "--quiet");

        let params = SimParams {
            imu_hz: 200.0,
            lidar_hz: 10.0,
            duration,
            radius: 5.0,
            omega: 0.3,
            height: 1.0,
            points_per_scan: 600,
            init_static: 0.5,
            acc_noise: 0.02,
            gyr_noise: 0.002,
        };
        let mut source = SimSource::new(&params);
        let writer = SingleFileWriter::open(&out).map_err(|e| e.to_string())?;
        let cfg = lio_cfg(LidarType::Velo16);
        let (stats, mut rec) = record_loop(writer, &mut source, None, None, &cfg, progress, true)?;

        println!(
            "[sim] recorded {stats:?} (total={}) -> {out}",
            stats.total()
        );
        let frames = export_map(&mut rec, &pcd, &pos)?;
        println!("[mapping] frames={frames} -> pcd={pcd}, pos={pos}");
        Ok(())
    }

    // ---------- 真机 Livox 路径 ----------

    #[cfg(feature = "livox")]
    fn run_discover(args: &[String]) -> Result<(), String> {
        use livox_sdk2::Sdk;

        let config = arg(args, "--config").unwrap_or_else(|| "mid360_config.json".into());
        println!("[discover] starting Livox SDK with {config} ...");
        let sdk = Sdk::new(&config).map_err(|e| format!("SDK init: {e}"))?;
        println!("[discover] waiting 5 s for device discovery ...");
        std::thread::sleep(Duration::from_secs(5));

        let devices = sdk.devices();
        if devices.is_empty() {
            return Err(
                "no LiDAR found — check that `host_ip` is the robot NIC on the LiDAR subnet, \
                 and the LiDAR is powered + cabled"
                    .into(),
            );
        }
        for d in devices {
            println!(
                "device: handle={} type={} ({}) SN={} IP={}",
                d.handle,
                d.dev_type,
                d.type_name(),
                d.sn,
                d.lidar_ip
            );
        }
        Ok(())
    }

    #[cfg(feature = "livox")]
    fn run_live(args: &[String]) -> Result<(), String> {
        use fast_lio_driver::{DriverParams, open};
        use std::sync::Arc;

        let config = arg(args, "--config").unwrap_or_else(|| "mid360_config.json".into());
        let out = arg(args, "--out").unwrap_or_else(|| "/tmp/live.dcap".into());
        let pcd = arg(args, "--pcd").unwrap_or_else(|| "/tmp/map_live.pcd".into());
        let pos = arg(args, "--pos").unwrap_or_else(|| "/tmp/pos_live.txt".into());
        let map = args.iter().any(|a| a == "--map");
        let progress = !args.iter().any(|a| a == "--quiet");
        // 0 = 不限时长，直到手动 Ctrl+C。
        let duration = argf(args, "--duration", 0.0);
        let scan_hz = argf(args, "--scan-hz", 10.0);
        let scan_period = Duration::from_secs_f64(1.0 / scan_hz);

        println!("[live] connecting Livox ({config}), scan_period={scan_period:?} ...");
        let params = DriverParams::livox(config, scan_period);
        let mut source = open(&params).map_err(|e| format!("open driver: {e}"))?;

        // 优雅停止开关：Ctrl+C 把标记置真，录制循环在退出前会先写出完整尾部索引。
        let stop = Arc::new(AtomicBool::new(false));
        ctrlc::set_handler({
            let stop = stop.clone();
            move || stop.store(true, Ordering::Relaxed)
        })
        .map_err(|e| format!("set SIGINT handler: {e}"))?;

        let deadline = if duration > 0.0 {
            println!("[live] recording {duration}s ...");
            Some(Instant::now() + Duration::from_secs_f64(duration))
        } else {
            println!("[live] recording until Ctrl+C ...");
            None
        };

        let writer = SingleFileWriter::open(&out).map_err(|e| e.to_string())?;
        let cfg = lio_cfg(LidarType::Avia);
        let (stats, mut rec) = record_loop(
            writer,
            source.as_mut(),
            deadline,
            Some(&stop),
            &cfg,
            progress,
            map,
        )?;
        if stats.total() == 0 {
            return Err(
                "recorded 0 samples — check that the LiDAR is reachable and the config is valid"
                    .into(),
            );
        }
        if stop.load(Ordering::Relaxed) {
            println!("[live] Ctrl+C received, finalized {out}");
        }

        println!(
            "[live] recorded {stats:?} (total={}) -> {out}",
            stats.total()
        );
        // Ctrl+C == 停止并保存，立即退出（不进入耗时的重放建图）。
        if !map {
            println!(
                "[hint] 需要同时导出 map.pcd/pos.txt: 加 --map, 或看内容: `doracap play {out} --json`"
            );
            return Ok(());
        }

        // 这里不再重放 .dcap：导出录制时那次 FAST-LIO 在内存里建好的地图与轨迹即可（一次完成）。
        let frames = export_map(&mut rec, &pcd, &pos)?;
        println!("[mapping] frames={frames} -> pcd={pcd}, pos={pos}");
        Ok(())
    }

    // ---------- 通用录制循环 ----------

    /// 把任意 `DataSource` 录制进 `.dcap`。
    /// - `deadline=Some`：按墙上时钟限时（真机 `--duration N`）。
    /// - `deadline=None` 且 `stop=None`：一直录到数据源结束（仿真）。
    /// - `deadline=None` 且 `stop=Some`：一直录到 Ctrl+C 把 `stop` 置真（真机不限时）。
    ///
    /// 退出时总会先 `finish()` 写出完整尾部索引，保证 `.dcap` 可回放。
    fn record_loop(
        writer: SingleFileWriter,
        source: &mut dyn DataSource,
        deadline: Option<Instant>,
        stop: Option<&AtomicBool>,
        cfg: &LioConfig,
        progress: bool,
        wait_map: bool,
    ) -> Result<(Stats, LioRecorder), String> {
        // 一遍录制：写传感器 + 喂 FAST-LIO + 把位姿写到同一条 `.dcap`。
        let mut rec = LioRecorder::new(Box::new(writer), cfg).map_err(|e| e.to_string())?;
        let mut stats = Stats::new();
        let started = Instant::now();
        let mut first_ts: Option<f64> = None;
        let mut last_ts: Option<f64> = None;
        let mut last_ui = Instant::now();
        let mut last_msg = Instant::now();
        let mut ui_imu = 0usize;
        let mut ui_lidar = 0usize;
        let mut last_pose = 0usize;
        let mut idle_warned = false;
        let tty = std::io::stderr().is_terminal();

        loop {
            if let Some(d) = deadline
                && Instant::now() >= d
            {
                break;
            }
            if let Some(s) = stop
                && s.load(Ordering::Relaxed)
            {
                break;
            }
            match source.try_next() {
                Ok(Some(data)) => {
                    stats.bump_sensor(&data);
                    last_msg = Instant::now();
                    let ts = msg_time(&data);
                    first_ts.get_or_insert(ts);
                    last_ts = Some(ts);
                    match &data {
                        SensorData::Imu(_) => ui_imu += 1,
                        SensorData::LidarStandard(_) | SensorData::LidarAvia(_) => ui_lidar += 1,
                    }
                    rec.push(&data).map_err(|e| e.to_string())?;
                }
                Ok(None) => break,
                Err(NonBlocking) => std::thread::sleep(Duration::from_millis(5)),
            }

            if !progress {
                continue;
            }
            let now = Instant::now();
            let since_ui = now.duration_since(last_ui).as_secs_f64();
            if since_ui < 0.5 {
                continue;
            }

            let wall = now.duration_since(started).as_secs_f64();
            let span = match (first_ts, last_ts) {
                (Some(a), Some(b)) => (b - a).max(0.0),
                _ => 0.0,
            };
            let rt = if wall > 0.0 { span / wall * 100.0 } else { 0.0 };
            let imu_hz = ui_imu as f64 / since_ui;
            let lidar_hz = ui_lidar as f64 / since_ui;
            let pose_n = rec.poses().len();
            let pose_hz = (pose_n - last_pose) as f64 / since_ui;
            let idle = now.duration_since(last_msg).as_secs_f64();
            let pos_s = match rec.poses().last() {
                Some(p) => format!("({:.3},{:.3},{:.3})", p.1[0], p.1[1], p.1[2]),
                None => "(n/a)".into(),
            };

            let line = format!(
                "[REC] wall {} | data {} ({:>3.0}% rt) | imu {} ({:>5.1}/s) lidar {} ({:>4.1}/s) | pose {} ({:>4.1}/s) | pos {} | idle {:.1}s",
                fmt_secs(wall),
                fmt_secs(span),
                rt,
                stats.imu,
                imu_hz,
                stats.lidar,
                lidar_hz,
                pose_n,
                pose_hz,
                pos_s,
                idle
            );
            if tty {
                eprint!("\r\x1b[2K{line}");
            } else {
                eprintln!("{line}");
            }
            if idle > 2.0 && !idle_warned {
                idle_warned = true;
                eprintln!(
                    "\n[REC] WARN: no new sensor data for {idle:.1}s (data source likely stalled/stopped)"
                );
            } else if idle <= 2.0 {
                idle_warned = false;
            }
            ui_imu = 0;
            ui_lidar = 0;
            last_pose = pose_n;
            last_ui = now;
        }
        if tty && progress {
            eprintln!();
        }
        // 录制与建图解耦：sensor 始终满速落盘，不因建图而丢帧。
        // - wait_map（真机 `--map` / 仿真）: 等建图线程把已收帧跑完（姿态/地图完整，耗时=建全程）。
        // - 否则 `finish_now`：立即收尾，弃建图积压，保住全部 sensor 数据。
        if wait_map {
            rec.finish().map_err(|e| e.to_string())?;
        } else {
            rec.finish_now().map_err(|e| e.to_string())?;
        }
        Ok((stats, rec))
    }

    // ---------- 从录制态直接导出（一次完成，不重放 .dcap）----------

    fn export_map(rec: &mut LioRecorder, pcd: &str, pos: &str) -> Result<usize, String> {
        rec.mapping_mut().ikdtree.flatten_to_storage();
        write_pcd(pcd, &rec.mapping().ikdtree.pcl_storage)?;
        write_pos_log(pos, rec.poses())?;
        Ok(rec.poses().len())
    }

    // ---------- 导出 ----------

    fn write_pcd(path: &str, points: &[PointType]) -> Result<(), String> {
        use std::io::Write;
        let mut f = std::fs::File::create(path).map_err(|e| format!("create {path}: {e}"))?;
        let mut w = |s: String| -> Result<(), String> {
            f.write_all(s.as_bytes()).map_err(|e| e.to_string())
        };
        w("# .PCD v0.7 - Point Cloud Data file format\n".into())?;
        w("VERSION 0.7\n".into())?;
        w("FIELDS x y z intensity\n".into())?;
        w("SIZE 4 4 4 4\n".into())?;
        w("TYPE F F F F\n".into())?;
        w("COUNT 1 1 1 1\n".into())?;
        w(format!("WIDTH {}\n", points.len()))?;
        w("HEIGHT 1\n".into())?;
        w("VIEWPOINT 0 0 0 1 0 0 0\n".into())?;
        w(format!("POINTS {}\n", points.len()))?;
        w("DATA ascii\n".into())?;
        for p in points {
            w(format!("{} {} {} {}\n", p.x, p.y, p.z, p.intensity))?;
        }
        Ok(())
    }

    fn write_pos_log(path: &str, rows: &[(f64, [f64; 3], [f64; 4])]) -> Result<(), String> {
        use std::io::Write;
        let mut f = std::fs::File::create(path).map_err(|e| format!("create {path}: {e}"))?;
        let header = "# time qw qx qy qz pos_x pos_y pos_z\n";
        f.write_all(header.as_bytes()).map_err(|e| e.to_string())?;
        for (t, pos, quat) in rows {
            let line = format!(
                "{t:.6} {:.8} {:.8} {:.8} {:.8} {:.6} {:.6} {:.6}\n",
                quat[0], quat[1], quat[2], quat[3], pos[0], pos[1], pos[2]
            );
            f.write_all(line.as_bytes()).map_err(|e| e.to_string())?;
        }
        Ok(())
    }

    // ---------- 辅助 ----------

    fn arg(args: &[String], name: &str) -> Option<String> {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
    }

    fn argf(args: &[String], name: &str, default: f64) -> f64 {
        arg(args, name)
            .and_then(|s| s.parse().ok())
            .unwrap_or(default)
    }

    /// 为指定的 LiDAR 类型给出 FAST-LIO 建图配置（Livox 走 Avia，仿真走 Velo16）。
    fn lio_cfg(lidar_type: LidarType) -> LioConfig {
        // fast-lio 0.1.0 的 Preprocess::new 把内部 pl_buff/typess 按默认 n_scans=6 预分配，
        // 而 LaserMapping::new 之后才用 cfg.n_scans 覆盖 preprocess.n_scans，并不重新扩容。
        // Avia 路径里 `for i in 0..n_scans { pl_buff[i] }` 是无条件执行的，因此 n_scans 必须 =6，
        // 否则第一帧就越界。Livox Mid-360 又是单扫描线(line=0)，n_scans=6 与 fast-lio-app 一致。
        let n_scans = match lidar_type {
            LidarType::Avia => 6,
            // Velo16 这里 feature_extract_enable=false，pl_buff 的遍历在 feature 分支内，
            // 故 n_scans=16 不触发越界；若开启 feature 需先修 fast-lio 的扩容 bug。
            _ => 16,
        };
        LioConfig {
            lidar_type,
            feature_extract_enable: false,
            point_filter_num: 2,
            n_scans,
            scan_rate: 10,
            timestamp_unit: TimeUnit::Us,
            filter_size_surf: 0.5,
            filter_size_map: 0.5,
            ..Default::default()
        }
    }
}

#[cfg(not(feature = "fastlio"))]
fn main() {
    eprintln!(
        "this example requires a feature.\n  --features fastlio : simulated source (no hardware)\n  --features livox   : real Livox (needs cmake + C++)"
    );
    std::process::exit(2);
}

#[cfg(feature = "fastlio")]
fn main() {
    if let Err(e) = app::run() {
        eprintln!("FAILED: {e}");
        std::process::exit(1);
    }
}
