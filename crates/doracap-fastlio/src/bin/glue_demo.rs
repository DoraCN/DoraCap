//! 胶水验证 demo：
//! 1. 用 FAST-LIO 的 `SimSource` 生成传感器流。
//! 2. `record_source` 把它录进一个 `.dcap`。
//! 3. `BagDataSource` 从 `.dcap` 回放成 `SensorData` 流。
//! 4. 与一个全新的 `SimSource`（同参数、确定性）逐条比对（round-trip）。
//! 5. 可选 `--lio`：跑一次 FAST-LIO `LaserMapping`，证明录制的 bag 能驱动建图。

use doracap_core::{PlayOptions, SingleFileWriter};
use doracap_fastlio::{BagDataSource, record_source};
use fast_lio::data_source::{DataSource, SimParams, SimSource};
use fast_lio::laser_mapping::{LaserMapping, LioConfig};
use fast_lio::types::{LidarType, SensorData, TimeUnit};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let run_lio_flag = args.iter().any(|a| a == "--lio");
    let path = std::env::temp_dir().join("doracap_fastlio_demo.dcap");
    let _ = std::fs::remove_file(&path);

    let params = SimParams {
        imu_hz: 200.0,
        lidar_hz: 10.0,
        duration: 5.0,
        radius: 5.0,
        omega: 0.15,
        height: 1.0,
        points_per_scan: 300,
        init_static: 0.5,
        acc_noise: 0.02,
        gyr_noise: 0.002,
    };

    // 1) 录制
    let mut src = SimSource::new(&params);
    let writer = SingleFileWriter::open(&path).expect("open writer");
    record_source(writer, &mut src).expect("record");
    println!("recorded -> {path:?}");

    // 2) 参考流（同参数、确定性）
    let mut reference = SimSource::new(&params);
    let mut expected = Vec::new();
    while let Some(d) = reference.next() {
        expected.push(d);
    }

    // 3) 回放
    let mut bag = BagDataSource::open(&path, PlayOptions::default().rate(0.0)).expect("open bag");
    let mut actual = Vec::new();
    while let Some(d) = bag.next() {
        actual.push(d);
    }

    // 4) round-trip 比对
    println!("expected={} actual={}", expected.len(), actual.len());
    assert_eq!(expected.len(), actual.len(), "message count mismatch");
    for (i, (a, b)) in expected.iter().zip(actual.iter()).enumerate() {
        assert!(sensor_eq(a, b), "sensor mismatch at #{i}: {a:?} vs {b:?}");
    }
    println!("glue OK: round-trip matches ({} samples)", actual.len());

    // 5) （可选）跑一次 LIO，证明 bag 能驱动 FAST-LIO 建图
    if run_lio_flag {
        let cfg = LioConfig {
            lidar_type: LidarType::Velo16,
            feature_extract_enable: false,
            point_filter_num: 2,
            n_scans: 16,
            scan_rate: 10,
            timestamp_unit: TimeUnit::Ms,
            filter_size_surf: 0.5,
            filter_size_map: 0.5,
            ..Default::default()
        };
        let mut mapping = LaserMapping::new(&cfg);
        let mut frames = 0usize;
        let mut map_points = 0i32;
        let mut pos = [0.0f64; 3];
        let mut replay = BagDataSource::open(&path, PlayOptions::default().rate(0.0))
            .expect("reopen bag for lio");
        while let Some(d) = replay.next() {
            match &d {
                SensorData::Imu(imu) => mapping.add_imu(imu),
                SensorData::LidarStandard(s) => mapping.add_lidar_standard(s),
                SensorData::LidarAvia(a) => mapping.add_lidar_avia(a),
            }
            if mapping.has_data()
                && let Some(res) = mapping.run_once()
            {
                frames += 1;
                map_points = res.map_points;
                pos = [res.pos[0], res.pos[1], res.pos[2]];
            }
        }
        println!(
            "lio on replay: frames={frames} map_points={map_points} pos=({:.3},{:.3},{:.3})",
            pos[0], pos[1], pos[2]
        );
    }
}

fn ns(s: f64) -> i64 {
    (s * 1e9).round() as i64
}

fn sensor_eq(a: &SensorData, b: &SensorData) -> bool {
    match (a, b) {
        (SensorData::Imu(x), SensorData::Imu(y)) => {
            ns(x.stamp) == ns(y.stamp) && x.acc == y.acc && x.gyr == y.gyr
        }
        (SensorData::LidarStandard(x), SensorData::LidarStandard(y)) => {
            ns(x.stamp) == ns(y.stamp)
                && x.points.len() == y.points.len()
                && x.points.iter().zip(&y.points).all(|(p, q)| {
                    p.x == q.x
                        && p.y == q.y
                        && p.z == q.z
                        && p.intensity == q.intensity
                        && p.time == q.time
                        && p.ring == q.ring
                })
        }
        (SensorData::LidarAvia(x), SensorData::LidarAvia(y)) => {
            ns(x.stamp) == ns(y.stamp)
                && x.points.len() == y.points.len()
                && x.points.iter().zip(&y.points).all(|(p, q)| {
                    p.x == q.x
                        && p.y == q.y
                        && p.z == q.z
                        && p.reflectivity == q.reflectivity
                        && p.tag == q.tag
                        && p.line == q.line
                        && p.offset_time == q.offset_time
                })
        }
        _ => false,
    }
}
