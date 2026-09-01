# doracap-core

Core, storage-agnostic value types and backend interfaces for **DoraCap** — a
recording/replay library that packs sensor streams plus poses and external
calibration into a single self-describing `.dcap` file.

This crate depends only on `std`. It provides:

- `Timestamp`, `Schema`, `Message`, `ChannelMeta`
- `StorageWriter` / `StorageReader` storage traits
- the single-file `.dcap` backend (`SingleFileWriter` / `SingleFileReader`)
- `Recorder` and `Player` (time-sorted playback, with rate / loop / seek)

## Usage

```rust,ignore
use doracap_core::{Recorder, Schema, SingleFileWriter, Timestamp};

let writer = SingleFileWriter::open("out.dcap")?;
let mut rec = Recorder::new(Box::new(writer));
rec.add_channel("imu", &schema)?;
rec.write("imu", Timestamp::from_secs_f64(0.0), &payload)?;
rec.finish()?;
```

The `.dcap` byte layout is documented in `docs/doracap-format.md` in the
source repository (see the `repository` field in `Cargo.toml`).

License: MIT OR Apache-2.0
