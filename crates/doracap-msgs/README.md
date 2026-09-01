# doracap-msgs

Canonical, source-library-agnostic mapping / navigation messages and the compact
`rbag1` codec used by the `.dcap` format.

Provides:

- `Header`, `Time`, `Stamped`
- `PointCloud`, `PointField`, `Imu`, `PoseStamped`
- `SceneMeta` (world frame + `ChannelRole`), for self-describing `.dcap` files
- the `Codec` trait with `rbag1` binary encoding (golden-bytes + round-trip tests)

It only depends on `doracap-core`'s `Timestamp` for header semantic time, and
never references any source-library types.

License: MIT OR Apache-2.0
