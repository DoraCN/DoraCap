# doracap-fastlio

Adapter that bridges **FAST-LIO** sensor streams to **DoraCap**:

- **Record**: any `fast_lio::data_source::DataSource` → `.dcap`
- **Replay**: a `.dcap` → `fast_lio::data_source::DataSource`

Recording writes a self-describing `doracap/SceneMeta` channel (world frame +
channel roles) so a playback visualizer can rebuild the map without re-running
SLAM.

License: MIT OR Apache-2.0
