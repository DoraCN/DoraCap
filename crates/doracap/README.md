# doracap

Command line for **DoraCap**: record sensor streams into one self-describing
`.dcap` file and replay it with playback controls, or pipe frames to an
external visualizer.

## Commands

- `selftest` — end-to-end round-trip check (record → `.dcap` → play → decode)
- `info <file>` — print channels, schemas and scene metadata
- `play <file>` — replay with `--rate`, `--loop`, `--json`, `--seek`,
  `--seek-ratio` and `--show <cmd>`

## Usage

```bash
doracap info data.dcap
doracap play --rate 1.0 --loop data.dcap
```

License: MIT OR Apache-2.0
