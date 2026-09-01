# DoraCap `.dcap` 文件格式规范 (v2)

> 本文是 `.dcap` 的**公开发行格式契约**。
>
> **目标读者**：想实现一个能回放建图过程的第三方 viz / 工具开发者。你只需遵守本文，无需阅读
> `doracap` 任何源码即可实现读取器，并对一个 `.dcap` 文件做到：
>
> 1. 解析自描述容器（schema / 频道 / 消息 / 索引）；
> 2. 按时间轴以视频方式回放（播放 / 暂停 / 拖动 / 倍速 / 循环）；
> 3. 无需重跑 SLAM，把原始点云帧 + 每帧位姿逐帧累加，重现正在生长的地图 / 轨迹。
>
> 官方参考实现：`doracap-core`（容器读 / 写 + 回放）、`doracap-msgs`（规范消息 + `rbag1` 编解码）。
> 纯 Rust 参考读取器：`doracap-core::SingleFileReader` + `doracap-core::Player`。

## 目录

0. 术语与约定
1. 设计目标
2. 总体布局
3. Header
4. Schema 注册表
5. Channel 注册表
6. Chunk 与消息记录
7. Footer 与索引
8. 写入端契约
9. 读取端独立伪代码
10. `rbag1` 编码规则
11. 规范消息 wire 布局
12. 场景元数据 `doracap/SceneMeta`
13. 建图回放行为契约
14. 第三方实现自检清单
15. 跨语言 golden 样例
16. 向后兼容 / 版本

---

## 0. 术语与约定

- **little-endian**：所有多字节整型与浮点均为小端。
- **str**：`u32 len` + `len` 字节 UTF-8（长度不含结尾符）。
- **bytes**：`u32 len` + `len` 字节原始数据。
- **chunk**：一组消息记录 + 压缩单元。`.dcap` 把消息按偏移量+大小分组成 chunk，每个 chunk 独立压缩。
- **compressor**：Header 里的 u32 字段，声明 chunk 字节流的压缩算法：
  `0=不压缩`、`1=zstd（保留，未实现）`、`2=deflate/zlib`（参考实现默认）。
- **四元数**：一律 `[w, x, y, z]`（标量在前），与 FAST-LIO 一致。`PoseStamped::orientation`、
  `Imu::orientation` 均为此顺序。**实现时务必核对**，否则位姿会错。
- **规范消息**：`type_name` 以 `doracap/` 开头，payload 用 `rbag1` 编码，均在本规范定义。

## 1. 设计目标

`.dcap` 是一个类型无关、自描述、单文件、可 seek 的容器：

- **类型无关**：doracap 只认识 `(channel, stamp, schema, payload)` 四元组，不解析领域语义。
- **自描述**：文件内含 schema 注册表、频道清单、消息流、索引，以及场景元数据 `SceneMeta`（§12）。
- **单文件**：一切都在一个 `.dcap` 里，便于分发、备份、seek 与可视化。
- **可回放建图**：通过 `SceneMeta` 声明世界系与通道角色，viz 把点云帧经对应位姿变换到世界系并逐帧累加。
- **有界内存 & 可 seek**：消息按 chunk 压缩并按时间建索引；读端可按时间定位、只解压目标 chunk 流式播放，
  长录制（数十 GB 原始）不把整个文件/全部消息载入内存。

## 2. 总体布局

按文件顺序：

| 区段 | 内容 |
|---|---|
| Header | `magic "#DCAP"(5B) + version(u32) + flags(u32) + compressor(u32)` |
| Schemas | `u32 count`；每条 `{ u16 id; str type_name; str encoding }` |
| Channels | `u32 count`；每条 `{ u16 id; str name; u16 schema_id }` |
| Chunks | 每条 `{ Chunk Header(50B); compressor(chunk 记录块) }` |
| Footer | `[index 条目 × chunk_count] + trailer(32B)` |

`data_offset` = Header + Schemas + Channels 之后的**首个 chunk 起点**（字节偏移）。

关于 chunk：
- 读端**应当按 chunk 顺序**遍历；每解压一个 chunk 即得到若干条消息记录。
- chunk 内消息按 `stamp` 升序；chunk 之间也按时间序排列（写端稳定排序），因而整条时间线单调。
- 每个 chunk 是独立的压缩单元；读取/seek 只需按索引定位到目标 chunk 并解压它，不必解压整个文件。

## 3. Header

```text
magic   : 0x23 0x44 0x43 0x41 0x50     # ASCII "#DCAP"，5 字节
version : u32                           # 本规范 = 2
flags   : u32                           # 保留，当前恒为 0
compressor : u32                        # chunk 压缩算法：0=无，1=zstd(保留)，2=deflate/zlib
```

读取器若遇 `version > 支持的最大版本`，应报"不支持的版本"并停止，而不是尝试解析。
`compressor` 决定 §6 chunk 记录块的解码方式；`2` 表示每条 chunk 的记录块是 zlib 封装（含首尾校验）
的 deflate 流。

## 4. Schema 注册表

```text
count : u32                     # schema 条数
每条  :
  id        : u16               # schema 稳定标识
  type_name : str               # 稳定类型名，如 "doracap/PointCloud"
  encoding  : str               # 编码标识，如 "rbag1"
```

读取端规则：
- **必须使用文件中的 `id`**，不得假设 `id` 从 1 开始、连续或与顺序相关。
- 以 `type_name` 判定消息种类并以 `encoding` 决定 payload 解码方式。
- `doracap/` 前缀为规范消息保留；其他前缀表示自定义消息，viz 若无对应解码器应跳过或显示字节。

## 5. Channel 注册表

```text
count : u32                     # 频道条数
每条  :
  id        : u16               # 频道稳定标识
  name      : str               # 主题名，如 "imu" / "lidar" / "pose"
  schema_id : u16               # 指向 Schema 注册表的 id
```

读取端规则：
- 频道名 `imu` / `lidar` / `pose` 是**约定**；结合 `SceneMeta`（§12）才构成"完整 / 显式"声明。
- `schema_id` 引用 §4 中的 schema。消息记录里的 `channel_id` 引用这里。

## 6. Chunk 与消息记录

消息**不再直接线性排列**，而是被分组进若干 chunk。数据结构如下：

### 6.1 Chunk Header（50 字节，未压缩，直接写盘）

```text
magic             : 0x44 0x43 0x41 0x50 0x5F 0x43 0x48 0x55 0x4E 0x4B   # ASCII "DCAP_CHUNK"（10 字节）
msg_begin         : u64                                               # 本 chunk 首条消息的全局序号（0 基）
msg_count         : u32                                               # 本 chunk 内的消息条数
start_stamp       : u64                                               # 本 chunk 内最小调度时间（ns）
end_stamp         : u64                                               # 本 chunk 内最大调度时间（ns）
uncompressed_size : u32                                               # records 解压后字节数
compressed_size   : u32                                               # 紧随其后的压缩块字节数
crc32             : u32                                               # records（解压后）的 IEEE CRC32
```

`chunk = ChunkHeader + compressor(records)`，其中 `compressor` 由 Header 的 `compressor` 字段声明。

### 6.2 records（解压后）

`records` 是若干**消息记录**的连续拼接（小端、每条 14 字节定头 + payload）：

```text
channel_id : u16               # 指向 Channel 注册表的 id
stamp      : u64               # 调度时间，单位：纳秒（单调、整数）
len        : u32               # payload 字节数
payload    : bytes             # len 字节；doracap 不解码，由 encoding 解释
```

- `stamp` 单位纳秒；doracap 只用它做排序 / 调度，不解析 / 换算消息体内的再一层时间。
- 消息体可另带 `Header.stamp`（sec + nsec 语义时间），doracap 原样存、原样吐，不解码。
- chunk 内的 `msg_count` 必须与 records 中实际消息条数一致；解压后应先比对 `len(records) == uncompressed_size`，
  再按 `msg_count` 逐条解析（越界即视为损坏 / 截断，读到此处停止）。

### 6.3 压缩（`compressor`）

- `0`：records 原样写入，`compressed_size == uncompressed_size`。
- `2`：records 用 **zlib 封装的 deflate**（即 RFC 1950/1951，含 Adler-32 校验）压缩，写入 `compressed_size` 字节。
- `1`：**zstd**，保留未实现；读取端遇 `1` 应报"不支持的 compressor"。
- 读端解压后**必须校验 `len(out) == uncompressed_size` 且 `crc32(out) == crc32`**；不一致视为损坏。

## 7. Footer 与索引

Footer 位于文件**末尾**，由 chunk 索引条目 + 32 字节尾标组成：

```text
[index 条目 × chunk_count]     # 每条 28 字节，紧跟最后一个 chunk 之后
magic         : 0x44 0x43 0x41 0x50 0x5F 0x45 0x4E 0x44   # ASCII "DCAP_END"（8 字节）
data_offset   : u64                                       # 首个 chunk 的字节偏移
chunk_count   : u64                                       # chunk 总数
message_count : u64                                       # 消息总条数（所有 chunk 之和）
```

每条 index 条目：

```text
offset      : u64   # 该 chunk 头的字节偏移
start_stamp : u64   # 该 chunk 内最小调度时间（ns）
end_stamp   : u64   # 该 chunk 内最大调度时间（ns）
msg_count   : u32   # 该 chunk 内的消息条数
```

读取器优先用 Footer 快速定位；若缺失（截断 / 断电），**回退为线性扫描**：从 `data_offset` 起逐 chunk 读头、
解压、解析；遇到"magic 不符 / 长度越界 / 尾部不足"即停止，忽略截断尾巴（仍能读到已完整写入的 chunk）。

## 8. 写入端契约

以下约束是写入方的责任，读取方应据此容忍：

1. **所有 schema / channel 必须在首条消息写入前注册完毕**。因为 `.dcap` 在第一条消息前一次性把
   Schemas + Channels 段落盘（`SingleFileWriter` 在 `ensure_header` 中写一次），之后再注册的新通道
   不会进入头部。规范录制（`LioRecorder` / `record_source`）都是先注册全部通道再写。
2. `stamp` 应为**单调非递减**；写入方负责保证。读取方按 `stamp` 稳定升序（平局保持写入顺序）。
3. schema / channel 的 `id` 由写入方自动分配（自 1 递增），读取方不得依赖其值含义。
4. 同一频道名（`channel.name`）应绑定同一 schema；写入方需自行保证。
5. **chunk 由写入方自动分块**：通常按未压缩字节阈值（默认 ~4 MiB，约 1 秒的点云流）切块；
   写入方在 `finish()` 时把最后一块 chunk 落盘，并在文件末尾写 footer 索引。
6. 每个 chunk 内的消息**按 `stamp` 升序**；chunk 之间按写入顺序，整体时间单调。被读端按顺序遍历即得时间序。
7. `compressor` 默认 `2`（deflate/zlib），写入方也可选 `0`（不压缩）。读取方须按 header 中值解码。

## 9. 读取端独立伪代码

这是不含任何 doracap 实现细节、可直接翻译成目标语言的读取流程。包含两种能力：

1. **离线 / 全量**：读整个文件、解压全部 chunk，得到所有消息（适合 `info` / 导出）。
2. **流式 / 有界内存**：只读 footer（索引）定位到目标 chunk，按需解压，适合长时间播放 / seek。

```text
open(path):
  data = read_all_bytes(path)
  assert data[0..5] == "#DCAP"                       // 31. 见 §3
  version = le_u32(data[5..9]); flags = le_u32(data[9..13])
  if version > SUPPORTED: error
  compressor = le_u32(data[13..17])                  // 0=无 1=zstd(保留) 2=deflate/zlib

  pos = 17
  schema_count = le_u32(data[pos..pos+4]); pos += 4
  schemas = {}
  repeat schema_count:
    id = le_u16(...); type_name = read_str(...); encoding = read_str(...)
    schemas[id] = (type_name, encoding)

  channel_count = le_u32(...); pos += 4
  channels = {}
  repeat channel_count:
    id = le_u16(...); name = read_str(...); schema_id = le_u16(...)
    channels[id] = (name, schema_id)

  data_offset = pos

  // 尝试读取 Footer（最后 32 字节 = 尾标）
  if len(data) >= 32 and data[len-32..len-24] == "DCAP_END":
    footer_data_offset = le_u64(data[len-24..len-16])
    chunk_count       = le_u64(data[len-16..len-8])
    message_count     = le_u64(data[len-8..len])
    index_start = len - 32 - chunk_count * 28
    index = read_index(data, index_start, chunk_count)   // 每条 {offset,start,end,msg_count}
    // 全量：逐 chunk 解压
    messages = []
    for entry in index:
        messages += read_chunk(data, entry.offset, compressor)
    // 或流式：见下 seek_streaming
  else:
    messages = scan_chunks_linear(data, data_offset)       // 线性扫描，容忍截断

  return (schemas, channels, messages, SceneMeta?)          // SceneMeta 见 §12

// 读单个 chunk：校验头 + 解压 + 校验 CRC + 解析消息
read_chunk(data, offset, compressor):
  assert data[offset..offset+10] == "DCAP_CHUNK"
  p = offset + 10
  msg_begin = le_u64(data[p..p+8]); p += 8
  msg_count = le_u32(data[p..p+4]); p += 4
  start_stamp = le_u64(data[p..p+8]); p += 8
  end_stamp   = le_u64(data[p..p+8]); p += 8
  uncompressed_size = le_u32(data[p..p+4]); p += 4
  compressed_size   = le_u32(data[p..p+4]); p += 4
  crc = le_u32(data[p..p+4]); p += 4
  if p + compressed_size > len(data): error("chunk truncated")
  records = decompress(compressor, data[p..p+compressed_size])
  if len(records) != uncompressed_size: error("size mismatch")
  if crc32(records) != crc: error("crc mismatch")
  msgs = []
  q = 0
  repeat msg_count:
    channel_id = le_u16(records[q..q+2]); q += 2
    stamp      = le_u64(records[q..q+8]); q += 8
    len        = le_u32(records[q..q+4]); q += 4
    payload    = records[q..q+len]; q += len
    msgs.append(Message(channel_id, stamp, payload))
  return msgs

// 流式 / seek：按时间定位到第一个 >= target_stamp 的 chunk，解压它并跳过前面更早的消息
seek_streaming(index, target_stamp):
  if index.empty: return None
  idx = partition_point(index, entry.start_stamp <= target_stamp) - 1  // 边界处理
  idx = max(idx, 0)                                                     // 目标早于所有 chunk → 从 0 开始
  return idx

stream_from_chunk(chunk_msg, target_stamp):
  // 流式：先在 chunk 内二分/顺序跳到第一个 stamp >= target_stamp 的消息，再逐条吐出；
  // 该 chunk 读完后，按 index 顺序跳到下一个 chunk 重新 read_chunk。

// 容错线性扫描（无 footer）：从 data_offset 起逐 chunk 读头、解压、解析，直到 magic 不符或越界
scan_chunks_linear(data, data_offset):
  msgs = []; p = data_offset
  while p + 50 <= len(data) and data[p..p+10] == "DCAP_CHUNK":
    // 解析 chunk 头得到 compressed_size
    cs = ... ; msgs += read_chunk(data, p, compressor)
    p += 50 + cs
  return msgs
```

`read_str` / `le_u16` / `le_u32` / `le_u64` 为对应的小端读取；`read_str` 校验 UTF-8，失败应报错或跳过该条。
`crc32` 为 IEEE 多项式 `0xEDB88320` 的位非 CRC（与 zlib 的 `crc32` 相同）。

## 10. `rbag1` 编码规则（规范消息 payload）

- 全小端；**无对齐**；定宽类型固定字节。
- `string` = `u32 len` + UTF-8。
- `bytes` = `u32 len` + 原始字节。
- 序列 `T[]` = `u32 count` + 元素（递归）。
- 定长数组 `[T; N]` = `N * T` 内联，无长度前缀。
- 嵌套消息内联（序列中递归）。
- 未知尾部字段：实现者可选"跳过"或"报错"；本规范的严格参考实现选择报 `DecodeError`。

## 11. 规范消息 wire 布局（`rbag1`）

### 11.1 Time / Header

```text
Time   { sec: i64; nsec: u32 }        # nsec ∈ [0, 1e9)
Header { stamp: Time; frame_id: string }
```

### 11.2 doracap/Imu

```text
Header
orientation    : [f64;4]     # w,x,y,z（可为全 0 表示未知）
orientation_cov: [f64;9]
ang_vel        : [f64;3]
ang_vel_cov    : [f64;9]
lin_acc        : [f64;3]
lin_acc_cov    : [f64;9]
```

### 11.3 doracap/PointCloud

```text
Header
height     : u32
width      : u32
fields     : PointField[]    # { name:string; offset:u32; datatype:u8; count:u32 }
is_bigendian: u8
point_step : u32
row_step   : u32
data       : bytes           # raw point buffer（u32 len + bytes）
is_dense   : u8
```

`datatype`：`1=i8 2=u8 3=i16 4=u16 5=i32 6=u32 7=f32 8=f64`。

点坐标读取方法：第 `i` 点起点 = `i * point_step`；字段 `x / y / z` 按其 `offset + count * datatype_size`
从 `data` 读取（`is_bigendian` 为真时交换字节序；当前为假）。

### 11.4 doracap/PoseStamped

```text
Header
position    : [f64;3]
orientation : [f64;4]       # w,x,y,z
```

### 11.5 doracap/SceneMeta

见 §12。

## 12. 场景元数据 `doracap/SceneMeta`

为了让 viz **只读一个文件就能回放建图**，录制方会在 `.dcap` 中写入**一条** `doracap/SceneMeta`
消息（约定写入频道 `scene`，`stamp = 0`）。viz 读到它即获得建图回放上下文。

若文件没有 `SceneMeta` 通道（旧文件 / 无元数据记录），viz **应回退到通道名约定**：

- `imu` 通道 → `doracap/Imu`；
- `lidar` 通道 → `doracap/PointCloud`；
- `pose` 通道 → `doracap/PoseStamped`；
- 世界系取第一个 `pose` 消息的 `frame_id`，并给出"使用约定而非声明"的警告。

### 12.1 wire 布局（`rbag1`，小端）

```text
world_frame : str                     # 世界系，如 "map"
n_channels  : u32
每条 channel_role:
  name     : str                      # 频道名，如 "lidar"
  role     : str                      # 角色：lidar / imu / pose / tf / ...
  frame_id : str                      # 该频道数据所在坐标系
```

### 12.2 约定

- `world_frame`：建图 / 累加地图所用的坐标系（如 "map"）。
- `role` 为开放字符串，不设枚举。规范角色：`lidar` / `imu` / `pose`。
- `pose` 频道的 `frame_id` 通常等于 `world_frame`（位姿是机体 / 雷达在世界系的位姿）。
- 点云频道的 `frame_id` 通常为 `"lidar"`。
- 可有多个 `lidar` / `imu`；`pose` 通常唯一，也可多个。

## 13. 建图回放行为契约

下列行为是 viz 消费 `.dcap` 时**推荐且确定**的规则，实现时照做即可保持一致体验。

### 13.1 播放器调度

- 基准：首条消息 `stamp` 记为 `t0`。当前模拟时间 `sim = t0 + (now - start) * rate`。
- `rate`：0 = 不限速（尽量快），1 = 实时，>1 = 加速，<1 = 慢放。
- `loop`：到末尾回绕，并**重算基准**使 `sim` 单调，保证首尾 `stamp` 不出现倒退。
- `seek(ts)`：把播放头定位到**首个 `stamp >= ts` 的消息**，并重新锚定「现在 = 目标时刻」。
- `pause` / 单步：等价于把 `rate` 置 0 或不推进，由调用方轮询。

### 13.2 跨通道合并

所有消息按 `stamp` 升序合并为一条时间线；**平局保持写入顺序**（稳定排序）。这样能保证
`imu` / `lidar` 相对顺序与录制一致，不会破坏"先 IMU 后点云帧"的消费假设。

### 13.3 位姿配帧（点云 → 位姿）

每个 `lidar` 帧需配一个位姿才能变换到世界系。推荐规则：

1. 若该点云帧有 `pose` 通道且在时间轴上接近，取**与 `lidar.stamp` 时间差绝对值最小**的 `pose`。
2. 若无 `pose` 通道，则只能渲染"机体姿态"或跳过建图（不累加地图）。

> 说明：这是**消费策略**而非容器强制。`.dcap` 只保证 `imu` / `lidar` / `pose` 都在时间轴上正确调度；
> 究竟"哪个 pose 配哪帧点云"由 viz 依据 `stamp` 对齐。上述规则对 FAST-LIO 录制（pose.stamp 略晚于
> 对应点云帧）是精确对应的。

### 13.4 四元数 → 旋转矩阵（Hamilton，w 在前）

设 `q = [w, x, y, z]`（单位四元数）。从机体 / 传感器系到世界系的旋转矩阵：

```text
R = [ 1-2(y²+z²)   2(xy-wz)     2(xz+wy)   ]
    [ 2(xy+wz)     1-2(x²+z²)   2(yz-wx)   ]
    [ 2(xz-wy)     2(yz+wx)     1-2(x²+y²) ]
```

给定 `PoseStamped { position: p, orientation: q }`，把某点 `x` 从该帧坐标系变换到世界系：

```text
x_world = R * x + p
```

(若存在外参 `T_pose_lidar`，则 `T_world_lidar = T_world_pose * T_pose_lidar`；本期无外参约定。)

### 13.5 逐帧累加与 seek 重建

- 播放头前进时，把每个 `lidar` 帧的 `x_world` 累加到增量地图（建议体素 / 下采样控存储）并渲染当前帧。
- `seek` 回退时，从头部重新累加到目标时刻，保证地图与目标时间一致。
- `imu` / `pose` 通道也按时间线渲染（轨迹、姿态动画）。

## 14. 第三方实现自检清单

实现完读取器后，用它逐项自检（用 §15 的 golden 文件）：

1. 能正确识别 magic / version / flags / compressor，并对未知大版本或未知 compressor 报错。
2. 能解析出 schema 列表：本 golden 含 4 个 schema（SceneMeta / Imu / PointCloud / PoseStamped）。
3. 能解析出 channel 列表：本 golden 含 4 个 channel（scene / imu / lidar / pose）。
4. 能从 `data_offset` 解压出 1 个 chunk、4 条消息，且 `stamp` 分别为 0 / 1e9 / 2e9 / 2e9 纳秒。
5. 每条 payload 长度与 §15 一致（scene=82、imu=315、lidar=101、pose=75 字节）。
6. 每条 `type_name` 对应的 payload 能用 §11 wire 布局解码，且数值与 §15 一致。
7. 能读取 Footer，得到 `chunk_count=1`、`data_offset` 与 `message_count=4`，以及 1 条 index 条目。
8. 删掉 Footer（最后 32 字节）后仍能线性扫描出 4 条消息（容错）。
9. 用真实 `live1.dcap`（无 SceneMeta）验证：能回退到通道名约定并警告。
10. 能按 §13 完成 seek / rate / loop 并逐帧累加出地图。

## 15. 跨语言 golden 样例

最小的、含三个规范消息 + 场景元数据的参考文件：**495 字节**（v2，deflate 压缩）。

> 说明：v2 中消息被压缩进一个 chunk，因此 golden 验证的是**容器结构 + 解压后的消息布局**；
> 压缩块字节本身随压缩库/版本可能变化，不作为对拍依据。下面的 `total`/`chunk` 数字来自当前
> 参考实现（deflate 压缩）。

可用参考实现重新生成对拍文件：

```bash
cargo run -p doracap --example gen_golden        # 打印各区段 hex 与消息长度
```

### 15.1 Header（17 B）

```text
23 44 43 41 50 02 00 00 00 00 00 00 00 02 00 00 00
```

即 `"#DCAP"` + `version=2` + `flags=0` + `compressor=2(deflate)`。

### 15.2 Schemas 段（129 B）

```
count=4
id=1  type_name="doracap/SceneMeta"    encoding="rbag1"
id=2  type_name="doracap/Imu"          encoding="rbag1"
id=3  type_name="doracap/PointCloud"   encoding="rbag1"
id=4  type_name="doracap/PoseStamped"  encoding="rbag1"
```

hex：

```text
04 00 00 00
01 00 11 00 00 00 64 6f 72 61 63 61 70 2f 53 63 65 6e 65 4d 65 74 61 05 00 00 00 72 62 61 67 31
02 00 0b 00 00 00 64 6f 72 61 63 61 70 2f 49 6d 75 05 00 00 00 72 62 61 67 31
03 00 12 00 00 00 64 6f 72 61 63 61 70 2f 50 6f 69 6e 74 43 6c 6f 75 64 05 00 00 00 72 62 61 67 31
04 00 13 00 00 00 64 6f 72 61 63 61 70 2f 50 6f 73 65 53 74 61 6d 70 65 64 05 00 00 00 72 62 61 67 31
```

### 15.3 Channels 段（53 B）

```
count=4
id=1  name="scene"  schema_id=1
id=2  name="imu"    schema_id=2
id=3  name="lidar"  schema_id=3
id=4  name="pose"   schema_id=4
```

hex：

```text
04 00 00 00
01 00 05 00 00 00 73 63 65 6e 65 01 00
02 00 03 00 00 00 69 6d 75 02 00
03 00 05 00 00 00 6c 69 64 61 72 03 00
04 00 04 00 00 00 70 6f 73 65 04 00
```

`data_offset = 199`。

### 15.4 消息与 payload

| # | channel | stamp(ns) | payload_len | 说明 |
|---|---|---|---|---|
| 0 | scene (id=1) | 0 | 82 | SceneMeta |
| 1 | imu (id=2) | 1000000000 | 315 | Imu；头 19B + 296B 数组 |
| 2 | lidar (id=3) | 2000000000 | 101 | PointCloud，1 点 |
| 3 | pose (id=4) | 2000000000 | 75 | PoseStamped |

`message_count = 4`，总 **495 字节**。

### 15.5 Chunk 与 Footer

这 4 条消息被写进**一个 chunk**（`data_offset=199` 起）。Chunk Header（50 B）：

```text
magic "DCAP_CHUNK" | msg_begin=0 | msg_count=4 | start_ns=0 | end_ns=2000000000 | uncompressed=629 | compressed=186 | crc=0d6f2c31
```

- 解压后 `records` 629 字节，由 4 条消息记录（各 14 字节头 + payload）组成：
  `scene(14+82) + imu(14+315) + lidar(14+101) + pose(14+75) = 629`。
- 压缩后 186 字节（deflate/zlib）。
- Footer：末尾 32 字节尾标 = `"DCAP_END" + data_offset(199) + chunk_count(1) + message_count(4)`；
  前面紧跟 1 条 index 条目（28 字节）：`{offset=199, start_ns=0, end_ns=2000000000, msg_count=4}`。

**SceneMeta payload（82 B）**：

```text
03 00 00 00 6d 61 70                          # frame "map"   (world_frame)
03 00 00 00                                   # n_channels = 3
03 00 00 00 69 6d 75 03 00 00 00 69 6d 75 03 00 00 00 69 6d 75       # imu   @ imu
05 00 00 00 6c 69 64 61 72 05 00 00 00 6c 69 64 61 72 05 00 00 00 6c 69 64 61 72   # lidar @ lidar
04 00 00 00 70 6f 73 65 04 00 00 00 70 6f 73 65 03 00 00 00 6d 61 70 # pose  @ map
```

**PoseStamped payload（75 B，p=[1,2,3], q=[1,0,0,0]）** 关键字节：

```text
02 00 00 00 00 00 00 00 00 00 00 00 03 00 00 00 6d 61 70   # Header: sec=2,nsec=0,"map"
00 00 00 00 00 00 f0 3f                                     # pos[0] = 1.0
00 00 00 00 00 00 00 40                                     # pos[1] = 2.0
00 00 00 00 00 00 08 40                                     # pos[2] = 3.0
00 00 00 00 00 00 f0 3f                                     # q[0]   = 1.0 (w)
00 00 00 00 00 00 00 00 (×3)                                # q[xyz] = 0.0
```

> 完整文件（含 imu / lidar 全部字节）由 `gen_golden` 生成并打印，可直接对拍。

## 16. 向后兼容 / 版本

- 容器级：`Header.version`。读取器拒绝高于其支持版本的大版本；`version=1`（旧线性格式，无压缩）仍可读，
  但 `version=2` 使用 chunk+压缩+索引（本规范）。
- 压缩：读取端以 header 的 `compressor` 分派；`0`/`2` 应实现，`1`(zstd) 为保留，未实现时可报错或降级跳过。
- schema 级：规范消息只追加、不删改；破坏性变更用新 `type_name`（如 `doracap/Imu2`）。
- 旧文件：无 `SceneMeta` 通道仍可读，viz 回退到通道名约定（§12）。
- 读取器应宽容：截断尾巴 / 缺 Footer / 未知频道 schema 时尽量降级而非整体失败。

## 关联产物

- `doracap-core`：`SingleFileWriter` / `SingleFileReader`（容器）、`Recorder` / `Player`（录制 / 回放）。
- `doracap-msgs`：规范消息 + `rbag1`。
- `doracap-fastlio::LioRecorder`：一遍录制（sensor + pose + `SceneMeta`），产出自洽建图回放源。
- CLI：`doracap info <file>`（含 scene 元信息）、`doracap play <file> [--rate] [--seek] [--json] [--show cmd]`。
- 对拍工具：`cargo run -p doracap --example gen_golden`。
