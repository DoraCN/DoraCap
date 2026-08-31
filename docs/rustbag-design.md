# RustBag 设计方案（独立、与建图/导航库解耦的录制与回放库）

> 本文产出：针对“做一个**不绑定任何建图/导航系统**的通用 Rust bag（录制 + 回放）”的完整设计。
> 目标：任意库只要满足 rustbag 的几条要求，就能用 rustbag 做录制与回放。
> 文中以 FAST-LIO 的 Rust 移植版作为**第一个接入方（consumer）**来回推设计，但 rustbag 本身
> **不包含、不依赖任何 FAST-LIO 类型**。

---

## 0. 结论摘要（TL;DR）

- rustbag **不服务特定建图/导航库**：它是一个**类型无关的容器 + 时间调度引擎**。
- rustbag 唯一认识的消息结构是四元组：**“主题(topic) + 时间戳(timestamp) + 模式(schema) + 字节(payload)”**。
- 任何库要使用 rustbag，只需满足**一份最小契约**（见 §4）：给消息一个主题名、一个单调递增时间戳、
  一份可序列化的字节表示，并注册一份 schema 描述。其余（消息是什么、单位是什么、怎么解码）全部由
  **调用方**负责，rustbag 一概不管。
- 因此 rustbag 是**独立仓库/独立发布的通用库**。接入 FAST-LIO（或任何别的 LIO/SLAM/导航库）时，
  只需要在**对方项目里**写一层很薄的“胶水适配器”（把 rustbag 的字节流转成它自己的 `DataSource`），
  而**不把任何库的类型塞进 rustbag**。
- 类比：rustbag ≈ “ROS 的 rosbag/rosbag2 的通用内核”，但把类型系统、QoS、时钟这些 ROS 负担彻底抽掉，
  只留下**以字节为载体的时间序列容器**。
- **最终目标（单文件原则）**：录制时把 schema 注册表、所有频道/消息、索引、场景元数据、压缩
  **全部写进一个 `.rbag` 文件**；回放读取该 `.rbag`，即可交给独立的 RViz 类工具进行可视化回放。
  详见 §6。

---

## 1. 为什么必须解耦（而不是当 FAST-LIO 的兄弟 crate）

上一版我建议“与 `fast-lio-driver` 平级的兄弟 crate”，那是**以 FAST-LIO 为中心**的做法——直接依赖
`fast_lio::data_source::DataSource` 和 `fast_lio::types::SensorData`。这确实简单，但带来两个问题：

1. **rustbag 和 FAST-LIO 的类型强绑定**：`rustbag` 一旦依赖 `SensorData`，就永远只能服务 FAST-LIO；
   换一个 SLAM/导航库，就得重写一个“同样但不同”的 bag。
2. **边界越界**：建图导航库是“高内聚的算法层”，而 bag 是“通用的持久化层”。通用的东西不该知道
   特定算法的类型。

**解耦的核心手段**：rustbag 不去理解“IMU 是 f64 秒、激光有逐点 offset_time、位姿是 quaternion”，
它只做三件通用的事：

- **存**：把一段字节按“主题 + 时间戳 + schema”写进自描述容器。
- **读/回放**：按时间戳顺序把这些字节以正确节奏吐出来。
- **工具**：info / 时间窗口 / loop / rate / seek / 转换 / 压缩。

“字节里装的是什么、时间戳是什么意思、怎么解码”，全部交给调用方。这就是“满足要求即可用”的全部含义。

---

## 2. 解耦的核心抽象：rustbag 只认识一个四元组

rustbag 内部的“消息”定义为：

```rust
pub struct Message<'a> {
    pub channel:  &'a str,     // 主题名，如 "imu"、"lidar"（rustbag 不理解其语义）
    pub stamp:    Timestamp,   // 回放调度的依据（整数，带单位，见 §5）
    pub schema:   &'a Schema,  // 自描述元数据（类型名、编码、可选字段表）
    pub payload:  &'a [u8],    // 调用方序列化后的字节，rustbag 不做任何解码
}
```

- rustbag **永远不**命中 “`ImuRaw` / `AviaMsg` / `SensorData` / `LioConfig`” 等具体类型。
- 这正是 ROS2 里 `rmw_serialized_message_t`（串行化字节）+ `message_definition`（类型描述）的
  思路，但 rustbag 把“有没有 /clock、有没有 QoS、是不是 DDS”这些 ROS 包袱全部去掉。

---

## 3. 分层架构：rustbag 与消费者之间隔着一层“胶水”

```
┌─────────────────────────────┐
│   rustbag（通用内核）          │   ← 独立仓库，零领域类型
│  - 容器/索引（chunk+index）    │
│  - 时间调度引擎（rate/loop）    │
│  - schema 注册表、工具         │
└──────────────┬──────────────┘
               │ 只通过 Message 四元组交互
┌──────────────▼──────────────┐
│  胶水适配器（在消费者项目里）     │   ← 只在这里知道“两种世界”的类型
│  rustbag-fastlio：bytes ⇄ SensorData │
│  序列化 + 时间戳提取 + DataSource 包装 │
└──────────────┬──────────────┘
               │ 实现 consumer 自己的抽象
┌──────────────▼──────────────┐
│  FAST-LIO / lidar-nav / 别的库 │
│  （它们完全不知道 rustbag 的存在） │
└─────────────────────────────┘
```

关键点：

- **依赖方向单向、无环**：`rustbag` 谁都不依赖；`glue` 依赖 `rustbag` + `fast-lio`；
  `fast-lio` 不依赖 rustbag。
- **rustbag 可以被任何库复用**，不只 FAST-LIO。
- **换库的成本**：只在对方项目里写一个新 glue（一个 `DataSource` 适配器 + 消息序列化），
  rustbag 内核一行不改。

---

## 4. “满足 rustbag 要求”的契约（调用方必须提供的四样东西）

这是 rustbag 对使用者提出的**最小要求**，也是“不管用什么建图导航系统，都能用”的判据：

1. **给每条消息一个主题名（channel）**——例如 `imu`、`lidar`、`odom`、`pose`。
2. **给每条消息一个单调递增的时间戳（stamp）**——rustbag 用它来排序和安排回放节奏。
3. **给每条消息一种字节表示**——由调用方定义（serde/自定义二进制/CDR/……），并注册一个
   编码与解码手段。
4. **（推荐）提供一份 schema 描述**——类型名、编码方式、可选字段表；用于 `info`/工具自省。
   即使不提供，rustbag 也能按“不透明字节”录制与回放，只是工具无法自动解析内容。

反过来说：**只要满足了这四样，任何库都能被 rustbag 录制和回放**。这正是你想要的“解耦”。

### 4.1 时间戳的归属（很重要）

不同库把时间放在不同地方：

- FAST-LIO：`ImuRaw.stamp` / `AviaMsg.stamp` / `StandardMsg.stamp`，都是 `f64` 秒；
- 别家：可能用 header 的 `sec/nsec`、也可能是点在消息体里的 `offset_time`；
- 单个消息内部可能还有“次一级时间戳”（如激光点相对当前帧的 `offset_time`）。

rustbag **不猜**这些。它在**写入时**由调用方告诉它“调度这条消息用哪个时间”；在**播放时**按这个
时间调度。至于消息体里还带不带一个 header、header 里是什么、逐点时间怎么用——rustbag 不关心，
只当成字节。**这样既保真（逐点时间原样在字节里），又不耦合（rustbag 不解析它）。**

> 注意：这里有个取舍——如果将来想实现“按 header 里内嵌时间去同步/重采样”，就需要 rustbag 能
> 从字节里读出时间，那会引入“特定 header 约定”。建议作为**可选扩展**，而不是默认契约，
> 以保住解耦。

---

## 5. 时间轴 / 时钟引擎（rustbag 的核心职责之一）

rustbag 不发布 `/clock`，也不强制“模拟时间”，但它**天生要负责按时间戳给消费者安排节奏**。这是它
存在的意义，也是通用部分。

### 5.1 时间戳表示

- 建议用**整数**（如 `u64` 纳秒，或 `sec + nsec`），避免 `f64` 秒在大时间值下丢精度、并便于做整数
  调度与比较。
- 容器头记录一个 `time_base` / `time_scale`，调用方在写入时把它的 `f64` 秒或 `sec/nsec` 换算进来。
- 回放调度用“相邻消息的整数时间差”，最小单位可配置（`ns`/`us`/`ms`），不受浮点误差影响。

### 5.2 回放引擎能力

- **按原速回放**：写出消息时，根据相邻 `stamp` 差 sleep，保持“录制时的时间关系”。
- **倍速**：`rate` 缩放时间差（`0` = 不限速灌数据，`>0` = 按倍速）。
- **循环 / 回绕**：到末尾回到开头，保证首尾 `stamp` 单调（循环时通常重算基准）。
- **暂停 / 单帧步进**：由消费者控制推进（等价于 ROS 的 space/pause）。
- **seek / 时间窗口 / start-offset**：靠索引在容器内跳到目标时间。
- **非阻塞读取**：暴露 `try_next()` 风格的接口，让调用方在“无数据”时能轮询自己的 Ctrl-C / 超时，
  与 FAST-LIO 的 `DataSource::try_next` 语义对齐；同时也支持“不限速快进”。

### 5.3 是否需要“时钟源”？

FAST-LIO 自己用 `stamp`，不需要外部时钟；但**别家系统可能需要“当前回放时间”**。建议 rustbag 提供
一个**可选**的 `PlaybackClock`（返回当前回放进度），消费者想用就用；这是附加能力，**不是核心契约**，
不会因为一个库不想要而被强加。

---

## 6. 存储格式：单一 `.rbag` 文件（自描述 + 可插拔）

> **最终目标（单文件原则）**：一切（schema 注册表、所有频道/消息、索引、场景元数据、压缩）都录进
> **一个 `.rbag` 文件**。回放时读取该文件，即可直接交给独立 RViz 类工具进行可视化回放。
>
> `.rbag` = 单一自描述、可 seek、可压缩的自包含文件；建议底层用 MCAP（本就是单文件、自描述、
> 分块、带索引、可压缩），rustbag 在其上叠加 `.rbag` 语义与规范消息。

因为 rustbag 不认识领域类型，它**必须自描述**才能在“不知道调用方类型”的情况下被别人用工具打开。

### 6.0 单文件原则（最终目标）

- **一个 `.rbag` = 一份完整录制**：schema 注册表 + 频道/主题清单 + 消息 + chunk 索引 + 场景元数据
  + 压缩，全部在**一个文件**内，便于分发、备份、seek。
- **回放端**：读 `some.rbag` → 还原四元组流与元数据 → 交给独立 viz 工具渲染。
- **首选实现**：MCAP（单文件、自描述、带 schema 与索引、zstd/lz4 压缩、可与 ROS2/Foxglove 互通）；
  rustbag 在其上叠加 `.rbag` 文件头语义（profile = `"rustbag"`、library = 工具版本、规范型约定）。
- **可选后端**：sqlite3、自定义 chunked 等；凡能产出“单文件”均可接入，核心只依赖 `storage` 插件接口。

> **澄清：rosbag2 的形态 ≠ rustbag 的单文件 `.rbag`。**
>
> - `rosbag2` 的“录制”是一份**目录**：`metadata.yaml` + 一个或多个 `.mcap`（或 `.db3`）数据文件
>   （`relative_file_paths` 可多个，分卷）。`metadata.yaml` 记录版本、storage 类型、文件列表、
>   topic 元数据、时长、消息数。
> - **`sqlite3` 与 `mcap` 是并列的两个存储后端**，不是包含关系：
>   `sqlite3` = 真正的 SQLite 数据库（`.db3`，含 `topics`/`messages` 表）；
>   `mcap` = 自定义二进制容器（单文件，含 header/schema/channel/message/chunk/index/metadata）。
>   rosbag2 默认早期为 `sqlite3`，**Iron 起改为 `mcap`**。
> - rustbag 的 `.rbag` 用的是 MCAP 的“**单文件自包含**”能力：把主题列表、时间范围、场景元数据
>   放进 MCAP 自己的 `header`(profile=`"rustbag"`)、`metadata`、`statistics` 记录，
>   **不需要 sidecar `metadata.yaml`**。因此 `.rbag` 是一个真正的单一文件，比 rosbag2 的目录式更紧凑。

### 6.1 容器需要记录

- **schema 注册表**：每个主题的类型名、编码方式、（可选）字段描述（类型名、字段名、字段类型）。
- **频道/主题清单**：主题名、数据类型、消息数、时间范围、字节数。
- **chunk + index**：分块存储消息（便于 seek 与压缩）；索引记录“时间 ↔ 字节偏移”，支撑时间窗口查找。
- **版本号**：文件格式版本，用于向后兼容。

### 6.2 存储后端做成插件

- **默认：`storage-mcap`（即单文件 `.rbag`）**。MCAP 本身是自描述、分块、带 schema 注册表与索引的
  跨语言**单文件**容器，已经被
  ROS2 rosbag2、Foxglove Studio、PlotJuggler 广泛采用。rustbag 采用它能：
  - **一个文件就装下全部录制**（契合“单文件 `.rbag`”目标）；
  - 与既有 ROS2 bag / 可视化工具**互通**；
  - 不用重造“自描述 + 索引”轮子；
  - 未来可直接读现成的 ROS bag 数据集。
- **可选：自定义后端**（如 sqlite3、自定义 chunked 格式），抽象成 `storage` 插件，按需替换；
  核心只要求“支持单文件读/写 + 元数据 + 索引”。

### 6.3 序列化 / codec 做成插件

- 默认支持 `serde`（bincode/postcard 等紧凑编码）；也允许调用方用任意自定义编码
  （如 DDS CDR、vendor 二进制）。
- codec 是**调用方在接入时提供**的，rustbag 只是“存字节 + 记编码标识”，不做解码。

### 6.4 为什么这样不会“损伤保真”

FAST-LIO 关心的逐点时间（Avia `offset_time`、Standard `time`）都在消息字节里；rustbag 把它们
当作 opaque 字节存起来，回放时原样吐回。**语义/单位/换算完全由 FAST-LIO 自己解释**，rustbag 不求甚解，
也就不会因为“不认识”而丢字段。

---

## 7. 数据保真的责任划分（谁负责什么）

| 方面 | 负责方 |
|---|---|
| 主题名、所属频道 | 调用方在写入时指定 |
| 调度时间戳（单调、单位） | 调用方提供，rustbag 只负责排序/调度 |
| 消息字节表示（序列化） | 调用方（serde / 自定义编码） |
| 逐点时间、单位换算、帧内语义 | **调用方**（rustbag 存字节，不解码） |
| 容器/索引/压缩/seek/速率/循环 | **rustbag** |
| 类型/单位自省（info/工具） | rustbag 用 schema 元数据，调用方提供 schema |

---

## 8. 接入 FAST-LIO：薄胶水层（不进入 rustbag）

FAST-LIO 想用 rustbag，只做两件事，且**都在 FAST-LIO 侧的 glue 里**：

### 8.1 录制端（Record）

把现有的 `DataSource` 输出流（`SimSource` / `LivoxSource` / 未来驱动）序列化成字节并写入 rustbag：

```rust
// 在 FAST-LIO 侧定义“如何把 SensorData 编码成字节 + 取时间戳”
fn encode(s: &SensorData) -> (channel, stamp, Schema, Vec<u8>) {
    match s {
        SensorData::Imu(imu)      => ("imu",   ts_to_ns(imu.stamp), SCHEMA_IMU,  bincode(&imu)),
        SensorData::LidarAvia(m)  => ("lidar", ts_to_ns(m.stamp),   SCHEMA_AVIA, bincode(&m)),
        SensorData::LidarStandard(m) => ("lidar", ts_to_ns(m.stamp), SCHEMA_STD,  bincode(&m)),
    }
}
```

### 8.2 回放端（Play）

rustbag 把字节吐出来，胶水层解码并把它们喂回 FAST-LIO 自己的 `DataSource`：

```rust
impl DataSource for BagSource {
    fn next(&mut self) -> Option<SensorData> {
        // 从 rustbag 的 play 流读到一条 Message，再 decode 成 SensorData
        self.play.next_message().map(decode)
    }
    fn try_next(&mut self) -> Result<Option<SensorData>, NonBlocking> {
        // 对应 rustbag 的非阻塞读
    }
}
```

**FAST-LIO 核心库一行不改**，因为它依然在面对它的 `DataSource`；只是这次 `DataSource` 的底层来源
从“硬件/模拟”变成了“rustbag 回放”。这一点与现在的 `SimSource` / `LivoxSource` 完全平级。

---

## 9. 仓库 / CRATE 结构

### 9.1 rustbag：独立的通用库（自己一个仓库）

```
rustbag/                       # 独立 repo，crates.io 可发布
  core/        # 容器 + 索引 + 时间调度引擎 + schema（不依赖任何领域库）
  storage-mcap/    # 单文件稳定格式（默认；产出 .rbag）
  storage-custom/  # 可选：sqlite3 / 自定义 chunked 单文件
  codec-serde/     # 可选：serde(bincode/postcard) codec
  cli/         # 命令：record / play / info / convert / reindex（产物为 .rbag 单文件）
```

rustbag 对外的依赖都是**通用**的：`serde`、容器/索引相关、`chrono`/`std::time`，以及可选 codec 插件。
**没有任何** LiDAR / IMU / SLAM / 导航依赖。最终产物是**一个 `.rbag` 单文件**，
回放时可直接交给独立 RViz 类工具。

### 9.2 FAST-LIO 侧：薄胶水 crate

在 FAST-LIO **workspace** 里新增一个很小的 crate（如 `crates/rustbag-fastlio`），依赖
`rustbag`（path/git）+ `fast-lio`，只做“bytes ⇄ SensorData + 时间戳提取 + DataSource 包装”。
两全其美：**rustbag 保持通用，FAST-LIO 用户零改动地接入。**

```
fast-lio workspace/
  crates/fast-lio/
  crates/fast-lio-driver/
  crates/rustbag-fastlio/     # 新增：胶水，仅此 crate 同时了解 rustbag 与 fast-lio
  crates/fast-lio-app/
  crates/nav-app/
```

---

## 10. 与 ROS 原生 rosbag 的对比，以及 rustbag 的“通用契约”清单

rustbag 复用了 ROS rosbag 的“通用内核”思想，但把 ROS 特有的部分砍掉：

| ROS 概念 | rustbag 处理方式 |
|---|---|
| 消息类型系统 | 不内置；用“字节 + schema”自描述，调用方负责解码 |
| Topics | 保留为 `channel`（字符串），rustbag 不解析语义 |
| `/clock` / `use_sim_time` | 不内置；timestamps 由调用方提供；可选的 `PlaybackClock` 供需要者用 |
| QoS / DDS | 无；不涉及 |
| TF / tf_static | 不关心；是“字节里的位姿数据”，调用方解释 |
| 逐点/消息内时间 | rustbag 存字节，逐点时间原样保留，由调用方解释 |
| record / play / info | rustbag 提供（通用） |
| chunk + index / seek | rustbag 提供 |
| 压缩（zstd/lz4） | rustbag 提供，作用于 chunk |

### “满足要求即可用”的判据（一句话）

> 只要你能给每条消息一个主题名、一个单调递增时间戳、一份字节表示和一份（可选的）schema，
> rustbag 就能录制、回放、seek、循环、变速，不管你后面跑的是 FAST-LIO、Cartographer、
> Odom、还是纯导航。

---

## 11. 里程碑（建议实现顺序：先 P0，ROS 兼容整体后置）

> 决策已定：**当前不做 ROS/ROS2 兼容**（无必须直接回放的存量 ROS 数据）。先做 P0 核心主链路；
> `rustbag-ros` 作为“后置可选层”保留设计，仅在 §15.5 触发信号出现时再启动。

1. **内核**：定义 `Message`/`Timestamp`/`Schema`/`Registry`；单文件 `.rbag` 容器（chunk + index + 压缩）。
2. **`storage-mcap`**：用 MCAP 产出**单文件 `.rbag`**（内容 = `rustbag-msgs` 规范消息 + compact 编码，
   非 CDR；profile=`"rustbag"`）。
3. **`rustbag-msgs`**：`PointCloud` / `TransformStamped` / `PoseStamped` / `Header` / `Time`
   共享类型 + 文档化字节布局。
4. **record**：从任意 `DataSource` 流式写入，产出**一个 `.rbag` 单文件**。
5. **play**：时间调度引擎（rate/loop/seek/非阻塞），读取 `.rbag` 喂给消费方 / 独立 viz。
6. **工具**：`info` / `convert` / `reindex`；`rustbag-fastlio` glue 接入 FAST-LIO。
7. **（后置 / 可选）** 读 ROS2 mcap / ROS1 `.bag`、写回 MCAP；仅在触发信号出现时启动。

---

## 12. 兼容 rosbag（ROS1 `.bag` / ROS2 sqlite3 / MCAP）——【后置 / 可选，按需启用】

> **状态：后置 / 可选。** 当前决策为“不做”，本节作为已论证的**将来可启用设计**保留；
> 仅当 §15.5 的触发信号出现时才启动实施。

### 12.1 结论：兼容很值，但要做成“可选兼容层”，不进 rustbag 核心

“兼容 rosbag”讲的是**与 ROS 的“容器格式 + 消息格式”互通**，这属于“传输/容器”层的问题，与
“是不是某家建图导航库”**正交**。所以它**不会破坏 rustbag 的通用性**，只要把它做成核心之外的
可选模块即可：

```
rustbag(核心, 通用四元组)
  ├─ storage-mcap         ← ROS2 rosbag2 现在常用 MCAP，天然互通
  ├─ storage-sqlite3      ← 读取旧版 ROS2 rosbag2
  ├─ storage-rosbag1      ← 读取 ROS1 经典 .bag
  ├─ codec-ros2-cdr       ← ROS2 消息 = CDR 字节
  ├─ codec-ros1           ← ROS1 消息 = ROS1 序列化字节
  └─ ros-types            ← 针对常用传感器消息的手写解码（见 12.4）
```

核心依然不认识 `sensor_msgs/Imu` / `livox_ros_driver/CustomMsg`；这些“认识”都集中在
`rustbag-ros` 这一层。

### 12.2 “兼容 rosbag”要分三层，别一锅端

| 层级 | 含义 | 工作量 | 优先级 |
|---|---|---|---|
| **存储兼容** | 能读写同一种容器文件（MCAP / `.db3` / `.bag`） | 低 | ✅ 先做 |
| **消息兼容** | 能按 ROS 的消息 schema 解码/编码字节（CDR / ROS1 序列化） | 高 | ✅ 核心瓶颈 |
| **语义兼容** | 把 ROS 主题/消息映射到消费方的数据模型（如 FAST-LIO 的 `SensorData`），处理时间单位、点云字段 | 中 | 由 glue 负责 |

建议：**存储兼容 + 消息兼容**做进 `rustbag-ros` 层；**语义兼容**继续留在 FAST-LIO 的 glue 里，
核心保持不带任何方向性。

### 12.3 为什么 MCAP 是关键桥梁

- ROS2 rosbag2（Iron 之后）默认存储就是 **MCAP**：自描述、带 schema 注册表、chunk + 索引、支持压缩。
- rustbag 核心**默认就用 storage-mcap**，于是“读 ROS2 MCAP bag”几乎白拿——只需再补一个
  **CDR codec** 和**类型描述**即可。
- 读旧版 ROS2 的 sqlite3 `.db3`：解析三个表（topics / messages / metadata）即可，CDR 解码共用同一套 codec。
- 读 ROS1 经典 `.bag`：需要解析 chunk/connection/index 记录 + ROS1 消息序列化。这是最大的一块独立工作。

> **写回**：如果要让 Foxglove / `ros2 bag play` 打开 rustbag 的录制，用 MCAP + CDR 就能直接对上；
> 若要写回 ROS1 `.bag`，工作量更高，建议**只做读、暂不做写**。

### 12.4 最大的难点：消息解码（一个 schema 注册表就能兜住常规做法）

真正的“难”来自消息层，因为 rustbag 不认识 ROS 消息。正确做法是：**用 rustbag 的 `Schema`
承载 ROS 的类型元数据**，再配合**手写解码器**处理少数常用消息类型：

```rust
pub struct Schema {
    pub type_name: String,          // "sensor_msgs/msg/Imu" / "livox_ros_driver2/msg/CustomMsg"
    pub encoding:   Encoding,       // Cdr | Ros1 | ... 
    pub message_definition: String, // .msg 文本（ROS2 type description 也是这个）
    pub md5: Option<String>,        // ROS1 用
}
```

需要手写解码的“建图常用”消息（远少于 ROS 全量消息）：

- `sensor_msgs/msg/Imu`：线性结构，CDR/ROS1 都好解。
- `sensor_msgs/msg/PointCloud2`：结构**不含动态数组**，`fields[]` 描述每个点内各字段的
  offset/datatype，`data` 是 `point_step × width` 的缓冲。按 `fields` 里 `x/y/z/intensity/time/ring`
  的名称取即可；`time` 字段的**单位**随发布者而异（常见秒/毫秒），要按数据来源设定 `timestamp_unit`。
- Livox `CustomMsg`（`livox_ros_driver` / `livox_ros_driver2` / `livox_ros_driver3`）：
  `offset_time(µs)` 逐点时间戳，直接映射到 FAST-LIO 的 `AviaMsg`。注意 **ROS1 与 ROS2 的包名/命名空间不同**。
- `nav_msgs/Odometry`、`geometry_msgs/PoseStamped`、`sensor_msgs/...` 等其它话题：作为“字节 + schema”
  原样透传即可，消费方按需解码。

> 结论：**不要去做“能解所有 ROS 消息的动态反序列化器”（那相当于重写 rosidl dynamic typesupport）**。
> 只需要为建图用到的几类消息写手写解码器，其余按“不透明字节”透传。成本可控，且完全不伤通用性。

### 12.5 时间语义：rosbag 兼容必须澄清“用哪个时间调度回放”

这是最容易做错、也最影响建图一致性的点：

- ROS 里同一消息有**两个时间**：记录时间（`log/record time`）和消息头里的传感器时间（header `stamp`）。
- `rosbag play` 的调度基准是 **header `stamp`**（无 header 才退回记录时间）。
- ROS2 MCAP 额外存了 `log_time` 和 `publish_time`；真正的语义时间仍在消息 payload / header 里。

所以 rustbag 需要给每个频道一个 **`time_policy`**：

```rust
enum TimePolicy {
    HeaderStamp,      // 读 payload 里的 header stamp（默认，匹配 rosbag）
    LogTime,          // 用记录/log 时间（无 header 消息）
    Custom,           // 由消费方决定
}
```

这会让“时间提取”稍微侵入消息字节（读 header 需知道类型），因此这个逻辑放在 rustbag-ros / codec
里，**核心只管拿到的调度时间戳**，保持四元组抽象不变。

### 12.6 `tf` / `tf_static` 等话题

核心也把它们当作普通频道（字节 + schema）存储与回放，不做解析。若消费方（如用到 TF 的 SLAM）
需要，在 glue 层按需解码即可；FAST-LIO 不消费 TF，可忽略。

### 12.7 落地顺序建议

1. `storage-mcap` 读写（默认）。至此 **ROS2 MCAP bag 即可读**。
2. `codec-ros2-cdr` + 手写 `Imu`/`PointCloud2`/`CustomMsg` 解码，先接入 FAST-LIO glue 打通链路。
3. `storage-sqlite3` 读旧版 ROS2 bag（共用 CDR codec）。
4. `codec-ros1` + `storage-rosbag1` 读经典 ROS1 数据集（FAST-LIO 很多老数据集是 ROS1 Livox CustomMsg）。
5. （暂缓）写回 ROS1 `.bag`；MCAP + CDR 的写回已能对接 Foxglove / `ros2 bag play`。

> 一句话：**核心依然通用；`rustbag-ros` 这一层负责把 ROS 的“容器 + 消息”翻译成 rustbag 的
> 四元组；语义映射留在消费方的 glue。** 这样既拿到 rosbag 时代的存量数据，又不破坏
> “不绑定建图/导航库”的初衷。

---

## 13. rustbag-ros 实现细节（字节级拆解）

> **状态：后置 / 可选设计参考。** 本节是 `rustbag-ros` 将来实现时的字节级蓝图，供按需启用；
> 当前不实现（§11 决策）。

> rustbag-ros 的职责一句话：**把“ROS 世界的容器 + 消息”翻译成 rustbag 的四元组
> `(channel, stamp, schema, bytes)`，再交给消费方解。** 内部按“存储层 → 消息层 → 解码层 →
> 时间层”四段做。

### 13.1 存储层：三种容器适配器

rustbag 用 `storage` 插件抽象，三种后端对应 ROS 的三种文件。

#### MCAP（ROS2 现行默认）

小端 TLV 记录流。每条记录：

```
magic: 8字节 = 0x89 'M' 'C' 'A' 'P' '0' 0x0D 0x0A
record: [op:u8][len_body:u64 LE][body:len_body 字节]
opcode: header=01 footer=02 schema=03 channel=04 message=05 chunk=06
        message_index=07 chunk_index=08 ... data_end=0x0F
```

关键 record body（全小端；`str` = `len:u32` + UTF-8）：

- **Schema (03)**：`id:u16` `name:str` `encoding:str` `len_data:u32` `data`
- **Channel (04)**：`id:u16` `schema_id:u16` `topic:str` `message_encoding:str` `metadata:map<str,str>`
- **Message (05)**：`channel_id:u16` `sequence:u32` `log_time:u64` `publish_time:u64` `data(剩余全部)`
- **Chunk (06)**：`msg_start:u64` `msg_end:u64` `uncompressed_size:u64` `uncompressed_crc:u32`
  `compression:str` `len_records:u64` `records(=压缩字典)`

对 ROS2：`schema.encoding = "ros2msg"`（或 `"ros2idl"`），`schema.data` = 完整 `.msg` 文本；
`channel.message_encoding = "cdr"`；`message.data` = CDR 字节。因此 **rustbag-ros 复用核心的
`storage-mcap` 就能直接读 ROS2 MCAP**。

#### sqlite3（旧版 ROS2 默认）

`.bag` 目录 = `metadata.yaml` + `rosbag2_*.db3`：

```
topics(id INTEGER PK, name TEXT, type TEXT, serialization_format TEXT, offered_qos_profiles TEXT)
messages(id INTEGER PK, topic_id INTEGER, timestamp INTEGER, data BLOB)
```

`messages.data` 是 **CDR 字节**，`messages.timestamp` 是 log 时间。sqlite3 版**没有**自描述 schema
（只有 `topics.type`，如 `sensor_msgs/msg/Imu`），所以要解字段必须**自己知道类型布局**——这也是
“手写常用消息解码器”是刚需的原因。

#### ROS1 `.bag`

以 `#ROSBAG V2.0\n` 开头，随后是记录序列。每条记录：

```
<header_len:u32 LE><header:header_len 字节><data_len:u32 LE><data:data_len 字节>
header = 若干 <field_len:u32><field_name>=<field_value>   // '=' 分隔；field_len = name+'='+value 长度
```

`header` 必含 `op`（u8）：

| op | 记录 | header 关键字段 | data |
|---|---|---|---|
| `03` | Bag header | `index_pos:u64`,`conn_count:u32`,`chunk_count:u32` | 填充到 4096 字节 |
| `05` | Chunk | `compression:(none/bz2)`,`size:u32` | 压缩后的 connection+message 记录 |
| `07` | Connection | `conn:u32`,`topic:str` | 另一段“header 格式”字符串：`topic,target,md5sum,message_definition,...` |
| `02` | Message data | `conn:u32`,`time:u64` | 序列化的 ROS1 消息字节 |
| `04` | Index data | `ver:u32`,`conn:u32`,`count:u32` | count×(`time:u64`,`offset:u32`) |
| `06` | Chunk info | `ver:u32`,`chunk_pos:u64`,`start_time:u64`,`end_time:u64`,`count:u32` | count×(`conn:u32`,`count:u32`) |

要点：**字节序全小端**；`op` 在 header 里（不像 MCAP 在记录最前）；`time` 是 8 字节记录时间；
连接信息（`type`/`md5sum`/`message_definition`）在 Connection 记录里，是解消息的关键元数据。

### 13.2 消息层：CDR 与 ROS1 两种序列化

#### ROS2 / CDR

- 所有 CDR 数据前 4 字节是**封装头**：`Bytes0-1 = Representation Identifier`，
  `Bytes2-3 = Options(常为 0)`。`0x0000`=PLAIN_CDR_BE，`0x0001`=PLAIN_CDR_LE（ROS2 默认 LE，
  写出来是 `[0x00,0x01,0x00,0x00]`）。
- 之后**按 4 字节对齐**（`f64`/`i64`/`u64` 按 8 对齐；struct 按其最宽成员对齐）。对齐基点是
  **封装头之后**（`origin=4`）。
- 编码规则：定长类型按 size；`string`=`u32 长度(含\0)` + 字节 + 补零到 4 对齐；
  变长数组=`u32 元素数` + 元素；**定长数组（如 `float64[9]`、`float64[3]`）无长度前缀**，直接连续存；
  嵌套 struct 内联递归。

#### ROS1 序列化

- **无对齐、无补零**，紧凑小端；`string`=`u32 长度(含\0)` + 字节；变长数组=`u32 元素数` + 元素；
  定长数组无前缀。
- 与 CDR 关键区别：ROS1 **无封装头、无对齐**。

#### Schema 元数据装进 rustbag 的 `Schema`

```rust
pub struct Schema {
    pub type_name: String,          // ROS1: "sensor_msgs/Imu"; ROS2: "sensor_msgs/msg/Imu"
    pub encoding:   Encoding,       // Cdr | CdrX2 | Ros1 | ...
    pub message_definition: String, // .msg 文本（ROS2 type description / ROS1 message_definition）
    pub md5: Option<String>,        // 仅 ROS1
}
```

### 13.3 解码层：手写“建图常用消息”

每个解码器输入 `(payload 字节, schema)`，输出**中性结构**（供消费方）或原样字节（其它话题）。
不做“全量动态 ROS 反序列化器”。

#### 统一 Header / 时间提取

- ROS1：`seq(u32)` `stamp.sec(u32)` `stamp.nsec(u32)` `frame_id(string)`
- ROS2：`stamp.sec(int32)` `stamp.nanosec(uint32)` `frame_id(string)`（无 `seq`）

先跳过封装头（若 CDR）或直接读（若 ROS1），提取 `stamp` → 作为调度/语义时间；再跳过 `frame_id`。

#### `sensor_msgs/Imu`

```
header | q(x,y,z,w f64) | orient_cov[9 f64] | ang_vel(xyz f64) | ang_vel_cov[9] | lin_acc(xyz f64) | lin_acc_cov[9]
```

全定长，直接按偏移读 → `ImuRaw { stamp, acc: lin_acc, gyr: ang_vel }`。

#### `sensor_msgs/PointCloud2`（最复杂也最规则）

```
header | height:u32 | width:u32 | fields: PointField[] | is_bigendian:bool | point_step:u32 | row_step:u32 | data:uint8[](变长) | is_dense:bool
PointField = { name:string, offset:u32, datatype:u8, count:u32 }
datatype: 1=i8 2=u8 3=i16 4=u16 5=i32 6=u32 7=f32 8=f64
```

解码逻辑：

1. 按 `fields` 的 `name` 找到 `x/y/z/intensity/time/ring` 的 `offset` 与 `datatype`；
2. `data` 按 `point_step` 切成 `width*height` 个点，从对应 `offset` 读字段；
3. `time` 的单位随数据来源（秒/毫秒），由消费方/配置定 `timestamp_unit`，rustbag-ros 只带出字段；
4. 输出 `StdPointMsg { x,y,z,intensity,time,ring }`。

> `PointCloud2` **没有“动态点结构”**，布局全靠 `fields+point_step` 描述，天然可跨 ROS1/ROS2
> （差异只在 CDR vs ROS1 的“数组前缀 + 对齐”）。

#### Livox `CustomMsg`（Livox Avia / HAP / Mid-360）

ROS1 与 ROS2 **包名/命名空间不同**：`livox_ros_driver/CustomMsg` vs
`livox_ros_driver2|3/msg/CustomMsg`，字段结构接近：

```
header | timebase:u64 | point_num:u16 | lidar_id:u8 | rsvd | points: CustomPoint[定长]
CustomPoint = { offset_time:u32, x,y,z:f32, reflectivity:u8, tag:u8, line:u8 }
```

- `points` 是**定长数组**（无长度前缀），有效点数在 `point_num`；`reflectivity==0xFF` 丢弃。
- 直接映射到 `AviaMsg`/`AviaPointMsg`，`offset_time` 为**微秒**（与 `types.rs` 一致，无需转码）。

#### 其它话题

`Odometry`、`PoseStamped`、`tf`、`tf_static` 等：**原样“字节+schema”透传**，需要时由消费方解析。

### 13.4 时间层：`log_time` vs `header.stamp`（最易错）

- 一个消息有**两个**时间：记录时间（MCAP `log_time`/`publish_time`、sqlite3 `timestamp`、
  rosbag 记录 `time`）和消息内 **header `stamp`**。
- **`rosbag play` 的调度基准是 header `stamp`**（无 header 才退回记录时间）。
- 因此 rustbag-ros 给每个频道设 `time_policy`：

```rust
enum TimePolicy {
    HeaderStamp,                        // 从 payload 的 header 里取（默认，匹配 rosbag）
    LogTime,                            // 用 log/publish/record 时间
    Custom(fn(&[u8], &Schema) -> Timestamp),
}
```

- 只有 `HeaderStamp` 需要“从 payload 头部解 header”，这只在 rustbag-ros 做；rustbag 核心依然只拿到
  一个调度 `Timestamp`，四元组抽象不变。
- 默认 `HeaderStamp`；对 `PointCloud2`/`Imu`/`CustomMsg`，header `stamp` 就是传感器时刻。

### 13.5 数据流：从“文件字节”到消费方的 `DataSource`

```
读取端：
  storage 插件(mcap/sqlite3/rosbag1)        → (channel, log_time, publish_time, raw_bytes, schema)
  → 按 TimePolicy 取调度 timestamp
  → 组装成 rustbag::Message {channel, stamp, schema, payload}
  → 时间调度引擎(rate/loop/seek)
  → 消费方 glue 调 decoder(Imu/PointCloud2/CustomMsg) → SensorData
  → 包装成 fast_lio::DataSource
```

写入端反向：消费方把 `SensorData` 序列化成字节 + 取 stamp + 给 schema，写进 `storage-mcap`。

### 13.6 兼容性边界 / 已知风险

1. **CDR 变体**：ROS2 底层有 `PLAIN_CDR_BE/LE`、`PL_CDR`、`XCDR2`（FastDDS/CycloneDDS 默认不同）。
   先支持 `PLAIN_CDR_LE/BE`（4 字节封装头），`XCDR2`（带 DHEADER）作后续扩展。
2. **对齐陷阱**：CDR 对齐基点是封装头之后（`origin=4`）；严格按 OMG CDR 规则实现，用真实 `.bag` 回放验证。
3. **`.msg` 定义**：ROS1 用 `md5sum` 验证、`message_definition` 参考；ROS2 用 `ros2msg`/`ros2idl`。
   真正解码以“手写已知类型布局”为准，定义文本仅用于自省/版本核对。
4. **时间单位**：rosbag 记录 `time`（微秒）只是记录时间；传感器时间是 header `stamp`（`sec`+`nsec`）。
5. **压缩**：MCAP chunk 常见 `zstd`/`lz4`；ROS1 chunk 常见 `none`/`bz2`；解码在 chunk 层先解压再读内部。
   sqlite3 无 chunk 压缩（DB 自带）。
6. **`PointCloud2` 的 `time` 单位**随发布者不同（秒/毫秒），按数据来源设 `timestamp_unit`，不要硬编码。

### 13.7 模块 / API 草案

```rust
// rustbag-ros
pub mod storage { pub mod mcap; pub mod sqlite3; pub mod rosbag1; }
pub mod codec   { pub mod cdr; pub mod ros1; }
pub mod decode  { pub mod header; pub mod imu; pub mod pointcloud2; pub mod livox; }
pub mod time    { pub enum TimePolicy { HeaderStamp, LogTime, Custom(..) } }

// 中性输出（消费方 glue 用）
pub struct ImuMsg     { pub header: Header, pub acc: [f64;3], pub gyr: [f64;3] }
pub struct Pc2Frame   { pub header: Header, pub points: Vec<PointT>, pub fields: Vec<PointField> }
pub struct LivoxFrame { pub header: Header, pub timebase: u64, pub points: Vec<CustomPoint> }

pub trait MsgDecoder<T> {
    fn decode(payload: &[u8], schema: &Schema) -> Result<T, DecodeError>;
}
```

依赖关系（单向、无环）：

```
rustbag(core)          ← 无领域类型
rustbag-ros            ← 只依赖核心 + 容器/编解码，不依赖任何建图库
rustbag-fastlio(glue)  ← 依赖 rustbag-ros + fast-lio，做 SensorData⇄解码结果 + DataSource 包装
```

### 13.8 小结

可行实现 = **三种存储适配器（MCAP/sqlite3/ROS1 `.bag`）+ 两种消息编解码（CDR/ROS1）+
手写四类建图消息解码（Header/Imu/PointCloud2/Livox CustomMsg）+ 默认 `HeaderStamp` 的时间策略**。
它不碰 FAST-LIO，只把“ROS 的字节 + 元数据”翻译成 rustbag 的四元组；语义映射留给消费方 glue。
最大工作量在 **CDR/ROS1 字段级解码与对齐**，最大坑在 **`log_time` vs `header.stamp` 的时间语义**。

---

## 14. rustbag-msgs：规范消息清单设计

> 目标：给外部「RViz 类可视化工具」提供**可保证被解码**的数据。viz 工具不需要认识任何来源库，
> 只要认识 rustbag 的**规范消息**。这相当于 ROS 定义 `sensor_msgs/PointCloud2`。
>
> rustbag-msgs = **一套规范消息（词汇表）+ 一份 schema 源（`.rsm`）+ 一个文档化 wire 编码**。
> rustbag 核心仍只存 `(channel, stamp, schema, bytes)`，规范层在它之上，不破坏类型无关。

### 14.1 四个设计问题与选择

| 问题 | 选择 |
|---|---|
| 清单有哪些类型 | 一个**闭集**（curated set），覆盖“RViz 类渲染”的最小需求 |
| 怎么表达 schema | **共享 Rust 规范类型 + 文档化字节布局**（不造 `.rsm`；跨语言时复用现成格式） |
| 字节怎么编码 | 一个**跨语言、稳定、文档化**的二进制编码 |
| 怎么演进 | **带版本号 + 向后兼容规则**（加字段在末尾、未知字段跳过） |

### 14.2 核心权衡：封闭集合 vs 动态 schema

- **封闭集合（推荐）**：定义固定类型；viz 工具编译期就知道这些类型，按 type 名分派解码器。
  好处：硬保证、快、无需动态反射、实现简单。代价：新增消息类型需发新版本。
- **动态 schema（开放扩展）**：允许自定义类型 + 自签名 schema。好处：无上限。代价：viz 需要动态解码器。

**推荐：封闭规范为主 + 保留开放扩展。** 规范消息是“保证能解”的子集；自定义消息可共存
（viz 对不认识的类型按“不透明字节”跳过）。既保证 viz 可用，又不锁死未来。

### 14.3 规范消息清单（curated set）

面向“3D LiDAR 回放 / 场景重建 / 导航”，最小集合：

| 规范名 | 用途 | 关键字段 |
|---|---|---|
| `rustbag/PointCloud` | 3D 点云（核心） | header、fields[]、point_step、data 缓冲 |
| `rustbag/Imu` | IMU 惯导（录制 / 离线里程计 / 回放） | header、lin_acc、ang_vel、各 covariance |
| `rustbag/TransformStamped` | 坐标系变换（TF） | stamp、frame_a、frame_b、pos、quat |
| `rustbag/PoseStamped` | 时间戳位姿（轨迹） | stamp、frame_id、pos、quat |
| `rustbag/Path` | 位姿序列（轨迹） | header、poses[] |
| `rustbag/Image` | 相机（可选） | header、编码、分辨率、data |
| `rustbag/OccupancyGrid` | 2D 栅格（导航） | header、resolution、data |
| `rustbag/Marker` | 任意标注（可选） | 类型、位姿、颜色、几何 |
| `rustbag/Clock` | 模拟时间（可选） | sec/nsec |

对“3D 激光雷达实时回放”，**至少需要 `PointCloud` + `TransformStamped`/`PoseStamped`**。
点云必须带 `frame_id` 和逐点时间，Transform 用于把点云从 sensor 帧放进世界系。

### 14.4 契约形式：不做 `.rsm`，用“共享 Rust 类型 + 文档化字节布局”

`.rsm` 只是“单一事实来源 + 跨语言 codegen”这个目标的一种手段，不是必要条件。真正必要的是：
**一个 rustbag 与 viz 工具都认同的、文档化、版本化的“字节契约”**，外加四元组里 `Schema` 携带的
type 名 / encoding。

要点：

- **`Schema`（元数据）必要，但只需 `type_name` + `encoding`**，让 viz 选对解码器；不需要一份 `.rsm` 文件。
- **点云本身自描述**：`PointCloud` 的字段布局在 payload 内的 `fields[]`，外部工具读消息体即可，
  无需外部 schema。
- **若 viz 也是 Rust**：rustbag 与 viz 直接共用 `rustbag-msgs` crate，**crate 即契约**；
  `.rsm`/codegen 是纯开销。
- **契约源头** = Rust 结构体 + codec 实现 + 一份人写的字节布局规范（spec doc）；可加一个
  “把类型渲染成规范文本”的轻量宏，防止 doc 与代码 drift（这是轻量 codegen，不是 `.rsm` 语言）。

#### 什么情况才需要“schema 文件”

1. 需要**非 Rust 客户端自动 codegen** 解码器；
2. 需要**运行时自描述 / 泛型解码**；
3. 需要多语言绑定且不想手维护。

即便到此，也**不要自造 `.rsm`**，直接复用现成格式作为“编码插件”而非主契约：

| 需要 | 用现成格式 |
|---|---|
| 兼顾 ROS / 建图互操作 | ROS `.msg` / `.idl` |
| 零拷贝、大点云高效 + 跨语言 codegen | FlatBuffers `.fbs` |
| 通用、生态大、易绑定多语言 | protobuf `.proto` / JSON Schema |

### 14.5 wire 编码：物理契约（最关键决定）

外部工具要能解，编码必须**文档化、跨语言、稳定**，且对**大数组（点云缓冲）高效**。三条候选：

| 方案 | 优点 | 缺点 | 适合 |
|---|---|---|---|
| **自定义 compact**（小端、定长固定宽度、变长有长度前缀、无对齐、版本头） | 零依赖、快、贴合“元数据 + 大 buffer”结构、易跨语言实现 | 自己维护编码规范 | ✅ **推荐（独立 rustbag 场景）** |
| **CDR**（ROS2/DDS 用） | 与 ROS/MCAP 互通 | 有对齐规则、封装头、较复杂 | 若想与 rosbag2/MCAP 互通 |
| **FlatBuffers / Cap'n Proto** | 零拷贝、点云 buffer 高效、跨语言 codegen | 引入依赖 + 生成代码 | 若追求极致性能 |

**推荐：自定义 compact 编码，`.rsm` 为唯一事实来源，编码规则写成明确规范。** 点云大头（`data` 缓冲）
就是一个 `u32 长度 + 原始字节`，最可移植；元数据用定宽标量 + 长度前缀。若将来要喂 ROS/Foxglove，
可额外提供 **CDR 编码器**作为“编码插件”，同一份 `.rsm` 生成两种编码（CDR / 自定义）。

**示例：`rustbag/PointCloud` wire 布局（小端、无对齐）**

```
u32  payload_len；payload := 后续字段
  Header: i32 sec; u32 nsec; str frame_id
  u32 height; u32 width
  u32 fields_count; PointField[]   # 每项 {str name; u32 offset; u8 datatype; u32 count}
  u8 is_bigendian
  u32 point_step; u32 row_step
  bytes data                        # u32 len + 原始点缓冲
  u8 is_dense
```

外部 viz 工具按此规范即可解出点云，不依赖任何库。

### 14.6 版本与演进规则

- 消息带 `schema_version`/`encoding`（core 的 `Schema` 已能放）。
- **只向后兼容地加字段**：新字段**追加在末尾**；解码器**跳过未知字段**；不删已有字段（或标记 deprecated）。
- **不兼容变更 → 新 schema_id / 新类型名**，不同名共存。
- `TypeName` 用完整、稳定的命名空间（`rustbag/PointCloud`），避免碰撞。
- 点云的 `fields[]` 天然支持“字段增减/顺序变化”，点云本身不怕传感器字段变化，这是此类消息健壮的关键。

### 14.7 与“type-agnostic 核心”共存（分层）

```
rustbag(核心)            # 只存字节 + Schema，不认识任何消息内容
rustbag-msgs:           # 规范消息 + .rsm + 编码器/解码器（中立契约）
   ├─ 生成 Rust 类型、Schema、wire encoder/decoder
   └─ well-known 类型注册表：type 名 → 编解码器
glue/消费方              # 把来源数据(SensorData)转成规范消息，或读规范消息
外部 viz 工具             # 只认识 rustbag 规范名 + 编码，用注册表分派解码
```

- core **不“认识”点云/位姿**，但 `rustbag-msgs` 提供“如何把某规范类型编码成字节 + 其 `Schema`”。
- **record 时用规范类型**：glue 把 `SensorData`（或点云）转成 `rustbag/PointCloud` 写入
  → 这张 bag 就是“可直接被任意 viz 解”的数据。
- **viz 工具按 well-known 注册表分派**：见 `rustbag/PointCloud` → 用其解码器；不认识的私有类型 → 跳过。

### 14.8 落到“3D 激光雷达实时回放”

要真实回放一帧场景，bag 需要这几类**规范数据**：

1. `rustbag/PointCloud`（每帧 + `frame_id` + 逐点 `time` 字段 → 光束扫动动画）；
2. `rustbag/TransformStamped`（`lidar↔base`、`lidar↔IMU` 外参 + `base↔map` 每帧位姿）→ 把点云放进世界系；
3. `rustbag/PoseStamped`/`Path`（轨迹，便于展示传感器运动）。

viz 工具工作：读 `TransformStamped` 建 frame 图 → 把 `PointCloud.frame_id` 变换到显示系 → 累加/渲染。

### 14.9 落地顺序

1. 定 `PointCloud` 的 `.rsm` + wire 编码（最关键，覆盖 3D LiDAR）。
2. `.rsm` → Rust 类型 + encode/decode 的 codegen（或先手写这一个类型，验证编解码往返）。
3. 定 `TransformStamped` / `PoseStamped` / `Header` / `Time` 的 `.rsm`。
4. 规范类型注册表（type 名 → 编解码器），让 `info`/读取端按名分派。
5. 接 `rustbag-fastlio` glue：把 `SensorData` 转成规范 `PointCloud` + `Transform`/`Pose` 写入。
6. 后续补 `Image` / `OccupancyGrid` / `Path` / `Marker`。

### 14.10 小结

`rustbag-msgs` 的设计 = **一组共享 Rust 规范类型 + 一套“小端、长度前缀、大小全用”的文档化
compact 编码（物理）+ 一个封闭规范集合（保证可解）+ 版本演进规则 + well-known 类型注册表（分派）**。
不需要专门的 `.rsm` 语言；若要跨语言 codegen，复用现有 schema 格式（FlatBuffers/proto/ROS .msg）
而非自造。这样任何独立 viz 工具按公开字节布局实现解码，就能渲染 rustbag 录制的数据；rustbag 核心
仍然完全类型无关。

---

## 15. 是否兼容 ROS/ROS2 bag 的决策论证

> 由三个子 agent 分持「赞成 / 反对 / 中立范围决策」三视角辩论后综合。结论高度收敛：
> **核心不依赖 ROS，兼容是可选、分阶段、feature-gated 的附加层。**

### 15.1 结论

- **核心主链路不需要 ROS 兼容**：`单一 .rbag` + `rustbag-msgs` 规范消息 + 喂独立 viz，
  完全不依赖 ROS。
- **兼容仅作为可选层**，放在 `rustbag-ros`，用 feature 隔离（`--features ros2` / `ros1`），
  核心 crate 保持零依赖、零 ROS 类型。
- **分阶段**：P0 核心 → P1 读 ROS2 mcap（高价值低成本）→ P2 读 ROS1 `.bag`（按需）→ P3 写回（低）。

### 15.2 决策矩阵

| 范围 | 价值 | 成本 | 与单文件 `.rbag`/中立契约一致性 | 判定 |
|---|---|---|---|---|
| 核心 `.rbag` + `rustbag-msgs` + 喂 viz | 必做 | — | 完全一致 | **P0 最先** |
| 读 ROS2 `mcap` | 高 | 低 | 高（同是单文件自描述） | **P1 高优先** |
| 读 ROS2 `sqlite3` | 中 | 低 | 中 | P1 附带（复用 CDR codec） |
| 读 ROS1 `.bag` | 中 | 中高 | 低 | **P2 按需** |
| 写回 MCAP（ROS2/Foxglove 可读） | 低 | 低 | 中 | P3 低优先 |
| 写回 ROS1 `.bag` | 很低 | 高 | 低 | **明确不做** |
| 完整 ROS 类型系统 / 动态反序列化 | 中 | 很高 | 低 | **明确不做**（违背类型无关） |
| `/clock` + `use_sim_time` | 低 | 中 | 低 | **不做**（rustbag-msgs 自有时钟） |

### 15.3 三视角主要论点

- **赞成方**：存量 ROS1/ROS2 数据集要吃；MCAP 是 ROS2 默认且与 `.rbag` 单文件天然契合；CDR 读成本被
  低估；不必做全 ROS 类型系统，只需手写 `Imu`/`PointCloud2`/`CustomMsg`。
- **反对方**：核心目标不依赖它；CDR/ROS1 两套序列化 + 动态类型 + 时间语义是长期维护包袱；把兼容当核心
  会重新拉回“绑定具体库”的初衷；应作为可选/后置插件。同时自纠：“完全不做”不现实，因为大概率要用存量数据。
- **中立方**：用成本/价值/一致性矩阵做分阶段决策；给出触发信号与隔离方式。

### 15.4 建议的落地顺序（把兼容“延后但不排除”）

```
P0  类型无关 .rbag（MCAP 单文件）+ rustbag-msgs + 回放喂 viz        ← 先把主链路做稳
P1  storage-mcap + codec-cdr + 手写 Imu/PointCloud2/CustomMsg     ← 高价值低成本，紧跟 P0
P2  storage-rosbag1 + ROS1 序列化 + HeaderStamp 时间               ← 按需（经典数据集）
P3  写回 MCAP+CDR（Foxglove/rosbag2 读）                           ← 低优先级
明确不做  完整 ROS 类型系统 / 动态反序列化 / /clock+use_sim_time / 写回 ROS1 .bag
```

### 15.5 触发信号（决定把哪一项从“可选”升为“必须”）

1. 必须直接回放**现有 ROS1/ROS2 bag 数据**（如 FAST-LIO 公开数据集）→ 提前 P1/P2；
2. 独立 viz 工具需要**直接读 ROS2 MCAP** 互通 → 提前 P1；
3. 计划把 rustbag 作为**通用 bag 工具发布**，供别人直接读 ROS 数据 → 提前 P1/P2。

### 15.6 分水岭已拍板

**已有结论：无必须直接回放的现成 ROS bag 存量数据。**

- → 兼容**整体后置**：P1（读 ROS2 mcap）、P2（ROS1）、P3（写回）全部**暂不启动**；
- 当前只做 **P0 核心**（单一 `.rbag` + `rustbag-msgs` + 喂独立 viz）；
- 第 12/13 节的 `rustbag-ros` 设计**保留为“将来可选”**，仅当 §15.5 触发信号出现时再启动，
  并作为 feature-gated 可选层接入，不进核心。

---

## 16. 设计完整性评估：已覆盖 vs 待补齐

### 16.1 总体判断

**架构方向、分层、边界与关键决策（解耦 / 单文件 `.rbag` / 规范消息 / ROS 后置）已比较全面、自洽。**
但离“可直接实现 P0”还差几处**实现级**空白：有的只画了方向没落到字节/接口，有的清单本身不完整。

### 16.2 已覆盖（可视为定稿）

- 定位：rustbag = 类型无关容器 + 时间调度引擎；四元组 `(channel, stamp, schema, bytes)`。
- 分层：core / rustbag-msgs / rustbag-ros(后置) / glue，依赖单向无环。
- 存储方向：单一 `.rbag`（MCAP 底层），自描述 + 索引 + 压缩。
- 时间轴：rate / loop / seek / 非阻塞；整数时间戳。
- 契约：channel / stamp / bytes / schema 四要素。
- ROS 决策：当前不做，后置可选；`rustbag-ros` 保留为蓝图。
- 规范消息方向：共享 Rust 类型 + 文档化布局，不造 `.rsm`。

### 16.3 P0 实现前必须补的缺口（高优先）

1. **单文件 `.rbag` 字节级布局 + 崩溃恢复**：MCAP 的 footer/summary 只在正常 close 时写，
   中断/断电会导致无索引、seek/info 失效 → 需定义容器版本头、头部信息、重索引/恢复策略。
2. **时间 / 顺序语义落实**：
   - 时间戳表示（`u64 ns` vs `sec+nsec`）与换算；
   - play 的**顺序语义**：默认“单一合并时间序流”且**保持录制顺序、不重排**（匹配 FAST-LIO“先 IMU 后帧”）；
   - 回放精度 / 漂移、loop 重基准、`rate=0` 疯灌。
3. **`rustbag-msgs` 补全**：
   - 清单**已补 `Imu`**（FAST-LIO 录制/离线里程计必需，见 §14.3）；
   - 但每个规范消息的**完整字段 + wire 布局**尚未定义（仅 `PointCloud` 有方向）；`Header`/`Time` 需精确；
   - **compact 编码规则**（端序、长度前缀、嵌套、对齐）需写成明确规范。
4. **消费方 / viz 接口**（“回放喂给独立 viz”的最后一公里）：rustbag 通过什么接口把数据交给 viz？
   Rust 库 API（`for msg in player`）还是 CLI 启动 viz（`rustbag play --show xx.rbag`）？
   → 需明确集成模型。
5. **录制 API**：如何注册 channel/schema、写消息；多源混合（点云 + 位姿同轴）；live vs 离线；
   是否允许存“不透明字节”通道（允许，但 viz 不可解，需注明推荐规范消息）。

### 16.4 建议补齐（中优先）

6. **压缩策略**（zstd/lz4，作用于 chunk）；大点云的性能/内存（mmap / 流式读、零拷贝）。
7. **版本演进打通**：容器级格式版本 + schema 级版本、向后兼容。
8. **seek / 时间窗口 / info** 的具体查询接口。
9. **多传感器外参**：`TransformStamped`(tf_static) 通道或 `SceneMeta` 约定 + frame 图语义。
10. **性能 / 健壮性**：目标吞吐、golden 测试、跨语言校验点云 layout 的规范文档。

### 16.5 已拍板的设计决策

以下 4 点已按推荐方式落定（详见 §17）：

- **时间戳表示**：内部调度时间用 `u64 纳秒`；每个规范消息仍自带 `Header.stamp`（sec + nsec）。
- **play 输出模型**：默认**单一合并时间序流**，**保持录制顺序、不重排**；`sort_by_ts`（按 `bag.ts`
  严格重排）保留为可选，默认关闭。
- **viz 集成模型**：**rustbag 管时序 + `rustbag play --show viz_app` 启动独立 viz**；
  同时提供库 API 供 viz 直接读 `.rbag`。
- **wire 编码**：自研紧凑 `rbag1`（全小端、无对齐、长度前缀），跨语言按文档实现；CDR 作未来可选插件。

### 16.6 小结：补齐即可实现

当前方案**方向正确、架构清晰**，但实现 P0 前需补齐五点：
**① `.rbag` 字节级布局与崩溃恢复；② 时间/顺序语义；③ rustbag-msgs 消息与 wire 布局补全（含 `Imu`）；
④ 与 viz 的集成接口；⑤ 录制 API。** 补齐这五点即可进入实现。

---

## 17. P0 详细设计（必补 + 建议补）

### 17.1 必补（P0 阻塞项）

#### ① `.rbag` 单文件布局 + 崩溃恢复

`.rbag` 本身就是一份 **MCAP 文件**，`header.profile="rustbag"`、`library="rustbag/<ver>"`。

| MCAP 记录 | 承载内容 |
|---|---|
| `Header` | profile / library / 文件格式版本 |
| `Schema` | 每个规范消息的类型名 + 编码(`"rbag1"`) + 定义文本 |
| `Channel` | 主题名、schema_id、message_encoding |
| `Message` | channel_id、sequence、log_time、publish_time、data |
| `Chunk` | 压缩的消息集合（zstd/lz4）|
| `Message Index` / `Chunk Index` | 时间→偏移，供 seek |
| `Metadata` | bag 级信息：`scene_meta`、通道角色、时长、消息数 |
| `Statistics` / `Footer` | 汇总 + summary 偏移 |

**崩溃恢复（关键）**——MCAP 的 footer/summary 只在正常 close 时写，中断/断电会缺 summary、可能有截断尾巴：

1. **正常关闭**：写 `statistics` + `chunk_index` + `footer`；打开走 summary 快读。
2. **无 summary**：回退为**线性扫描**数据区，读到“下一条不完整记录”为止；遇 chunk 解压并读其内的
   `message_index`，**在线重建索引**。`info`/`play` 仍可用（稍慢）。
3. **`reindex` 工具**：显式扫描并回写新 summary/footer，恢复“快读”。
4. **容错**：chunk CRC（MCAP 自带）允许“校验失败→警告并跳过该 chunk”；末尾不完整记录忽略。

实现要点：`record` 边写边 flush（每 N 条或 N 秒写一个 chunk + message_index），保证中断后 chunk 内数据
完整可扫；`finish()` 写 summary+footer；`open` 先读 footer，缺失则触发恢复扫描。

#### ② 时间与顺序语义

**时间戳表示**：内部调度时间 `Timestamp = u64 纳秒`（单调、整数、无浮点漂移）。

```rust
impl Timestamp {
    fn from_secs_f64(s: f64) -> Self;            // 乘 1e9，四舍五入到 ns
    fn to_secs_f64(self) -> f64;
    fn from_sec_nsec(sec: i64, nsec: u32) -> Self;
}
```

规范消息仍自带 `Header.stamp`（秒 + 纳秒、语义时间）；`write_with_stamp` 自动从 Header 取值换算成
`bag.ts`（调度时间）。二者默认相等，但有“消息内语义时间”与“bag 调度时间”两层概念。

**顺序语义（最关键）**：
- `record` **按调用顺序追加**，每条带 `bag.ts`，不重排；
- `play` 默认**单一合并时间序流**：跨所有 channel 按 `bag.ts` 升序合并，**平局按录制顺序**（稳定）；
- 可选 `sort_by_ts`（严格按 `bag.ts` 重排，默认关闭），文档标注可能破坏消费端“先 IMU 后帧”假设。

**速率 / 循环 / 非阻塞**：
- `rate`：`next_due = sim_start + (msg.ts - first.ts) / rate`，按墙钟 sleep 到 `next_due`；
  `rate=0` = 疯灌，`rate=1` = 实时，`>1` 更快；整数时间差避免浮点漂移；
- `loop`：循环时重算时间基准，保证首尾时间戳单调；
- `seek(ts)`：把 `sim_start` 挪到目标；
- `try_next()`：未到 `next_due` 则返回 `NonBlocking`；`next()` 阻塞；
- `play.now()`：暴露当前模拟时间，供 viz 同步/动画。

#### ③ `rustbag-msgs` 补全 + wire 布局

**通用 `Header` / `Time`**：
```
Time   { sec: i64 (小端), nsec: u32 }        // nsec ∈ [0,1e9)
Header { stamp: Time, frame_id: string }     // string = u32 len + UTF-8
```

**compact 编码（`rbag1`）规则**：全小端；无对齐；定宽类型固定字节；`string`=`u32 len`+utf8；
`bytes`=`u32 len`+原字节；序列 `T[]`=`u32 count`+元素；定长数组 `[T;N]`=N×T 内联（无长度前缀）；
嵌套 msg 内联（序列中递归）。

**PointCloud wire**：
```
Header
height:u32  width:u32
fields: PointField[]        // { name:string, offset:u32, datatype:u8, count:u32 }
is_bigendian:u8  point_step:u32  row_step:u32
data: bytes                 // u32 len + 原始点缓冲
is_dense:u8
datatype: 1=i8 2=u8 3=i16 4=u16 5=i32 6=u32 7=f32 8=f64
```

**Imu wire**（FAST-LIO 必需）：
```
Header
orientation:   [f64;4]   // xyzw，可为 0 表示未知
orientation_cov: [f64;9]
ang_vel:       [f64;3]
ang_vel_cov:   [f64;9]
lin_acc:       [f64;3]
lin_acc_cov:   [f64;9]
```

**TransformStamped / PoseStamped / Path wire**：
```
TransformStamped: Header + frame_a:string + frame_b:string + translation:[f64;3] + rotation:[f64;4]
PoseStamped:      Header + pos:[f64;3] + quat:[f64;4]
Path:             Header + poses: PoseStamped[]
```

**Schema 注册表**：`type_name` 稳定名（如 `rustbag/PointCloud`）+ 可选 schema hash；
`rustbag-msgs` 提供 `type_name → encode/decode`；容器/schema 首版都带 `format_version` 写入 `.rbag` 头。

#### ④ 消费方 / viz 接口

两个接口都提供，**以库 API 为主**：
```rust
let mut play = rustbag::Player::open("a.rbag", PlayOptions::default().rate(1.0))?;
for msg in play {                       // Message {channel, stamp, schema, payload}
    if msg.channel == "lidar" {
        let cloud: PointCloud = rustbag_msgs::decode(&msg)?;
        viz.render(&cloud);
    }
}
play.now();                             // 当前模拟时间，供 viz 动画/同步
// 类型化便利：play.messages_on::<PointCloud>("lidar")
```

**CLI 启动模型**（demo 主链路）：
```bash
rustbag record -o a.rbag --lidar ... --imu ...
rustbag play a.rbag --show viz_app      # rustbag 管时序，拉起 viz_app 渲染
rustbag play --json a.rbag              # JSON 流，供非 Rust viz
```

同步：viz 用 `play.now()` + 各消息 `Header.stamp` 做多 channel 对齐；`play` 按 `bag.ts` 调度吐出。

#### ⑤ 录制 API

```rust
let mut rec = rustbag::Recorder::open("a.rbag", RecordOptions::default())?;
rec.register_channel("lidar", Schema::from_type::<PointCloud>())?; // 注册 codec
rec.write_with_stamp("lidar", &cloud)?;                            // 自动从 Header 取 stamp
rec.write("imu", ts, &imu)?;                                       // 显式 ts
rec.write_raw("scan", ts, schema_id, &raw_bytes)?;                  // 不透明通道（viz 不可解）
rec.flush()?;
rec.finish()?;                                                     // 写 summary+footer
```

- **多源混合**：调用方控制顺序；`MergedRecorder` 接收多个 `DataSource` 按 `bag.ts` 合并写入（与 play 对称）。
- **live vs 离线**：`write` 按 chunk 缓冲、周期 flush；可选无阻塞消费。
- **胶水**：`rustbag-fastlio` 用 `Recorder` 包装 `DataSource`（SimSource/LivoxSource），
  把 `SensorData` 编码成 `PointCloud`/`Imu` 并提取 stamp。

### 17.2 建议补齐（中优先）

#### ⑥ 压缩 + 大点云性能

- **压缩**：chunk 级；默认 `zstd`(level 3)，可选 `lz4`(快)/`none`；点云 `data` 缓冲不做场内压缩，
  交给 chunk 层。
- **大点云**：读按 chunk **流式**、有界内存、可选 `mmap`；写复用点缓冲；`try_next`+sleep 背压；
  基准目标：10 Hz × ~100 万点（几十 MB/s）实时回放 + 压缩（criterion 量化）。

#### ⑦ 版本演进

- **容器级**：`.rbag` header 带 `format_version`；读取器拒绝高于其支持的大版本。
- **schema 级**：规范消息**只追加、不删不改序**；未知尾部字段旧解析器跳过；破坏性变更 → 新类型名/版本。
- **schema hash**：类型定义内容哈希，校验写/读端一致，不匹配则警告。
- 格式规范维护在 `spec/`，作为“契约”。

#### ⑧ seek / 时间窗口 / info

- `Player::seek(ts)`、`set_window(start,end)`、`from(start)`。
- `info`：优先读 summary（statistics + chunk_index）；无 summary 则线性扫描。
- 查询：`Reader::channels()`、`channel_info(name)` → 类型、数量、时间范围、字节数。

#### ⑨ 多传感器外参 / SceneMeta / frame 图

- 外参用 `TransformStamped`(tf_static) 通道记录（parent→child frame）。
- `SceneMeta`（放 MCAP `metadata`）：`world_frame`（如 `map`）+ `channel_roles`（lidar/imu/pose 通道）。
- **frame 解析**：viz 读所有 `TransformStamped` 建 `child→parent` 树，把 `PointCloud.frame_id` 沿树组合
  变换到 `world_frame`；缺 frame 则回退为“视为 world 帧”或提示用户。
- 静态（一次、不变）与动态（逐 ts）变换区分；`tf` 缓存不变项。

#### ⑩ 性能 / 健壮性 / golden 测试

- **golden 测试**：每个规范消息生成黄金字节，断言 `encode→decode==原值` 且 `encode==文档字节`。
- **round-trip**：合成 `DataSource` → record → play，断言消息序列与时间戳完全一致。
- **reindex**：删 footer/summary → `reindex` → 内容一致。
- **损坏容错**：截断 / 坏 chunk 测试。
- **跨语言校验**：`spec/` 保留每个消息的样例 hex dump，供非 Rust 客户端对照。
- **性能基准**：encode/decode 吞吐、压缩比、回放抖动。

### 17.3 P0 最小闭环原型落地记录

**目标**：`record(Imu + PointCloud) → 单文件 .rbag → play → decode → round-trip` 跑通。

**实际结构**（离线、零外部依赖）：
- `crates/rustbag-core`：核心值类型 + 存储 trait + **`storage::singlefile`（自带单文件 `.rbag` 容器）**
  + `record::Recorder` + `play::Player`。
- `crates/rustbag-msgs`：`Header`/`Time`/`PointCloud`/`Imu` + `rbag1` 编解码（含 golden / round-trip 测试）。
- `crates/rustbag`：CLI（`selftest` / `info` / `play`；`record` 待接线）。
  `play` 支持 `--rate`/`--loop`/`--json`/`--show <cmd>`，可实时回放、输出单行 JSON、管道给外部 viz。
- `crates/rustbag-fastlio`：FAST-LIO 胶水，`SensorData ⇄ rustbag-msgs`；`record_source` 把
  `fast_lio::data_source::DataSource` 录进 `.rbag`，`BagDataSource` 从 `.rbag` 回放成 `DataSource`。

**与 §9 的分层差异（待审）**：原型把存储后端 + Recorder/Player 直接并入 `rustbag-core`，
而非 §9 的独立 `storage-mcap` / facade / cli 拆法。优点是启动快；代价是 core 不再“纯 I/O 无关”。
将来若要严格分层，可把 `storage::singlefile` 抽到独立 crate（实现同一 `StorageWriter/Reader` trait），
core 只留类型 + trait。**交换后端只需换一个 crate，无需改 core/msgs。**

**容器选择偏差**：设计 §6.0/§17 默认 MCAP；因离线无 `mcap` crate，P0 用**自实现单文件 `.rbag` 容器**
（`#RBAG` 头 + schema/channel 段 + 消息流 + `RBAG_END` 尾索引 + 缺失时线性扫描恢复）。
仍是“单一文件”，且通过 storage trait 可后续置换为 MCAP。

**验证结果**：
- `cargo build` ✅；`cargo test`（rustbag-msgs 6 项）✅；
- `cargo run -p rustbag -- selftest` → `selftest OK`；
- `rustbag info <file>` → `messages: 2`、`topic "lidar": rustbag/PointCloud`、`topic "imu": rustbag/Imu`；
- `rustbag play <file> --json` → 每条消息一行 JSON（点云含解码后 points、IMU 含 lin_acc/ang_vel）；
  `--show <cmd>` 把 JSON 逐条写入外部进程 stdin（rustbag 管时序、viz 管渲染）。
- `rustbag-fastlio glue_demo` → `expected=1052 actual=1052`、`glue OK`；`--lio` 用回放数据驱动
  FAST-LIO 建图：`frames=47 map_points=284 pos=(3.06,1.06,-0.02)`。
- 文件头 `#RBAG`，尾 `RBAG_END` + `data_offset` + `message_count=2`。

**执行方式说明（并行）**：任务按依赖规划为
`core↔msgs（无依赖，可并行）→ storage-file / facade（依赖 core/msgs，可并行）→ 集成`。
但环境并发 agent 槽位受限，子 agent 无法新增 spawn；facade 子 agent 为一并推进而中断了 storage worker，
并亲自落地 `① 单文件后端 ② Recorder/Player ③ CLI`，实际收敛为串行落地。**净产出为一个可运行的 P0 闭环。**

---

## 附：FAST-LIO 时间语义（供 glue 实现参考）

> 以下内容只用于**写 FAST-LIO 的胶水**，不属于 rustbag 内核。

- `crates/fast-lio/src/data_source.rs` — `DataSource` trait、`SimSource`；样本必须时间有序。
- `crates/fast-lio/src/types.rs` — `SensorData` / `ImuRaw` / `AviaMsg` / `StandardMsg` / `TimeUnit`。
- `crates/fast-lio/src/laser_mapping.rs` —
  `add_imu`/`add_lidar_*` 对乱序会 `clear()`；`sync_packages` 需 `last_timestamp_imu >= lidar_end_time`
  才触发，并要求 IMU 落在 `[lidar_beg_time, lidar_end_time]` 区间。
- `AviaPointMsg.offset_time`（µs）、`StdPointMsg.time`（单位由 `timestamp_unit` 决定）：逐点时间必须在
  消息字节中原样保留；回放时必须**先输出完某帧区间内的 IMU，再输出该帧激光**（保证严格按 `stamp` 升序）。
- `README_zh.md` — 核心 crate 零 I/O、无 ROS；未来路线含“实现 rosbag 的 `DataSource`”。
