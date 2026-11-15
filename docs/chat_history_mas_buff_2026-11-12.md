# 聊天记录：实际检测与录制回放模块化
- 会话 ID: `sess_6e7463a5-a925-4b6a-a17b-5a5c65fde529`
- 原工作目录: `D:\vs\vscode\c++\huge mas\mas_buff`
- 创建: 2026-11-12 12:25:34 / 最后更新: 2026-11-12 16:09:43

---

## 🧑 我  (12:25:34)
我的想法是各模块化，各司其职。跑实际检测的时候就跑实际检测，然后录制回放的话，它就在另一个模块单独写一块模板，然后跑这个回放。你看看是这样吗？还是说是交叉的

---

## 🤖 助手  (12:25:34)
> 🔧 `Bash` — List project structure

> 🔧 `Bash` — Find C++ source files

> 🔧 `Bash` — List project source files excluding deps

> 🔧 `Bash` — Read code file guide

> 🔧 `Bash` — Read main entry and app headers

> 🔧 `Bash` — Read module and replay interfaces

> 🔧 `Bash` — Count lines of key files

> 🔧 `Bash` — Read replay implementation

> 🔧 `Bash` — Read live and record implementations

> 🔧 `Bash` — Read buff thread and pipeline

> 🔧 `Bash` — Grep architecture audit for relevant notes

> 🔧 `Bash` — Read configs and build targets

> 🔧 `Bash` — Get line numbers for duplicated pipeline logic

## 结论

方向对了一半：**“各司其职”应体现在驱动层和 IO 层，而不是把算法再写第二份。**
换句话说：

- 实机、录制、回放各自有独立的“入口模板、输入源、时钟和输出” —— ✅ 应该分开
- 检测、跟踪、火控以及“单帧里先做什么后做什么” —— ❌ 只能有一份，回放必须调用实机同一套

如果回放模块自己再写一套检测/跟踪流程，那就不叫模块化，叫**平行实现**，实机和回放会慢慢漂移。你现在仓库里已经接近正确分层，但确实存在这种“交叉”。

---

## 你现在仓库的实际分层

```
apps/main.cpp                         命令分派
 ├─ live   ─┐
 ├─ record ─┤  各自独立的驱动：输入源、时钟、循环、输出
 ├─ replay ─┤
 ├─ read   ─┤
 └─ check  ─┘
        ↓ 只应调用
auto_buff/buff_replay::process_buff_frame    ← 单帧语义（应唯一）
        ↓
buff_detect / buff_track / buff_shoot        ← 算法内核（已唯一）
        ↓
rm_utils / hardware / threads                ← 基础设施
```

已经分对的部分：

| 部分 | 现状 | 评价 |
|---|---|---|
| `apps/main.cpp` 命令分派 | run/read/export/record/replay/check 各自入口 | ✅ 正确 |
| `run_record` | 只采集，不创建 detector/tracker/shooter (`apps/buff/record.cpp:24`) | ✅ 正确，保持 |
| `run_replay(check)` | 只校验配对，不加载模型 (`apps/buff/replay.cpp:80-91`) | ✅ 正确 |
| record 生成 `replay.yaml` 和配置快照 | 只通过**数据格式**衔接，不共享代码 | ✅ 这正是“不相交”的边界 |
| 算法类 | 实机和回放共用 `make_buff_modules` | ✅ 已唯一 |

---

## 现在真正“交叉”的地方

### 1. 实机流水线和离线流水线是两份（最主要的问题）

- 实机：`threads/buff_thread/buff_thread.cpp:33-103` 内联写了一遍 `detect → 筛选 → 取姿态 → track → shoot → sendVision`
- 离线：`auto_buff/buff_replay/buff_replay.cpp:20-72` 又写了一遍 `detect → 筛选 → 姿态 → track → shoot`

两者用的是同一批算法类，但**阶段顺序、筛选、姿态注入、重置策略各写各的**。文档里也明确写了“实机当前不走此函数”（`docs/CODE_STRUCTURE_AUDIT_20261109.md:82`）。这就是典型的“用类复用了，用编排没复用”。

后果：改一个跟踪前处理或姿态顺序，要改两处；漏一处，回放就不再能复现实机。

### 2. 模式/颜色/重置策略重复

- 实机：`threads/buff_thread/buff_thread.cpp:112-136`
- 回放：`apps/buff/replay.cpp:192-226`

同样是“模式变了就 reset + 按红蓝位切颜色”，写了两遍，而且初始模式、偶数模式的解释还不完全一样。

### 3. `replay.cpp` 一个 268 行函数兼了五件事

播放控制、终端按键、算法驱动、视频输出、JSONL/摘要全在一个 `run_replay` 里（`apps/buff/replay.cpp:73-266`）。这不算“交叉”，但属于**驱动层没有继续拆**，后面加回放功能会越来越难改。

### 4. 轻微的职责渗漏

`process_buff_frame` 里的 `realtime` 和 `waiting_for_pose`（`auto_buff/buff_replay/buff_replay.cpp:36-45`）其实是驱动层策略（实机/离线）渗进了单帧核心。不算致命，但说明现在的“共享函数”在替驱动做决定。

---

## 我建议的正确切法

三层，规则只有一句：**计算共享，副作用注入。**

### ① 算法内核层（唯一）
`buff_detect` / `buff_track` / `buff_shoot`
不碰相机、串口、时钟、文件。现在基本就是这样。

### ② 单帧编排层（唯一）
只保留一个“处理一帧”的入口，把差异做成策略参数：

```
process_frame(input, Policy)
Policy:
  - 时间来源：实时 clock / 记录时间
  - 姿态来源：串口按曝光时刻取 / 回放 CSV
  - 允许无姿态：实机 false / 离线固定相机 true
  - 帧龄检查：实机有 / 回放无
  - 模式切换、颜色、reset 回调
  - 结果输出回调（sendVision / 写文件）
```

关键点：实机要求“**先 detect，再按曝光时刻取姿态**”（`buff_thread.cpp:61-68`），所以策略里要允许在 detect 之后回调取姿态，而不是像现在 `process_buff_frame` 那样在 detect 前就准备好。

### ③ 驱动层（各自独立，“模板”指这里）
| 驱动 | 输入 | 时间 | 输出 | 是否跑算法 |
|---|---|---|---|---|
| live | 海康线程 | steady_clock | sendVision + JSONL/CSV | 是 |
| record | 海康线程 | 曝光时刻 | paired 录制 | **否** |
| replay | 成对播放器 | CSV 记录时间 | 标注视频 + JSONL | 是 |
| read/export | 媒体文件 | 媒体时间 | 导出 | 是 |
| check | 成对播放器 | 无 | 校验结果 | **否** |

驱动之间可以共享“生命周期骨架”（初始化 → 主循环 → 排空 → summary），但**不能共享算法流程的实现**。

### ④ 数据层（唯一）
`paired_frame`、`paired_recorder/player`、`replay.yaml` 和会话快照。record 和 replay 只在这里相交，这是健康的相交。

---

## 特别说一下 record / replay 的关系

你说“录制回放单独写一块模板”，更准确的说法是：

- **record 是生产者，replay 是消费者**，它们不是同一个模板的两个变体。
- 二者之间只应该有**文件格式 + 配置快照**这一条契约，不应该有共享的运行时代码。
- 尤其 record 不能顺手跑算法（现在没跑，保持住）。否则录制性能受算法影响，录下来的数据也不再中立。

---

## 一个简单的自检标准

以后判断某块该不该分，用这个问题：

> **改一次检测/跟踪/火控的参数或阶段顺序，需要改几个地方？**

- 改一个地方 → 分对了
- 改两个地方 → 已经交叉了，不管它叫不叫“模块”

按这个标准，你现在最该先消除的是第 1 条（buff_thread 和 process_buff_frame 的双份编排），其次是第 2 条（模式策略重复）。第 3、4 条可以随后慢慢拆。

---

如果你认可这个切法，我可以下一步给你出一份具体的重构方案：先把共享单帧编排的 `Policy` 接口定出来，再把 `buff_thread` 改成调用它，并补一个“同一帧分别走实机路径和回放路径结果一致”的回归测试。要我做吗？

---

## 🧑 我  (12:31:13)
D:\vs\vscode\c++\huge mas\mas_vision-main不能跟这个一样吗？

---

## 🤖 助手  (12:31:13)
> 🔧 `Bash` — Explore mas_vision-main structure

> 🔧 `Bash` — Read mas_vision app entry and replay

> 🔧 `Bash` — Read mas_vision replay player and recorder

---

## 🤖 助手  (12:31:18)
> 🔧 `Bash` — Read mas_vision replay main

> 🔧 `Bash` — Read mas_vision build and armor thread

> 🔧 `Bash` — Check recording usage and main rest

> 🔧 `Bash` — Read mas_vision design notes and build

> 🔧 `Bash` — Read replay README fully

> 🔧 `Bash` — Read replay tooling scripts

> 🔧 `Bash` — Get exact line refs in mas_vision

能，而且 mas_vision 的做法本质上就是你说的思路——**实机一套、回放另起一个独立程序，但共用同一批算法源码**。mas_buff 现在其实已经是这个架构，差的只是“打包方式”。我上次说的“交叉”，按 mas_vision 的标准需要修正一下：

> **编排重复可以接受，算法重复才是交叉。**
> mas_vision 明确允许 live 和 replay 各写一套 process，只要求算法不重写。

它自己在 `CLAUDE.md:32` 就写死了这条规则：

> 回放必须继续调用生产路径中的 `ArmorDetector::ArmorDetect`、`ArmorTrack::track` 和 `ArmorShoot::shoot`，不另写一套算法流程。

---

## mas_vision 实际是怎么分的

| 层面 | mas_vision | 说明 |
|---|---|---|
| 可执行文件 | `base`（实机）+ `replay`（回放） | `apps/replay/CMakeLists.txt:24` 独立目标，`EXCLUDE_FROM_ALL` |
| 算法源码 | replay 直接编译同一批 `.cpp` | 不是复制文件，是同一个源文件参与两个 target |
| 实机编排 | `threads/armor_thread/armor_thread.cpp:19-118` | 检测→取姿态→跟踪→火控→串口 |
| 回放编排 | `apps/replay/main.cpp:115-133` `process_frame` | 自己的驱动，但只调用生产算法类 |
| 回放范围 | 只到 track，不跑 shoot | 它是 EKF/检测诊断工具，不是火控复现 |
| 录制 | Python 准备 live 的隔离副本，注入 `PairedRecorder` | `apps/replay/record.py`，不改原文件 |
| 输入 | 自己的 `player` + 回放按键 + CSV 输出 | 与实机线程完全解耦 |

关键点：**mas_vision 的回放也是“另一块模板”，而且它没有调用实机的 `armor_thread`。** 它复用的是算法类，不是实机循环。这正是你想表达的意思。

---

## mas_buff 现在和它的差别

| 维度 | mas_vision | mas_buff 现状 |
|---|---|---|
| 打包 | 两个可执行文件 | 一个 `mas_buff`，用子命令分派 |
| 算法复用 | replay target 编译同一批 `.cpp` | 共用 `buff_modules` 库 |
| 回放编排 | replay 自己写 `process_frame` | 共用 `process_buff_frame` |
| 实机编排 | `armor_thread` | `threads/buff_thread` |
| 录制 | 注入 live 副本 | 独立 `run_record`，**不跑算法** |
| 回放是否跑火控 | 否 | 是（detect→track→shoot） |

所以：

1. **架构方向上 mas_buff 已经和 mas_vision 一致**：实机循环和回放驱动分开，算法类共用。
2. 甚至 mas_buff 的回放比 mas_vision 更共享：它调的是 `process_buff_frame`，而 mas_vision 的 replay 自己写了 `process_frame`。
3. 真正的差别是 **mas_vision 把回放拆成了独立二进制，mas_buff 把它做成了子命令**。
4. mas_vision 的 `record` 是“给 live 副本打补丁”，不是独立模块；mas_buff 的 `record` 不跑算法、只采数据，反而更符合你说的“各司其职”。

---

## 如果你要“跟它一样”，只需要做 target 拆分

不用动算法，也不用重写回放逻辑，`run_replay` 直接搬到新二进制里调用即可：

```cmake
# apps/CMakeLists.txt 大致改成
add_library(buff_app STATIC
    buff/live.cpp buff/read.cpp buff/record.cpp buff/session.cpp)

add_executable(mas_buff main.cpp)
target_link_libraries(mas_buff PRIVATE buff_app)

# 回放独立程序，复用同一个 buff_app 里的 run_replay 和算法库
add_executable(mas_buff_replay replay_main.cpp)
target_link_libraries(mas_buff_replay PRIVATE buff_app)
set_target_properties(mas_buff_replay PROPERTIES EXCLUDE_FROM_ALL TRUE)
```

然后：

- `apps/replay_main.cpp`：只解析 `replay/check` 和配置路径，调用 `app::run_replay`。
- `apps/main.cpp`：去掉 `replay`、`check` 分支，只保留 `run/read/export/record`。
- `run_replay` 本身不需要改，它已经只依赖 `auto_buff`、`buff_io`、`buff_debug`，不碰相机和串口。
- 可选借鉴 mas_vision：`source_manifest.json` 记录本次回放编译的源码哈希，便于确认“回放跑的是哪版算法”。

---

## 但不建议照抄的地方

### 1. 录制方式
mas_vision 的 record 是“Python 复制整个工程 → 正则注入 recorder → 编译副本”，原因是它**不想改 live 源码**。代价是构建和路径都很绕。mas_buff 的 `run_record` 是独立进程、不跑算法、直接产出成对数据（`apps/buff/record.cpp:24`），更干净。除非你要求录制时必须完整复刻 live 的 shoot/串口时序，否则不要换成注入式。

### 2. 回放是否跑 shoot
mas_vision 回放只到 track，因为它是调 EKF 的。mas_buff 回放目前跑完整 `detect→track→shoot`，这是它的价值所在——能验证火控输出。是否保留取决于你要回放来干什么：
- 调检测/跟踪 → 可以像 mas_vision 一样关掉 shooter
- 验证整条打符链路 → 保留现在的完整链路

---

## 结论

**可以一样，而且推荐只抄它的一半：**

- ✅ 抄：独立回放可执行文件、共用算法源码、回放自己的 player/按键/输出模板、源码哈希
- ❌ 不抄：注入式录制、回放只到 track（除非你确实只需要诊断）

如果你确认要按 mas_vision 的方式拆，我可以直接把 `apps/replay_main.cpp`、`apps/main.cpp` 和 `apps/CMakeLists.txt` 改好，`run_replay` 逻辑不动，只换入口和构建目标。要我动手吗？

---

## 🧑 我  (12:37:33)
1.D:\vs\vscode\c++\huge mas\mas_buff\apps\buff\live.cpp 里面    // 实机只保存诊断数据；视频采集由record入口负责。
    const auto directory = tool::create_session_directory(config_path.parent_path() / "../output/live");
    app::save_recording_snapshot(config_path, config, directory, false);
    rm_utils::MasLog::init(tool::utf8(directory / "app.log"));

    auto modules = auto_buff::make_buff_modules(config_path);
    UserSerial::getInstance(tool::utf8(config_path.parent_path() / "serial_config.yaml"));

    std::ofstream report(directory / "detections.jsonl");
    std::ofstream metadata(directory / "frames.csv");
    if (!report || !metadata)
        throw std::runtime_error("Cannot create live reports");
        
    report << std::setprecision(17);
    tool::write_metadata_header(metadata);

    // 会话先于诊断线程创建，退出时先结束诊断再关闭会话。
    std::unique_ptr<app::Preview> preview;
    if (show)
    {
        preview = std::make_unique<app::Preview>(display, config_path, directory);
    }

    auto_buff::BuffDebug debug(config, true);
    auto_buff::PerformanceLog performance(config, directory);
弄这么多诊断那我录制的意义是啥？
2.// 应用层录制本帧实际使用的姿态，不再次查询串口。
struct SynchronizedResult
{
    auto_buff::BuffResult result;
    std::optional<FramePose> pose;
};
为啥本机实际打符的时候需要录制本帧的姿态

---

## 🤖 助手  (12:37:33)
> 🔧 `Bash` — Read report and performance log

> 🔧 `Bash` — Read session snapshot, debug and CSV schema

> 🔧 `Bash` — Show pose capture and publish lines

这两个问题其实是同一件事：**live 里那些东西不是“录制”，而是“实机运行的证据链”**。名字容易混，所以你觉得重复。

---

## 1. live 的诊断 vs record 的录制

先看两边到底产出了什么：

| 产物 | live (`run`) | record |
|---|---|---|
| 图像 / 视频 | ❌ 明确不存 | ✅ `raw.avi` 或 `images/` |
| `frames.csv` | ✅ 有，但**没有图像** | ✅ 有，且和图像严格配对 |
| `detections.jsonl` | ✅ 每帧检测/跟踪/火控结果 | ❌ |
| 配置快照 | ✅ 但 `copy_model=false`，模型不复制 | ✅ `copy_model=true`，模型一起带走 |
| `replay.yaml` | ❌ | ✅ 生成可直接回放的配置 |
| `performance.csv` | 仅 `runtime.profile=true` | ❌ |
| `app.log` | ✅ | ✅ |
| 可视化 | 仅 debug 开关打开 | ❌ |

所以两者回答的是不同问题：

- **live 诊断**：这局实机到底发生了什么？
  - 哪一帧检测到了几个候选
  - 哪一帧跟踪初始化/丢失
  - 哪一帧火控有解、为什么没发弹
  - 各阶段耗时、帧龄、姿态是否有效
  - 用的是哪份配置
- **record**：把这局的**输入**原样保存下来，之后能重跑算法。
  - 图像 + 曝光时刻 + 当帧姿态 + 模式
  - 模型和配置一起打包
  - 换检测/跟踪/火控参数后，可以反复回放同一段数据做对比

关键区别：

> **live 存的是“输出轨迹”，record 存的是“输入素材”。**
> 没有图像，就永远没法验证“检测器当时为什么没检出”；只有 `detections.jsonl`，你只能看到结果，看不到它面对的像素。

实际调参时缺一不可：

- 用 record 录一次素材；
- 用 replay 跑 N 个算法版本；
- 用 live 的同类 JSONL 格式看每次的决策轨迹。

所以 record 不冗余，它是**唯一能提供像素的入口**。live 注释里那句“视频采集由 record 入口负责”就是这个意思。

### 但这不代表 live 的诊断都该常开

现在 live 里真正**无条件**写的是：

- `detections.jsonl`（每帧完整候选，体积可能很大）
- `frames.csv`（每帧元数据）

其余本来就是条件性的：

- `performance.csv`：`runtime.profile`
- `BuffDebug` / `preview`：debug 开关
- 配置快照：每次都写，但 `copy_model=false`

如果你实际不看 `detections.jsonl`，它确实是纯开销。可以加一个 `runtime.diagnostics` 开关，默认关闭，需要复盘时再开。record 完全不受影响。

另外 `save_recording_snapshot` 这个名字在 live 里容易误导——它其实只是“会话配置快照”，不录图像也不复制模型。可以考虑改名为 `save_session_snapshot`，把“录制”这个词留给 record。

---

## 2. 为什么 live 要保存“本帧实际用的姿态”

因为 `SynchronizedResult` 保存的不是“给算法用的姿态”（算法已经用过了），而是**把算法实际消费的输入和输出冻结在一起**。

看代码顺序（`threads/buff_thread/buff_thread.cpp:57-103`）：

```cpp
SynchronizedResult output;
result.detection = detector.detect(...);            // 61
Eigen::Quaterniond pos = serial.q(frame.timestamp); // 68  ← 实际用的姿态
output.pose = FramePose{pos, current_mode};         // 69  ← 冻结
tracker->set_imu_orientation(pos);                  // 70
...
if (... || serial.getVisionMode() != current_mode)  // 87-88 模式可能已经变了
{
    result.aimed.reset();
    result.tracking.status = "frame_or_mode_changed";
}
publish(frame, output);                             // 103
```

如果 `publish` 里再去查一次串口，会出三个问题：

1. **姿态对不上**
   `serial.q()` 是持续更新的。处理完到 publish 之间串口可能已经来了新姿态，再查一次得到的就不是这帧 PnP/跟踪实际用的那个姿态。写进 `frames.csv` 的数据就和算法结果错位了。

2. **模式对不上**
   第 87 行专门检查了“处理结束时模式是否已变”。如果变了，这一帧的瞄准被作废。此时 `publish` 若重新读 `serial.getVisionMode()`，会把“新模式”写给一帧“按旧模式处理”的结果，诊断直接失真。

3. **输入不可追溯**
   `detections.jsonl` 里有跟踪状态、PnP、火控解，但如果没有当帧姿态和模式，你无法判断某次跟踪失败是：
   - 图像问题
   - 姿态跳变/延迟
   - 红蓝模式切错
   - 还是算法本身

   把这些输入和结果按帧号对齐，才是可分析的证据链。

所以 `SynchronizedResult` 的语义是：

> **result 和它实际使用的输入（pose + mode）一起返回，不允许事后重新取。**

它不会影响算法，也不占多少开销，只是每帧多存一个四元数和一个 mode。而且这也为将来“live 异常时把最近 N 帧图像+姿态 dump 出来做回放”留了口子——前提是姿态必须和帧严格配对。

---

## 建议的边界

- **live**：保留诊断，但把 `detections.jsonl` 和 `frames.csv` 收到 `runtime.diagnostics` 开关后面；配置快照改名，避免和 record 混淆。
- **record**：继续只做“输入采集”，不跑算法；这是它和 live 诊断最大的区别，不要合并。
- **pose 快照**：保留。它不是“为了录制”，而是为了让 live 的诊断结果可解释、可对齐。

要我把 live 的诊断加一个统一开关，并把 `save_recording_snapshot` 改名/拆成 live 和 record 两个明确入口吗？

---

## 🧑 我  (12:49:06)
1.frames.csv	✅ 有，但没有图像
2.detections.jsonl	✅ 每帧检测/跟踪/火控结果
配置快照	✅ 但 copy_model=false，模型不复制
4.performance.csv	仅 runtime.profile=true
有什么用。你的意思是我比赛也得开这个吗

---

## 🤖 助手  (12:49:06)
> 🔧 `Bash` — Check debug submit early return

> 🔧 `Bash` — Inspect auto_buff config runtime/debug flags

不是。**比赛不该全开，默认配置里那套 `debug: true` 是开发配置，不是比赛配置。**

这四样东西的定位是“开发/调参/赛后复盘”，不是打符运行的必要条件。逐个说：

| 产物 | 干什么用 | 开销 | 比赛建议 |
|---|---|---|---|
| `frames.csv` | 把帧号、曝光时间、接收时间、姿态、mode、pose_valid 按帧对齐。它**没有图像**，所以不能回放，只是给 `detections.jsonl` 和视频/日志提供帧索引 | 很小，一帧一行 | 可以留 |
| `detections.jsonl` | 复盘每一帧：检出几个候选、跟踪有没有初始化、火控为什么没给解、模式对不对 | **大**，且当前在 `publish` 里同步写 | 比赛关闭，或改轻量版 |
| 配置快照 | 赛后确认这局到底用了哪套参数、哪个模型路径 | 一次性，几 KB | 留 |
| `performance.csv` | 定位检测/跟踪/火控哪段慢、帧龄多少、记录/调试开销多少 | 每帧计时 + 写盘 | 只在调性能时开，比赛关 |
| debug 可视化 / preview | 实时看检测框、跟踪投影、火控曲线 | 图像拷贝 + 渲染 + 上屏 | 比赛关 |

---

## 比赛完全不开会失去什么

- 没有 `detections.jsonl`，赛后只能看视频和日志，**没法解释“这一枪为什么没打中”**：是没检出、跟踪没初始化、火控无解，还是模式切错。
- 没有 `performance.csv`，**没法知道慢在哪**。
- 但比赛的首要目标是稳定出枪，不是留证据。

所以正确的做法不是“全开/全关”，而是**分档**：

```
比赛：配置快照 + app.log + frames.csv（可留） + 轻量决策日志（推荐）
调参：+ performance.csv + debug 窗口
赛后深挖：+ detections.jsonl 完整版
```

---

## 当前代码的两个问题

1. **`detections.jsonl` 和 `frames.csv` 是无条件写的**（`apps/buff/live.cpp:101-104`，`publish` 里每帧都写）。
2. **写失败会抛异常结束整场**（`apps/buff/live.cpp:105-106`）。比赛时磁盘满、路径异常，诊断反而把比赛打停。

而 debug 可视化在默认 `config/auto_buff.yaml` 里是三个 `true`，`profile` 是 `false`。这说明现在这套配置是**台架开发配置**，直接拿去比赛不合适。

---

## 建议的比赛配置

加一个统一开关，而不是散落在各模块：

```yaml
runtime:
  mode: match          # match / debug
  diagnostics: light   # off / light / full
  profile: false
```

行为建议：

- `match + light`
  - debug 全关、preview 关、performance 关
  - 保留配置快照、`app.log`
  - `frames.csv` 保留
  - 轻量 JSONL 只写关键字段：
    ```json
    {"frame":123,"time":12.3,"mode":2,"pose_valid":true,
     "track_status":"tracking","initialized":true,"ready":true,
     "should_shoot":false,"yaw_rad":...,"pitch_rad":...,
     "pipeline_ms":...}
    ```
    异步写、有界队列；队列满时**丢诊断，不丢控制**。
- `debug + full`
  - 现在的完整 `detections.jsonl`、performance、debug 窗口全开，用于台架调参。
- 写诊断失败在 `match` 模式下只记日志，不抛异常终止比赛。

这样你比赛常用 `light`，真出问题时还有基本证据；需要深挖时临时切 `full`，并且知道代价。

---

## 直接回答你的问题

- **“有什么用”**：给开发/调参/赛后复盘解释“为什么没打中、慢在哪、用的什么参数”。它们不参与打符决策。
- **“比赛也得开这个吗”**：不需要。比赛默认应该**关掉重诊断**；最多保留配置快照、日志、`frames.csv` 和轻量决策日志。
- **“那录制的意义”**：record 提供**图像**，这是 live 诊断永远给不了的；live 的诊断只提供结果轨迹。两者不是替代关系。

要我把这套 `runtime.mode` / `runtime.diagnostics` 的分档接进 `live.cpp` 和配置里吗？可以做到比赛默认 `light`，切换 `full` 时行为和现在一致。

---

## 🧑 我  (12:52:52)
首先比赛的时候我需要的是最佳性能，不是在这调试呢你懂吗

---

## 🤖 助手  (12:52:52)
懂。**比赛时，运行路径应当只保留控制链路，不应把诊断调小后继续运行。**

这两者不是“开多少诊断”的区别，而是：

- 比赛：相机 → 检测 → 跟踪 → 火控 → `sendVision`，**不写文件、不拷贝图像、不渲染、不计时**
- 台架/回放：才需要 JSONL、CSV、性能、可视化

现在 `live.cpp` 的 `publish` 每帧都在做这些：

- `write_frame_report` 同步写 `detections.jsonl`
- `write_frame_metadata` 同步写 `frames.csv`
- `debug.submit`（关掉时早退，但仍是一次函数调用）
- `performance.write`（关掉时早退）
- 构造 `tool::Frame`、检查流状态

即使 debug 全关，`publish` 仍然存在，并且每帧都在执行。它排在 `sendVision` 之后（`buff_thread.cpp:96` → `publish` at `:103`），所以当前帧的发令不被它延迟，但它会拖慢下一帧的处理节奏，也会在比赛时引入磁盘 I/O 和异常终止风险（写失败会 throw）。

**比赛路径应该让 `publish` 几乎什么都不做：**

```cpp
// match 模式
const auto publish = [&](const CameraFrame &, threads::SynchronizedResult &)
{
    ++frames_processed;   // 只做内存计数
};
```

退出时写**一次** summary，不写每帧文件。

---

## 我建议的做法

不是加 `diagnostics: light` 这种“少开一点”，而是**把比赛和调试拆成两条路径**：

| 入口 | 内容 |
|---|---|
| `mas_buff match` | 只创建相机、串口、算法模块；无 Preview、无 BuffDebug、无 report/metadata、无 PerformanceLog；`publish` 只计数 |
| `mas_buff run` | 现在的完整诊断路径，台架用 |
| `mas_buff replay` | 独立回放，调参用 |

编译层面最好也分开：比赛 target 不链接 `buff_debug`、不包含 `report.cpp`，这样**代码里根本不存在诊断路径**，不会因为配置写错、文件路径异常、磁盘满而影响比赛。运行时开关再快，也不如编译期不存在。

比赛 target 剩下的就是：

```
hikcamera_thread + serial_thread + buff_thread + detect/track/shoot
```

没有任何 `ofstream` 在热路径上。

---

## 一句话

你要的是**比赛路径零诊断**，不是“诊断降级”。我完全同意。

要我直接加一个 `run_match` 入口（或独立 `mas_buff_match` target），把 `publish` 变成纯计数，退出时只写一行 summary 吗？现有的 `run`/`replay` 保持给台架用不变。

---

## 🧑 我  (12:56:07)
1.写日志可以留，开销小
2.detections.jsonl我要是真要调，我会自己走record模式
3.配置快照没啥用
4.performance.csv	定位检测/跟踪/火控哪段慢、帧龄多少、记录/调试开销多少依旧，我要是需要看，我自会看record
5.我在跑实机测试用的是debug 可视化

---

## 🤖 助手  (12:56:07)
> 🔧 `Bash` — Check symbol declarations and live usages

---

## 🧑 我  (12:56:34)
先不改先理解我的意思

---

## 🤖 助手  (12:56:34)
明白，先不动代码。我复述一下你的意思，你看对不对。

## 我理解的目标

**live/比赛路径只保留控制链路：**

- 相机线程 + 串口线程
- detect → track → shoot → `sendVision`
- `app.log`（开销小，保留）
- 调试可视化：**只在实机测试时按配置打开**，比赛时关闭

**live 里不再产出分析文件：**

- ❌ `detections.jsonl`
- ❌ `frames.csv`
- ❌ 配置快照
- ❌ `performance.csv`
- ❌ 任何每帧同步写文件

**分析全部走 record/replay：**

- 要调检测/跟踪/火控 → 自己走 `record` 录素材，再 `replay` 跑算法、出诊断
- 要看性能 → 同样从 `record`/`replay` 看
- live 本身不再承担“记录”和“分析”的职责

一句话：

> **live 只负责打符和写运行日志；不再记录任何逐帧数据。要分析就 record + replay。**

调试可视化保留，是因为你实机测试要用；比赛时关掉即可，不冲突。

---

## 我需要你确认两点

1. **`frames.csv` 是不是也一起去掉？**
   我的理解是：它是给 `detections.jsonl` 做帧对齐用的。`detections.jsonl` 不要了，它在 live 里就没有消费方，所以默认也删。如果你还想留一个“纯时间/姿态/mode 的轻量帧表”，告诉我。

2. **你说“要看就看 record”，但现在的 `record` 只采集图像+姿态+时间，不跑算法，也不产生 `detections.jsonl` / `performance.csv`。**
   你的意思是：
   - A. `record` 录素材，然后用 `replay` 跑算法看诊断；
   - B. 还是希望 `record` 本身也跑算法、直接出诊断？

我理解是 A，但这决定了 record 的职责边界，所以先确认。

这两点确认后，我再改 `live.cpp`。

---

## 🧑 我  (13:03:31)
1.frames.csv需要在截止时记录接收帧数和时间即可，还有其他日志，D:\vs\vscode\c++\huge mas\mas_buff\third_party\mas_log使用了吗
2.                 "配对检查    ./mas_buff check /记录目录/replay.yaml\n"
                 "导出标注视频 ./mas_buff export ../config/read.yaml\n"是干啥的
3.                 "离线检测    ./mas_buff read ../config/read.yaml\n"是不是和replay冲突了
4.                 "相机姿态采集 ./mas_buff record ../config/record.yaml\n"
串口数据记录了吗，record就是用来记录整个链路并用于回放的
5.你看看sp和rmcs他们的离线记录和回放是安插在主程序里还是咋样，我的想法是离线代码不要过度参与到实机运行代码

---

## 🤖 助手  (13:03:31)
> 🔧 `Bash` — List sibling repos and mas_log usage

> 🔧 `Bash` — Read offline read/export implementation

> 🔧 `Bash` — Find RMCS/SP repos

> 🔧 `Bash` — Inspect serial data available

> 🔧 `Bash` — Explore RMCS structure and record/replay

> 🔧 `Bash` — Explore SP vision structure and record/replay

> 🔧 `Bash` — Read RMCS executor and recorder/player interfaces

> 🔧 `Bash` — Read SP recorder and main integration

> 🔧 `Bash` — List RMCS io files and components

> 🔧 `Bash` — Read RMCS recorder and player implementation

> 🔧 `Bash` — Read mas_log implementation and build

> 🔧 `Bash` — Count and list MAS_LOG usage

> 🔧 `Bash` — Check per-frame logging and pose interpolation

> 🔧 `Bash` — Read RMCS player publish and component flow

---

## 🤖 助手  (13:04:12)
> 🔧 `Bash` — Check camera frame metadata and stats

> 🔧 `Bash` — Check paired recorder and record data fields

## 1. `frames.csv` 和日志

**mas_log 已经用了，而且用得不少。** `third_party/mas_log` 被编译进 `buff_utils`（`third_party/CMakeLists.txt:2-3`），全项目有 93 处 `MAS_LOG_*` 调用，覆盖 live/record/replay/read、相机、串口、线程。

它的性质：

- 异步 spdlog：65536 队列、1 个后台线程、`overrun_oldest`（`mas_log.cpp:43-49`）
- 文件默认 `debug`，控制台默认 `warn`（`mas_log.hpp:31-34`）
- 每 3 秒 flush，warn 立即 flush

所以 `app.log` 的开销确实小，可以留。需要注意的只有两处可能的逐帧日志：

- 串口 HEX 打印在 `debug_` 开关后面（`user_serial.cpp:236-247`），默认关
- 相机丢帧日志 5 秒节流一次（`hikcamera_capture.cpp:125-135`）

比赛时把文件级别降到 `info` 或 `warn` 就够了。

**`frames.csv` 你的想法是对的：实机不需要逐帧写。** 截止时写一条汇总即可。但要注意：

- 现在的 `summary.json` 只有 `frames`（处理帧数）和 `elapsed_seconds`
- “接收帧数”不等于处理帧数：相机线程可能丢帧，`HikCamera` 内部有 `dropped_total_` / `queue_dropped_total_`（`hikcamera.hpp:79-80`），但**没有暴露给外部**
- 要准确记录接收帧数，需要在相机线程加一个计数器，退出时和 `first/last t_recv` 一起写进 summary

建议实机退出时只写一次：

```
received_frames, processed_frames, dropped_grab, dropped_queue,
first_recv_ns, last_recv_ns, elapsed_s
```

这比 `frames.csv` 更符合你的需求，也不需要逐帧 I/O。

---

## 2. `check` 和 `export` 是干什么的

### `check`

`./mas_buff check /记录目录/replay.yaml` → `run_replay(config, true)`（`apps/buff/replay.cpp:80-91`）

它**不跑算法**：

- 打开视频和 `frames.csv`
- 解码全部视频帧
- 校验：视频帧数 = CSV 行数、帧号连续、时间递增、四元数合法
- 不加载模型、不创建输出目录、不显示、不碰相机/串口

用途：**回放前的数据完整性检查**，尤其是拷贝/移动过 session 之后，确认视频和 CSV 没丢帧、没错位。

### `export`

`./mas_buff export ../config/read.yaml` → `run_read(config, true)`

它本质是 `read` 加上“导出视频”开关：

- 读任意图片/视频
- 跑完整 detect→track→shoot
- 写标注视频、`detections.jsonl`、可选 mask
- 不碰硬件

用途：从任意素材导出带检测/跟踪/火控标注的视频。

**它们是否必要：**

- `check` 有价值，但前提是你经常搬动/手工拼 session；如果 record 产出的 session 你直接拿来回放，可以不做
- `export` 基本是 `read` 的别名。`read` 已经可以根据配置保存标注视频。如果不需要“单独导出一条视频”这个入口，可以删掉 `export`，只留 `read`

---

## 3. `read` 和 `replay` 冲突吗

**不是硬冲突，但确实重叠。**

| | `read` | `replay` |
|---|---|---|
| 输入 | 任意图片/视频 | 成对视频 + `frames.csv` |
| 姿态 | 没有，固定相机假设 | 每帧记录的四元数 |
| 模式 | 未知 | 记录的红/蓝/大小符 |
| 时间 | 媒体时间 | 记录曝光时间 |
| 播放控制 | 无 | 暂停/继续/单步 |
| 用途 | 快速看单条视频、导出 | 复现实机算法链路 |

两者都调用 `process_buff_frame`，都写 `detections.jsonl`、都做标注和性能记录（`read.cpp:53-90`，`replay.cpp:189-239`）。所以**重叠的是驱动层代码，不是算法**。

处理方式二选一：

1. **保留两者，明确分工**：`replay` 是权威回放路径；`read` 只是“没有 CSV/姿态的快速媒体检查”
2. **合并**：把 `read` 做成 `replay` 的一种输入源（有 CSV 用记录姿态，没有就固定相机），只保留一个离线驱动

如果目标是“离线代码不要参与实机”，这两个都属于离线侧，应该一起放到独立 target，而不是和 live 编译在一起。

---

## 4. record 到底记了什么，够不够“整个链路”

当前 `record` 每帧保存（`apps/buff/record.cpp:121-140`）：

- 图像
- 曝光时间 `timestamp_ns`
- 接收时间 `t_recv_ns`
- 模式 `serial.getVisionMode()`
- 姿态 `serial.q(camera.timestamp)`（有效时）

**没有记录：**

- 原始串口接收包（完整 `ReceivePacket`、裁判系统数据、陀螺仪）
- 串口发送历史（`sendVision` 的 yaw/pitch/found/fire_advice）
- 算法输出（检测/跟踪/火控）

所以要看你说的“整个链路”指哪一条：

### A. 视觉算法链：detect → track → shoot

**当前 record 足够。** 因为算法只消费：

- 图像
- 曝光时间
- 模式/颜色
- 曝光时刻的 IMU 四元数

这些都在。回放时重算检测、跟踪、火控，就能复现视觉决策链。record 不需要自己跑算法，否则录制吞吐受影响，数据也不再是“中立的输入”。

### B. 串口/电控链：IMU 原始流、裁判系统、发出的云台/开火指令

**当前 record 不够。** 需要额外记录：

- 每帧原始串口 RX 包（含时间戳）
- 每帧 `sendVision` 的发送内容
- 需要的话，裁判系统和陀螺仪

但这是“系统级回放”，和视觉算法回放是两件事。

### 还有一个时间基差异

实机 `shooter->update({Clock::now()})` 用的是**实时主机时间**（`buff_thread.cpp:76`）；回放用的是**记录时间**（`buff_replay.cpp:61`）。所以回放是确定性的，但不和实机逐位一致。如果要求火控完全复现，record 还需要保存每帧 shooter 更新用的时间，或保存曝光到处理的延迟。

**建议先明确：** 你说的“整个链路”是 A 还是 B。A 现在够；B 要加串口原始记录。

---

## 5. SP / RMCS 是怎么放离线代码的

### SP（`开源/sp_vision_25-main`）

- `tools::Recorder` 是主程序里的一个对象，`src/standard.cpp:39` 创建，主循环里调用 `recorder.record(img,q,t)`（`src/standard.cpp:67`，很多版本是注释掉的）
- recorder 自己有一个保存线程，和算法解耦
- SP 里**没有 player/replay**，离线分析是独立的 test/calibration 程序
- 不同模式是不同 main 文件/可执行文件

结论：**录制代码安插在主程序里，但只是工具对象，不在算法里；没有独立的回放程序。**

### RMCS（`开源/rmcs_auto_aim_v2-main`）

这是最值得参考的：

- recorder 和 player 都是独立 plugin component
- 通过 `config/executor.yaml:6-12` 选择启用哪个：
  - 实机：`AutoAimCapturerComponent` + `AutoAimComponent`
  - 离线：`AutoAimPlayerComponent` + `AutoAimComponent`
- recorder 订阅 `/gimbal/auto_aim/camera_frame`
- player **发布同一个 topic**，替代相机
- `AutoAimComponent` 完全不知道帧来自相机还是回放（`src/component.cpp:27` 起）

结论：**离线代码是 I/O 组件，不是算法的一部分；通过“换输入源”切换实机/回放。** 这是最干净的分离方式。

### mas_buff 现状

- 单个 `mas_buff` 二进制，`buff_app` 同时包含 live/read/record/replay/session（`apps/CMakeLists.txt:2-3`）
- live 链接了整个 `buff_app`，所以**离线代码在链接层面参与了实机二进制**
- 这和你“离线代码不要过度参与实机”的想法是冲突的

### 建议

不需要照搬 RMCS 的 ROS component 体系，但可以照搬它的边界：

```
buff_core       算法 + process_buff_frame + report/annotation
mas_buff        live：相机 + 串口 + 算法 + 日志 + 可选 debug
mas_buff_record 采集：相机 + 串口 + PairedRecorder，不跑算法
mas_buff_replay 离线：replay + check + read + export，不碰硬件
```

这样：

- live 二进制里没有 player、没有 `read`、没有 `check`、没有 `PairedRecorder`
- 离线程序里没有相机/串口
- 两者只共用 `buff_core`

record 虽然用硬件，但它也是独立工具，不要塞进 live 二进制，否则 live 又会链接录制代码。

---

## 一句话总结

1. `mas_log` 已经在用，异步、开销小，保留；`frames.csv` 改成退出时一条汇总，接收帧数需要相机线程暴露计数器。
2. `check` 是回放前的数据校验；`export` 是 `read` 的视频导出别名，可删可留。
3. `read` 和 `replay` 不冲突，但驱动层重叠；建议明确 `replay` 才是权威回放路径。
4. record 记的是“视觉链路输入”，足够重跑 detect→track→shoot；不包含原始串口 RX/TX。先确认你说的“整个链路”是视觉链还是串口链。
5. RMCS 的做法最接近你的想法：离线是独立 I/O 组件/可执行文件，不进入实机算法；mas_buff 应该按 target 把 live、record、replay 拆开。

---

## 🧑 我  (13:11:58)
1.check和export可以放在tools里拿python写，放在app层该不够格
2.没有记录：

原始串口接收包（完整 ReceivePacket、裁判系统数据、陀螺仪）
串口发送历史（sendVision 的 yaw/pitch/found/fire_advice）
算法输出（检测/跟踪/火控）
可以不记录
当前即可，只要能用来跑完整回放调参即可
3.RMCS（开源/rmcs_auto_aim_v2-main）
这是最值得参考的：

recorder 和 player 都是独立 plugin component
通过 config/executor.yaml:6-12 选择启用哪个：
实机：AutoAimCapturerComponent + AutoAimComponent
离线：AutoAimPlayerComponent + AutoAimComponent
recorder 订阅 /gimbal/auto_aim/camera_frame
player 发布同一个 topic，替代相机
AutoAimComponent 完全不知道帧来自相机还是回放（src/component.cpp:27 起）
结论：离线代码是 I/O 组件，不是算法的一部分；通过“换输入源”切换实机/回放。 这是最干净的分离方式。可以，是否跟mas_vision-main的一样

---

## 🤖 助手  (13:11:58)
## 1. `check` / `export` 放 tools 用 Python：可以，但 `export` 要分清是哪一种

### `check` → 纯 Python，完全够格

它只做数据校验：

- 用 `opencv-python` 解码视频，数字帧数
- 用 `csv` 读 `frames.csv`
- 校验帧数一致、帧号连续、时间递增、四元数合法

不需要 C++，不需要模型，不需要编译。mas_vision 的 `apps/replay/run.py check` 就是这么做的。放到 `tools/check_pairs.py` 很合适，Windows 上也能跑。

### `export` → 要看它到底干什么

现在的 `export` 是 `run_read(config, true)`，它会：

- 读任意视频
- **跑完整 detect→track→shoot**
- 写标注视频和 `detections.jsonl`

这不是纯 Python 能替代的，因为算法是 C++。所以有三种处理：

1. **直接删掉 `export`**
   replay 本身就有 `save_annotated: true`，回放时已经会输出标注视频。如果只是要“看带标注的录制”，不需要独立 export。
2. **如果 export 只是“把已有的 `detections.jsonl` 画回视频”**
   那可以用 Python + OpenCV 写，放 tools。
3. **如果要“对任意视频跑算法并导出”**
   它本质是离线 C++ 程序，不是 Python 工具。可以把它放进独立的离线可执行文件，Python 只做一层启动包装。

我的建议：`check` 用 Python；`export` 如果只是画图就 Python，如果要跑算法就归到离线 C++ 程序，或者直接删掉。这样 `apps/main.cpp` 就只剩实机入口，不再挂离线杂项。

---

## 2. record 当前字段够用，不用加串口原始包

确认你的判断是对的。回放完整视觉链只需要：

- 图像
- 曝光时间
- 曝光时刻的四元数
- 模式/颜色
- 接收时间（诊断用）

算法输出（检测/跟踪/火控）由回放重算，不需要记录。原始串口包、裁判系统、发送历史也不需要，因为算法不消费它们。

所以 `record` 保持现在的字段即可：

```
image + timestamp_ns + t_recv_ns + pose + mode + pose_valid
```

有两点保持现状就行：

- `pose_valid=0` 的帧要保留，回放时明确进入 `waiting_for_pose`，不要静默丢帧
- record 的 config/model 快照要留，它是让 session 可以搬走、可以重跑的前提；live 的快照才删

唯一要知道的差异：实机 shooter 用 `Clock::now()`，回放用记录时间。对调参来说回放是确定性的，这正是你要的。

---

## 3. RMCS 和 mas_vision 是不是一样的

**原理一样，机制不一样。**

| | RMCS | mas_vision | mas_buff 建议 |
|---|---|---|---|
| 离线入口 | 同进程 plugin component | 独立 `replay` 可执行文件 | 独立 `mas_buff_replay` |
| 切换方式 | `executor.yaml` 换组件 | 换可执行文件 | 换可执行文件 |
| 算法复用 | 同一个 AutoAim 组件 | 编译同一批算法源码 | 链接同一个 `buff_core` |
| 离线代码是否进实机 | 插件库存在，但未配置不加载 | 不进实机二进制 | 不进实机二进制 |
| 构建复杂度 | 高（pluginlib / ROS executor） | 低 | 低 |
| 适合 mas_buff | 不太适合 | 适合 | 适合 |

两者的共同点：

> **离线只换输入源和输出，不碰算法。**

RMCS 是在同一个 executor 里，用 config 把 `Capturer` 换成 `Player`，`AutoAimComponent` 不知道帧从哪来。

mas_vision 是直接编译两个可执行文件，replay 有自己的 main、player、播放控制、输出，但调用同一批算法源码。

### 对 mas_buff 来说，选 mas_vision 的方式更合适

因为你没有 ROS/pluginlib 基础设施，硬套 RMCS 的组件体系成本太高。mas_vision 的 target 拆分更直接：

```
buff_core        算法 + process_buff_frame + 回放需要的公共计算
mas_buff         live：相机 + 串口 + 算法 + 日志 + 可选 debug
mas_buff_record  采集：相机 + 串口 + PairedRecorder，不跑算法
mas_buff_replay  离线：player + 播放控制 + 标注输出 + read
tools/check_pairs.py    Python 数据校验
tools/export_*.py       如果只是画图
```

这样：

- live 二进制里没有 player、没有 `check`、没有 `export`、没有 `PairedRecorder`
- replay 二进制里没有相机、没有串口
- 两者只共享 `buff_core`

### 但不要照抄 mas_vision 的 record

mas_vision 的 record 是 Python 复制整个工程、注入 `PairedRecorder`、再编译副本。它这么做是为了不改原 live 源码。

mas_buff 的 `record` 已经是独立采集工具，不跑算法，不注入，比 mas_vision 的干净。**保留 mas_buff 的 record，只借 mas_vision 的 replay target 拆分。**

---

## 结论

1. `check` 用 Python 放 tools；`export` 如果只是画图也放 tools，如果要跑算法就归到离线 C++ 程序或直接删。
2. record 当前字段就够完整回放调参，不加串口原始包；保留 `pose_valid` 和 record 的配置/模型快照。
3. RMCS 和 mas_vision 的思想相同：离线只换 I/O，不碰算法。机制上 mas_buff 更适合 mas_vision 的“独立可执行文件 + 共用 core 库”，而不是 RMCS 的插件组件体系。

按这个方向，下一步就是拆 `mas_buff` / `mas_buff_record` / `mas_buff_replay` 三个 target，并把 check/export 移出 app 层。要我先出一份具体的文件/CMake 拆分方案吗？

---

## 🧑 我  (13:16:06)
1.直接删掉 export
2.可以
3.RMCS 和 mas_vision 的思想相同：离线只换 I/O，不碰算法。机制上 mas_buff 更适合 mas_vision 的“独立可执行文件 + 共用 core 库”，而不是 RMCS 的插件组件体系。可以直接总结一下就行

---

## 🤖 助手  (13:16:06)
定稿总结如下。

## 最终职责划分

| 入口 | 职责 | 不包含 |
|---|---|---|
| `mas_buff` | 实机打符：相机、串口、detect→track→shoot、`sendVision`、`app.log` | 检测日志、帧表、配置快照、性能表、回放、录制 |
| `mas_buff_record` | 采集：图像 + 曝光/接收时间 + 姿态 + 模式，生成可回放 session | 算法、检测输出、原始串口包 |
| `mas_buff_replay` | 离线：成对回放 + 播放控制 + 标注输出；`read` 也归这里 | 相机、串口 |
| `tools/check_pairs.py` | Python 校验视频帧数与 `frames.csv` 是否配对 | 算法、模型 |
| `export` | 删除 | — |

## 实机 live

保留：

- `app.log`（MasLog 异步，比赛降到 info/warn）
- 调试可视化（仅实机测试时按配置打开，比赛关闭）

删除：

- `detections.jsonl`
- `frames.csv` 逐帧写入
- 配置快照
- `performance.csv`
- 任何热路径文件 I/O

退出时只写一次汇总：

```
received_frames, processed_frames, dropped_grab, dropped_queue,
first_recv_ns, last_recv_ns, elapsed_s
```

其中“接收帧数”需要相机线程暴露计数器，`HikCamera` 目前有 `dropped_total_` / `queue_dropped_total_`，但没对外提供。

## record

保持现状，字段足够完整回放调参：

```
image + timestamp_ns + t_recv_ns + pose + mode + pose_valid
```

不记录原始串口包、裁判系统、发送历史、算法输出。保留 record 的配置/模型快照，让 session 可搬移、可重跑。

## 架构结论

- 一个 `buff_core`：算法 + `process_buff_frame`，live 和 replay 共用。
- live、record、replay 各自独立可执行文件，各自独立驱动。
- 离线只换 I/O，不碰算法。
- 不引入 RMCS 的 plugin component 体系。
- 离线代码不链接进实机二进制。

一句话：

> **live 只管打符和写日志；record 只管采集；replay 只管离线重跑；check 交给 Python；export 删除。算法只有一份，驱动各走各的。**

确认这个方向后，我再按它改 `apps/main.cpp`、`apps/CMakeLists.txt` 和 `live.cpp`。
