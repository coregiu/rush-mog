# 分层架构：基础能力层 / 场景编排层

> 版本：v1.0（2026-11）
> 目的：把“能做什么”（能力）和“在什么场景下按什么顺序做”（场景）拆开，
> 让新场景只写编排，不重复实现相机、导航、语音、安全。
> 相关实现：`core/scenarios/`、`ros_nodes/voice_scenario_node.py`、
> `edge_server/edge_web.py`、`web/audio.js`。

---

## 1. 为什么要分层

“语音找拖鞋”表面上是一个功能，实际横跨：

1. 语音识别（手机/终端）→ 文本；
2. 文本意图解析 → “找 slipper”；
3. 自主探索/建图；
4. 相机 + 深度/激光检测目标；
5. 规划路线、确认目标、接近目标；
6. TTS 播报“找到了”。

如果把这些写进一个节点，会迅速变成不可测试的“大泥球”，而且第二个场景
（比如“跟着我”“去巡逻”“把杯子拿过来”）要复制粘贴大半逻辑。

因此拆成四层：

```text
┌────────────────────────────────────────────────────────────────────┐
│ L4 交互/接入层                                                      │
│  手机网页语音、HTTP /api/voice、ROS /edge/voice_command、dashboard   │
└───────────────────────────────┬────────────────────────────────────┘
                                │ 文本 / 状态
┌───────────────────────────────▼────────────────────────────────────┐
│ L3 场景编排层（Scenario）                                            │
│  voice_scenario_node + FindObjectScenario                          │
│  “听懂了 → 开始找 → 播报进度 → 找到/失败 → 播报结果”                  │
└───────────────────────────────┬────────────────────────────────────┘
                                │ Action / Topic（只调用能力）
┌───────────────────────────────▼────────────────────────────────────┐
│ L2 基础能力层（Capability）                                          │
│  语音播报 / 目标检测 / 建图定位 / 导航 / 搜索探索 / 安全 / 底盘        │
│  纯逻辑模块 + 薄 ROS 适配节点，可以被任何场景复用                       │
└───────────────────────────────┬────────────────────────────────────┘
                                │ ROS2 / 串口 / I2C / UART / ALSA
┌───────────────────────────────▼────────────────────────────────────┐
│ L1 平台与驱动层（Platform / Driver）                                 │
│  ROS2 Humble、相机/雷达/IMU 驱动、STM32 固件、蓝牙音箱、Linux          │
└────────────────────────────────────────────────────────────────────┘
```

**核心规则：**

- **L3 不直接发布 `/cmd_vel`，不解析串口，不调用 OpenCV。** 它只调用 L2 暴露的
  Action/Topic，并根据结果决定“说什么、下一步做什么”。
- **L2 不知道“找拖鞋”这种业务语义。** `object_search_node` 只负责“搜索某个
  label”，`target_detector_node` 只负责“检测/测距/投影”，谁来调用、为什么调用
  与它们无关。
- **L2 的纯逻辑不 import rclpy。** 例如 `SearchCoordinator`、`InstructionParser`、
  `FindObjectScenario` 都能在 pytest 里直接实例化。
- **安全永远在能力层兜底。** 场景层可以取消任务，但急停、障碍、超时、低电量
  仍由 `safety_manager_node` / `velocity_mux_node` 保证。

---

## 2. 基础能力层（L2）

### 2.1 能力清单

| 能力 | 纯逻辑 | ROS/进程适配 | 对外契约 |
|---|---|---|---|
| 语音输入 | 浏览器 Web Speech / 外部 ASR | `audio.js` → `POST /api/voice`；或外部节点发布 | `/edge/voice_command`（`std_msgs/String`） |
| 意图解析 | `instruction_parser.py` | 场景节点内调用 | `MissionSpec` / `VoiceCommand` |
| 语音播报 | `speaker.py` | `edge_server_node` | `/body/say`（`std_msgs/String`） |
| 目标检测 | `object_detector.py` | `target_detector_node` | `/center/target_detections` |
| 测距/定位投影 | `target_localizer.py` | 同上 | `TargetDetection.position` |
| 目标跟踪 | `target_tracker.py` | 同上 | `track_id` / `hits` / `stable()` |
| 建图/前沿 | `occupancy_map.py` | `map_manager_node` | `/map`、`/center/frontiers`、`/center/map_info` |
| 定位 | `imu_monitor.py`、`pose_utils.py` | `localization_node`、EKF/AMCL | `/center/pose`、`/center/localization_status` |
| 导航 | `nav_supervisor.py` | `navigation_node` | `/center/navigate_to`（Action） |
| 搜索/探索 | `object_search.py` | `object_search_node` | `/center/search_object`、`/center/explore`（Action） |
| 安全 | `safety_checker.py` | `safety_manager_node` | `/center/safety_state` |
| 速度仲裁/底盘 | `velocity_mux.py`、`body_drive.py` | `velocity_mux_node`、`edge_server_node` | `/cmd_vel`、STM32 UART |

### 2.2 “搜索”能力已经是一个组合能力

`object_search_node` 内部并不只是一次检测，它组合了：

```text
前沿探索 → 发现候选 → 停车确认 → 接近目标 → 返回 found + target_pose
```

所以场景层不需要自己写“先转一圈、再靠近”的流程，只需要：

```text
调用 /center/search_object(target_label="slipper")
```

这是分层的关键：**能力层可以是组合的，但它的语义必须是通用的**
（“搜索某个标签”），而不是“语音找拖鞋”。

### 2.3 能力层的稳定性要求

新增场景时不应修改 L2。只有以下情况才改 L2：

- 新增传感器/执行器；
- 新增一种检测器（例如 YOLO-World 开放词表）；
- 修改导航/搜索的安全策略；
- 新增 ROS Action/Topic 契约。

---

## 3. 场景编排层（L3）

### 3.1 场景 = 事件 + 状态机 + 副作用

一个场景不直接“干活”，而是：

1. 接收事件（语音文本、搜索进度、搜索结果、急停）；
2. 推进自己的状态机；
3. 产出一组**效果（effect）**；
4. 由 ROS 适配节点把效果翻译成 Action/Topic 调用。

当前实现：

```text
core/scenarios/voice_intent.py   文本 → VoiceCommand(find/cancel/ignore)
core/scenarios/find_object.py    FindObjectScenario：找东西场景状态机
ros_nodes/voice_scenario_node.py ROS 适配：订阅语音、调用 Action、播报
```

### 3.2 “找东西”场景状态机

```text
                语音“找拖鞋”
IDLE ─────────────────────────────► SEARCHING
  ▲                                   │  │  │
  │                                   │  │  └── 搜索失败 ──► FAILED
  │                                   │  └───── 搜索超时 ──► FAILED
  │                                   └──────── 找到 ──────► SUCCEEDED
  │
  ├──────── 语音“别找了”/急停/新目标 ◄── SEARCHING（取消/切换）
  │
  └──────── 结果播报完成 / 下一次指令
```

状态值：`idle` / `searching` / `succeeded` / `failed` / `canceled`。

**效果类型：**

| effect | 含义 | 适配节点动作 |
|---|---|---|
| `speak` | 播报一句话 | 发布 `/body/say` |
| `start_search` | 开始搜索某个 label | 调用 `/center/search_object` Action |
| `cancel_search` | 取消当前搜索 | `cancel_goal_async()` |
| `publish_status` | 状态有变化 | 发布 `/center/scenario_status` |

> 纯逻辑场景只返回 `ScenarioEffect`，不 import rclpy，因此可以像普通 Python
> 对象一样单测；ROS 适配节点只做“翻译”，不写业务判断。

### 3.3 场景的输入/输出契约

**输入事件：**

| 事件 | 来源 | 场景处理 |
|---|---|---|
| 语音文本 | `/edge/voice_command` | 解析 find/cancel/ignore |
| 搜索进度 | `/center/search_status` | 更新 detail；里程碑播报 |
| 搜索结果 | `SearchObject` Action Result | 成功/失败播报，终止状态 |
| 动作不可用 | ActionClient 等待超时 | 播报“寻找功能未就绪” |
| 急停 | `/center/safety_state` | 取消搜索 + 播报 |
| 新目标 | `/edge/voice_command` | 取消旧搜索，启动新搜索 |

**输出：**

| 输出 | 类型 | 用途 |
|---|---|---|
| `/body/say` | `std_msgs/String` | TTS 播报 |
| `/center/scenario_status` | `std_msgs/String` (JSON) | 网页/dashboard 展示 |
| `/center/search_object` | Action | 调用搜索能力 |

### 3.4 后续场景只需要写编排

例如“跟随人”：

```text
语音“跟着我” → FollowScenario
  → 订阅 /center/target_detections(label=person)
  → 调用 /center/navigate_to / 速度控制能力
  → “跟丢了” → 搜索能力 → “找到了，继续跟”
```

例如“取杯子”：

```text
FindObjectScenario（找到杯子）
  → ArmCapability（抓取）
  → NavigateTo（送到指定点）
  → Speak（“放好了”）
```

这些都不需要改 `object_search_node` / `target_detector_node`。

---

## 4. 交互/接入层（L4）

### 4.1 为什么语音入口放在网页/终端

项目早期尝试过车载 ASR，但电机噪声导致识别率很差；当前设计把**语音识别**
放在手机/终端（Web Speech API），树莓派只接收文本。这样：

- 识别算力在手机，不占树莓派；
- 不受电机噪声影响；
- 任何能发 HTTP 的终端都能当语音入口。

### 4.2 三条等价入口

```text
A. 手机网页语音
   Web Speech API → POST /api/voice {"text":"帮我找一下拖鞋"}
   → edge_server_node 发布 /edge/voice_command

B. 任意 HTTP 客户端
   curl -X POST -d '帮我找一下拖鞋' http://<pi>/api/voice

C. 任意 ASR 节点/脚本
   ros2 topic pub /edge/voice_command std_msgs/String "{data: '帮我找一下拖鞋'}"
```

三条入口最终都进入同一个场景节点，行为一致。

### 4.3 状态回显

`voice_scenario_node` 以 1 Hz 发布 `/center/scenario_status`：

```json
{
  "ok": true,
  "state": "searching",
  "target_label": "slipper",
  "display_name": "拖鞋",
  "detail": "explore frontier (1.20, 0.80)",
  "search_state": "exploring",
  "elapsed_sec": 12.4,
  "confidence": 0.0,
  "last_utterance": "帮我找一下拖鞋",
  "updated_ms": 1760000000000
}
```

Edge Server 缓存该 JSON，`GET /api/voice` 返回给网页；网页据此更新
“正在找 / 找到了 / 没找到”的提示，不需要轮询 ROS。

---

## 5. 端到端数据流（找拖鞋）

```text
手机麦克风
  │ Web Speech API
  ▼
POST /api/voice  "帮我找一下拖鞋"
  │
  ▼
edge_server_node ──publish──► /edge/voice_command
  │
  ▼
voice_scenario_node
  ├─ InstructionParser：找 → search，拖鞋 → slipper
  ├─ FindObjectScenario：SEARCHING
  ├─ publish /body/say "好的，我去找拖鞋"
  └─ Action /center/search_object(target_label=slipper)
        │
        ▼
object_search_node
  ├─ 无地图 → /center/map_control(start) + 建图
  ├─ /center/frontiers + /center/pose → 选前沿
  ├─ /center/navigate_to → navigation_node → /nav/cmd_vel
  ├─ /center/target_detections ← target_detector_node ← camera + depth + lidar
  ├─ 候选确认 → 接近
  └─ Result(found=true, target_pose, confidence)
        │
        ▼
voice_scenario_node
  ├─ publish /body/say "找到拖鞋了，就在前面大约0.6米"
  └─ publish /center/scenario_status → Edge /api/voice → 网页状态
```

安全链路始终独立存在：

```text
safety_manager_node → /safety/cmd_vel_override → velocity_mux_node → /cmd_vel
```

场景层不能绕过它；急停时场景节点也会收到 `/center/safety_state` 并取消搜索。

---

## 6. 失败与降级

| 情况 | 行为 |
|---|---|
| 没有地图 | `search_object.explore_if_not_found=true` → 自动开始建图/探索 |
| 没装 Nav2 | `navigation.backend=auto` → 内置直航 |
| 检测器未配置/模型缺失 | 搜索超时 → 播报“没有找到拖鞋”，网页显示 failed |
| 搜索 Action 未启动 | 2 s 等待后播报“寻找功能还没准备好”，不进入 searching |
| TTS/音箱不可用 | 仍发布状态；网页显示结果，日志记录失败 |
| 急停 | 立即取消搜索并播报“已急停，停止寻找” |
| 浏览器不支持 Web Speech | 用 `/api/voice` 文本入口或外部 ASR 发布 topic |
| 浏览器语音需要联网 | 可改用外部离线 ASR（Vosk/Whisper）发布 `/edge/voice_command`，场景层无需改动 |
| 找不到 label | 使用支持该 label 的检测器（YOLO 自定义模型/YOLO-World），并保持
`instruction.object_labels` 的 canonical label 与检测输出一致 |

---

## 7. 新增一个场景的步骤

1. 在 `core/scenarios/` 写纯逻辑状态机，输入事件、输出 `ScenarioEffect`；
2. 在 `ros_nodes/` 写薄适配节点，订阅输入、调用 L2 Action、发布播报/状态；
3. 在 `config/*.yaml` 加 `scenario.<name>` 参数；
4. 在 `launch/edge_server.launch.py` 增加 `start_<name>` 开关；
5. 在 `tests/` 加纯逻辑单测；
6. 更新本文档与 README 的入口说明。

**不要**在第 1 步 import rclpy；**不要**在第 2 步写业务判断；**不要**让场景节点
直接发布 `/cmd_vel`。

---

## 8. 代码位置对照

| 层 | 位置 |
|---|---|
| L1 平台/驱动 | `edge_server/drivers/`、`launch/lidar.launch.py`、`launch/astra.launch.py`、`software/body_control/` |
| L2 能力纯逻辑 | `edge_server/core/*.py`（object_search、navigation、target_*、occupancy_map、speaker…） |
| L2 ROS 适配 | `ros_nodes/target_detector_node.py`、`object_search_node.py`、`navigation_node.py`、`map_manager_node.py`、`localization_node.py`、`safety_manager_node.py`、`edge_server_node.py` |
| L3 场景纯逻辑 | `edge_server/core/scenarios/voice_intent.py`、`find_object.py` |
| L3 场景 ROS 适配 | `ros_nodes/voice_scenario_node.py` |
| L4 交互 | `web/audio.js`、`edge_server/edge_web.py`（`/api/voice`）、`edge_server_node.py`（`/edge/voice_command`） |

---

## 9. 设计取舍

- **为什么不把场景逻辑塞进 `task_manager_node`？**
  `task_manager` 是通用任务状态机/任务入口，适合 VLA、建图、导航、搜索等
  一次性 mission；语音找东西包含“多轮播报、可打断、可切换目标”的交互语义，
  单独放场景层更清晰。两者可以共存：场景层调用搜索 Action，`task_manager`
  仍然可以被 HMI 或其他任务调用。

- **为什么用 `std_msgs/String` 而不是新增自定义状态消息？**
  场景状态是低频、可读性优先的观测数据，JSON 字符串便于网页直接使用，也避免
  因为新增 `.msg` 而必须同时升级 `custom_msgs`。如果未来需要强类型/频繁传输，
  再替换成 `custom_msgs/ScenarioStatus` 即可，场景纯逻辑不用改。

- **为什么场景层不直接读 `/center/target_detections`？**
  “搜索/确认/接近”的完整策略已经在 `SearchCoordinator` 里，场景层重复消费
  检测结果会形成第二套搜索逻辑。场景层只消费搜索 Action 的进度/结果。
