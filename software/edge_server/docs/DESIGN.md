# edge_server 设计文档

> 版本：v3.0（2026-11）
> 本文档由原 `DESIGN.md`、`ARCHITECTURE_LAYERS.md`、`CODE_MAP.md` 合并而成，是 `edge_server` 的唯一设计说明。
> 适用硬件：树莓派 4B/5 或 Jetson Orin Nano + STM32 + 2D 激光 + ICM-20948 + 4 路编码器。
>
> 配套文档：
> - [部署与运维手册](../deploy/DEPLOY.md)
> - [健康检查契约](../HEALTHCHECK.md)
> - [硬件缺口与选型](../../../../hardware/SLAM_NAV.md)
>
> 代码基线：2026-11-14，`software/edge_server` + `software/body_control`。

**文档地图**

| 章节 | 内容 |
|---|---|
| §0–§1 | 版本状态、设计目标与原则 |
| §2 | 分层架构、L2/L3/L4、硬件拓扑、Edge Server 与 ROS 边界 |
| §3 | 能力边界（输入/输出） |
| §4 | 软件结构、进程形态、节点总览、启动命令 |
| §5 | 模块详细设计 |
| §6 | 关键调用链与数据流 |
| §7 | ROS 话题/服务/Action、HTTP/Web 接口 |
| §8 | STM32 ASCII 协议与固件结构 |
| §9 | 配置系统 |
| §10–§11 | 异常处理、实时性与性能 |
| §12–§14 | 启动顺序、测试、当前限制与路线图 |

---

## 0. 版本与验证状态

| 项目 | 状态 |
|---|---|
| 代码 | `software/edge_server`（ament_python，ROS 2 Humble） |
| 消息/动作 | `software/custom_msgs`（v0.2.0） |
| 构建 | `colcon build --packages-select custom_msgs edge_server` 通过 |
| 单元测试 | `pytest tests -q`：179 passed（含远程 VLA/握手 26 项、语音场景 9 项、`/api/voice` 1 项） |
| 健康检查 | `test_health_monitor.py` + `test_health_http.py`：29 passed；ROS/HTTP 手工联调通过 |
| VLA/安全端到端 | `end_to_end_check.py`：7/7 passed |
| SLAM/导航/搜索联调 | `slam_nav_check.py` + `fake_slam_robot.py`：6/6 passed |
| 语音找东西联调 | `voice_scenario_check.py` + `fake_slam_robot.py`：3/3 passed（找到/取消/状态链） |
| 真实 VLA 模型 | 仓库不带权重；Center Server 提供 SmolVLA，离线回退规则桩，接口固定 |
| STM32 协议 | 当前 ASCII 固件由 `edge_server_node` 直连；二进制协议主机侧保留为 legacy |

---

## 1. 设计目标

`edge_server` 是 Rush-MOG 的**决策与任务中心**。在 v1 的 VLA 推理、安全、反向操作之上，v2/v3 增加五类能力：

1. **SLAM 建图**：消费 2D 激光与里程计，控制 slam_toolbox 建图、保存/加载地图、发布前沿。
2. **定位**：消费 ICM-20948、轮式里程计、AMCL/TF，输出定位健康度与 map 系位姿，门控导航。
3. **自主导航**：把“去某点/探索/回家”转成 Nav2 或内置直航的速度指令，带目标校验、停滞检测、超时。
4. **目标搜索**：相机检测 + 激光测距 + 地图投影 + 前沿探索 + 目标确认 + 接近，返回目标位姿。
5. **健康检查**：ROS 节点/话题 + 全部连接设备，HTTP 供同机 Edge Server/dashboard 调用。

设计原则：

- **分层**：任务层（mission）→ 导航层（nav）→ 速度层（mux）→ 执行器（STM32）。
- **单一速度出口**：所有来源经 `velocity_mux_node` 仲裁后只发布一个 `/cmd_vel`。
- **可降级**：没有 Nav2 用内置直航；没有 slam_toolbox 用内置 PGM/YAML 存图；没有深度相机用激光/bbox 估距。
- **可观测**：每个子系统都有状态话题，便于 HMI/日志/故障定位。
- **安全优先**：急停、障碍、数据超时、低电量、定位丢失都能覆盖普通任务。

---

## 2. 系统架构

### 2.1 四层模型

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

| 层 | 职责 | 主要位置 |
|---|---|---|
| L4 交互/接入 | 语音/HTTP/网页/dashboard，只收发文本与状态 | `web/`、`edge_server/edge_web.py` |
| L3 场景编排 | 解析意图、推进场景状态机、决定“说什么/下一步做什么” | `core/scenarios/`、`ros_nodes/voice_scenario_node.py` |
| L2 基础能力 | 搜索、导航、检测、建图、播报、安全、底盘 | `core/*.py` + `ros_nodes/*_node.py` |
| L1 平台/驱动 | ROS 2、传感器驱动、STM32 固件、蓝牙、Linux | `drivers/`、`launch/`、`software/body_control` |

**分层规则：**

- **L3 不直接发布 `/cmd_vel`，不解析串口，不调用 OpenCV。** 它只调用 L2 暴露的 Action/Topic，并根据结果决定“说什么、下一步做什么”。
- **L2 不知道“找拖鞋”这种业务语义。** `object_search_node` 只负责“搜索某个 label”，`target_detector_node` 只负责“检测/测距/投影”。
- **L2 的纯逻辑不 import rclpy。** `SearchCoordinator`、`InstructionParser`、`FindObjectScenario` 都能在 pytest 里直接实例化。
- **安全永远在能力层兜底。** 场景层可以取消任务，但急停、障碍、超时、低电量仍由 `safety_manager_node` / `velocity_mux_node` 保证。
- **依赖只向下。** 新增场景只写 L3 编排，不修改 L2。

### 2.2 基础能力层（L2）

| 能力 | 纯逻辑 | ROS/进程适配 | 对外契约 |
|---|---|---|---|
| 语音输入 | 浏览器 Web Speech / 外部 ASR | `audio.js` → `POST /api/voice`；或外部节点发布 | `/edge/voice_command`（`std_msgs/String`） |
| 意图解析 | `instruction_parser.py` | 场景节点内调用 | `MissionSpec` / `VoiceCommand` |
| 语音播报/音箱 | `speaker.py` | `edge_server_node` | `/body/say`、`/api/speaker/*`、`/body/audio_status` |
| 麦克风录音 | `microphone.py` | `edge_server_node` | `/api/microphone/*`、`/edge/audio`、`/body/mic_status` |
| 电源监控 | `power.py` | `edge_server_node` | `/api/power`、`/body/power_status` |
| 目标检测 | `object_detector.py` | `target_detector_node` | `/center/target_detections` |
| 测距/定位投影 | `target_localizer.py` | 同上 | `TargetDetection.position` |
| 目标跟踪 | `target_tracker.py` | 同上 | `track_id` / `hits` / `stable()` |
| 建图/前沿 | `occupancy_map.py` | `map_manager_node` | `/map`、`/center/frontiers`、`/center/map_info` |
| 定位 | `imu_monitor.py`、`pose_utils.py` | `localization_node`、EKF/AMCL | `/center/pose`、`/center/localization_status` |
| 导航 | `nav_supervisor.py` | `navigation_node` | `/center/navigate_to`（Action） |
| 搜索/探索 | `object_search.py` | `object_search_node` | `/center/search_object`、`/center/explore`（Action） |
| 安全 | `safety_checker.py` | `safety_manager_node` | `/center/safety_state` |
| 速度仲裁/底盘 | `velocity_mux.py`、`body_drive.py` | `velocity_mux_node`、`edge_server_node` | `/cmd_vel`、STM32 UART |

**“搜索”本身已是组合能力。** `object_search_node` 内部组合了“前沿探索 → 发现候选 → 停车确认 → 接近目标 → 返回 found + target_pose”，场景层只需调用：

```text
/center/search_object(target_label="slipper")
```

这是分层的关键：**能力层可以是组合的，但语义必须通用**（“搜索某个标签”），而不是“语音找拖鞋”。

**能力层稳定性要求。** 新增场景时不应修改 L2；只有以下情况才改：

- 新增传感器/执行器；
- 新增一种检测器（例如 YOLO-World 开放词表）；
- 修改导航/搜索的安全策略；
- 新增 ROS Action/Topic 契约。

### 2.3 场景编排层（L3）

“语音找拖鞋”表面上是一个功能，实际横跨语音识别、意图解析、自主探索、目标检测、导航、TTS 播报；写进一个节点会迅速变成不可测试的“大泥球”，第二个场景（跟随、巡逻、取物）还要复制大半逻辑。因此把“能做什么”（L2 能力）和“在什么场景下按什么顺序做”（L3 场景）拆开。

一个场景不直接“干活”，而是：**接收事件 → 推进自己的状态机 → 产出一组效果（effect）→ 由 ROS 适配节点翻译成 Action/Topic 调用。**

当前实现：

```text
core/scenarios/voice_intent.py   文本 → VoiceCommand(find/cancel/ignore)
core/scenarios/find_object.py    FindObjectScenario：找东西场景状态机
ros_nodes/voice_scenario_node.py ROS 适配：订阅语音、调用 Action、播报
```

**效果类型：**

| effect | 含义 | 适配节点动作 |
|---|---|---|
| `speak` | 播报一句话 | 发布 `/body/say` |
| `start_search` | 开始搜索某个 label | 调用 `/center/search_object` Action |
| `cancel_search` | 取消当前搜索 | `cancel_goal_async()` |
| `publish_status` | 状态有变化 | 发布 `/center/scenario_status` |

> 纯逻辑场景只返回 `ScenarioEffect`，不 import rclpy，因此可以像普通 Python 对象一样单测；ROS 适配节点只做“翻译”，不写业务判断。

**新增一个场景的步骤：**

1. 在 `core/scenarios/` 写纯逻辑状态机，输入事件、输出 `ScenarioEffect`；
2. 在 `ros_nodes/` 写薄适配节点，订阅输入、调用 L2 Action、发布播报/状态；
3. 在 `config/*.yaml` 加 `scenario.<name>` 参数；
4. 在 `launch/edge_server.launch.py` 增加 `start_<name>` 开关；
5. 在 `tests/` 加纯逻辑单测；
6. 更新本文档与 README 的入口说明。

**约束：** 第 1 步不要 import rclpy；第 2 步不要写业务判断；场景节点不要直接发布 `/cmd_vel`。

**设计取舍：**

- **为什么不把场景逻辑塞进 `task_manager_node`？** `task_manager` 是通用任务状态机/任务入口，适合 VLA、建图、导航、搜索等一次性 mission；语音找东西包含“多轮播报、可打断、可切换目标”的交互语义，单独放场景层更清晰。两者可共存：场景层调用搜索 Action，`task_manager` 仍可被 HMI 或其他任务调用。
- **为什么用 `std_msgs/String` 而不是新增自定义状态消息？** 场景状态是低频、可读性优先的观测数据，JSON 字符串便于网页直接使用，也避免因新增 `.msg` 而必须同时升级 `custom_msgs`。未来需要强类型/高频传输时，再替换成 `custom_msgs/ScenarioStatus`，场景纯逻辑不用改。
- **为什么场景层不直接读 `/center/target_detections`？** “搜索/确认/接近”的完整策略已经在 `SearchCoordinator` 里，场景层重复消费检测结果会形成第二套搜索逻辑；场景层只消费搜索 Action 的进度/结果。

**后续场景示例**（都不需要改 `object_search_node` / `target_detector_node`）：

```text
“跟着我” → FollowScenario：订阅 target_detections(label=person) → navigate_to / 速度能力
           → “跟丢了” → 搜索能力 → “找到了，继续跟”
“取杯子” → FindObjectScenario（找到杯子）→ ArmCapability（抓取）
           → NavigateTo（送到指定点）→ Speak（“放好了”）
```

### 2.4 交互/接入层（L4）

项目早期尝试过车载 ASR，但电机噪声导致识别率很差；当前把**语音识别**放在手机/终端（Web Speech API），树莓派只接收文本：

- 识别算力在手机，不占树莓派；
- 不受电机噪声影响；
- 任何能发 HTTP 的终端都能当语音入口。

**三条等价入口**最终都进入同一个场景节点，行为一致：

```text
A. 手机网页语音
   Web Speech API → POST /api/voice {"text":"帮我找一下拖鞋"}
   → edge_server_node 发布 /edge/voice_command

B. 任意 HTTP 客户端
   curl -X POST -d '帮我找一下拖鞋' http://<pi>/api/voice

C. 任意 ASR 节点/脚本
   ros2 topic pub /edge/voice_command std_msgs/String "{data: '帮我找一下拖鞋'}"
```

**状态回显：** `voice_scenario_node` 以 1 Hz（变化即发）发布 `/center/scenario_status`：

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

Edge Server 缓存该 JSON，`GET /api/voice` 返回给网页；网页据此更新“正在找 / 找到了 / 没找到”，不需要轮询 ROS。

### 2.5 硬件与部署拓扑

```text
┌──────────────────────────────────────────────────────────────┐
│ 云端 / PC（可选）                                             │
│   edge_server：VLA、任务编排、HMI、日志、地图管理             │
└──────────────────────────┬───────────────────────────────────┘
                           │ DDS / Wi-Fi（建议 5GHz 或有线）
┌──────────────────────────▼───────────────────────────────────┐
│ 机载 Linux（Pi 4B/5 或 Orin Nano）                             │
│   Edge Server :80/:8080（网页/API/图传 + STM32 串口）           │
│   ROS2 Humble：激光/相机驱动、EKF、slam_toolbox、Nav2、          │
│   edge_server 的导航/搜索/定位节点                            │
└───────┬──────────────────┬──────────────────┬────────────────┘
        │ USB/UART         │ USB/CSI          │ I2C
┌───────▼────────┐ ┌───────▼────────┐ ┌───────▼────────┐
│ LD19/LD06 激光  │ │ USB/CSI 相机    │ │ ICM-20948      │
└────────────────┘ └────────────────┘ └────────────────┘
        │
┌───────▼──────────────────────────────────────────────────────┐
│ STM32F103RCT6/F411：4 路编码器 + 4 路 PWM + 轮速 PID + 看门狗  │
└───────┬──────────────────────────────────────────────────────┘
        │
┌───────▼──────────────────────────────────────────────────────┐
│ TB6612/DRV8871/BTS7960 + 12V 电池 + 5V/5A + 6V/5A + 急停      │
└──────────────────────────────────────────────────────────────┘
```

| 选项 | 车上 | 云端/PC | 说明 |
|---|---|---|---|
| A（推荐） | Pi 跑驱动/EKF/SLAM/Nav2/搜索 | VLA/任务编排可远程 | 实时闭环在车上，网络差也可用 |
| B | Orin Nano 跑全部 | 无 | 本地 VLA，成本高 |
| C | 仅 Pi + STM32 | edge_server 远程 | 降级演示；Web/底盘仍在 Pi 本地闭环 |

代码对 A/B/C 相同；区别只是节点放在哪台机器、DDS 网络怎么配。网络方面，Pi 可用 hostapd/dnsmasq 提供 `RUSH-MOG` AP（`192.168.3.26`），也可接入现有局域网；手机/PC 直接访问 Pi 的 80 端口。

### 2.6 Edge Server 与 ROS 的边界

ESP32 与树莓派两个模块已合并为**一台树莓派 Edge Server**：

- `edge_server_node` 在 Pi 上提供 80 端口网页/API、8080 MJPEG；
- 直接通过 Pi UART 连接 STM32 USART3（115200 8N1），发送 ASCII 方向+PWM 命令，解析 `$B,...` 编码器/悬崖遥测；
- 网页手柄/摇杆手动控制与 ROS `/cmd_vel` 共用同一个串口出口：**网页手动优先**，松开/停止后 ROS 导航恢复；ROS raw 命令保留第二字节并带 1s 租约；
- ROS 侧仍由 `edge_server` 的 VLA/SLAM/导航/搜索节点组成，与 Edge Server 同机运行；`health_monitor_node` 的 HTTP `:8090` 由 Edge Server 同机代理；
- 旧的 `esp32_bridge_node`（Wi-Fi HTTP 桥）和 `base_bridge_node`（二进制串口）仅作为历史兼容保留，`robot.yaml` 默认 `base.transport=edge_serial`。

---

## 3. 能力边界

### 3.1 输入

| 类型 | 话题 | 说明 |
|---|---|---|
| 激光 | `/scan` 或 `/edge/radar` | 2D LaserScan，10Hz+ |
| 图像 | `/camera/color/image_raw` 或 `/edge/video` | bgr8，目标检测/VLA；Astra RGB |
| 音频 | `/edge/audio` 或 `/audio/raw` | 可选，缺失不阻塞 |
| 里程计 | `/odom` 或 `/robot/odometry` | 轮式/EKF |
| IMU | `/imu/data` | ICM-20948，100Hz |
| 关节 | `/joint_states` | 机械臂/轮子 |
| 电池 | `/battery_state` | 电压/百分比 |
| 地图 | `/map` | OccupancyGrid |
| 定位 | `/amcl_pose`、TF | map→odom→base_link |
| 任务 | `/center/execute_task`、`/center/execute_mission`、`/center/mission_request` | 用户/HMI/语音 |

### 3.2 输出

| 类型 | 话题/动作 | 说明 |
|---|---|---|
| 底盘速度 | `/cmd_vel` | 唯一最终速度出口 |
| VLA 原始速度 | `/cmd_vel_raw` | 经 mux 仲裁 |
| 导航速度 | `/nav/cmd_vel` | Nav2/直航 |
| 反向速度 | `/reverse/cmd_vel` | 回退/手动 |
| 安全覆盖 | `/safety/cmd_vel_override` | 最高优先级 |
| 机械臂 | `/arm/trajectory` | JointTrajectory |
| 任务请求 | `/task/request`、`/center/mission_request` | 内部/外部 |
| 状态 | `/center/status`、`/center/mission_status`、`/center/nav_status`、`/center/search_status`、`/center/localization_status`、`/center/safety_state`、`/center/velocity_status`、`/center/map_info`、`/center/frontiers`、`/center/target_detections` | 可观测性 |
| 健康 | `/center/health`、`/center/healthcheck`、HTTP `:8090/ros/health` | 系统/设备健康 |
| 位姿 | `/center/pose` | map 系，供导航/搜索/目标投影 |

---

## 4. 软件结构

### 4.1 仓库目录

```text
software/
├── edge_server/
│   ├── docs/
│   │   ├── design/DESIGN.md       # 本文档（设计 + 架构 + 代码地图）
│   │   ├── deploy/DEPLOY.md       # 部署与运维
│   │   └── HEALTHCHECK.md         # 健康检查契约
│   ├── edge_server/               # Python 库（尽量不依赖 ROS）
│   │   ├── edge_control.py        # 命令映射 / MotionState / 遥测解析
│   │   ├── edge_web.py            # HTTP 服务器 / 路由 / 静态文件 / 模块代理
│   │   ├── http_health.py         # 健康接口数据组装
│   │   ├── core/                  # 业务核心与纯逻辑
│   │   ├── core/scenarios/        # L3：voice_intent、find_object
│   │   ├── drivers/icm20948.py    # ICM-20948 寄存器级 I2C 驱动
│   │   ├── utils/                 # 配置、时钟、日志
│   │   └── config/                # config.yaml（仿真）、robot.yaml（实车）
│   ├── ros_nodes/                 # 19 个节点入口（setup.py console_scripts）
│   ├── web/                       # 手柄页 / dashboard / JS / CSS
│   ├── launch/                    # edge_server / robot_bringup / slam / nav2 / lidar / astra
│   ├── tests/                     # pytest + 端到端 / 联调脚本
│   └── setup.py                   # entry_points + launch/config/web 安装清单
├── custom_msgs/                   # 消息 / 服务 / Action（v0.2.0）
├── body_control/                  # STM32F103 固件（FreeRTOS）
│   ├── src/                       # main / controller / ps2_receiver / task_manager / ...
│   └── inc/command.h              # 协议常量、波特率、命令枚举
├── center_server/                 # PC 侧上位机（独立部署）
└── tools/                         # 联调脚本
```

`core/` 中的纯逻辑：`action_planner`、`data_preprocessor`、`body_drive`、`health_monitor`、`imu_monitor`、`instruction_parser`、`nav_supervisor`、`object_detector`、`object_search`、`occupancy_map`、`odometry`、`pose_utils`、`reverse_handler`、`safety_checker`、`speaker`、`microphone`、`power`、`state_machine`、`target_localizer`、`target_tracker`、`velocity_mux`、`vla_engine`。

### 4.2 进程形态

**当前部署（systemd）**：`edge_server.service` 常驻 `edge_server_node` + `health_monitor_node`。`lidar_node` 不在该服务内，由可选的 `rush-mog-lidar.service`（`ConditionPathExists=/dev/rush_lidar`）或全栈 launch 启动。

| 进程 | 入口 | 作用 | 端口 |
|---|---|---|---|
| `edge_server_node` | `ros_nodes/edge_server_node.py:main` | 网页/API、串口、`/cmd_vel`→STM32、STM32→`/odom`、蓝牙音箱/语音播报、HFP 麦克风录音、电源监控 | 80、8080 |
| `health_monitor_node` | `ros_nodes/health_monitor_node.py:main` | 话题/设备健康检查 | 127.0.0.1:8090 |
| `lidar_node`（可选服务/全栈） | `ros_nodes/lidar_node.py:main` | LD19/D6 UART → `/scan`、静态 TF、`/lidar/status` | 无 |

**全栈（可选）**：`robot_bringup.launch.py` 会再拉起 `edge_server.launch.py` 里的 VLA/感知/导航/定位/速度仲裁/目标搜索等节点。**它会自己再起一个 `edge_server_node`**，所以和 systemd 服务二选一，不能同时跑（80 端口冲突）。

| launch | 内容 |
|---|---|
| `edge_server.launch.py` | 全量节点，参数裁剪用 `start_*=true/false` |
| `robot_bringup.launch.py` | 实车：全量 + `base_transport:=edge_serial` |
| `slam_stack.launch.py` | 只包 `slam_toolbox`，不占 80 端口 |
| `nav2_stack.launch.py` | 只包 Nav2 navigation + localization |

### 4.3 节点总览

| # | 节点 | 职责 | 关键输入/输出 |
|---|---|---|---|
| 1 | `center_bridge_node` | 图像/激光/里程计软同步，缓存音频/关节/IMU | → `/center/synced_data`、`/center/robot_state` |
| 2 | `vla_inference_node` | 多模态预处理 + Center SmolVLA（规则桩兜底，异步单飞） | → `/center/action_intent`、`/center/inference_result` |
| 3 | `action_generator_node` | 单步意图 → `/cmd_vel_raw`，发布前安全校验 | → `/arm/trajectory` |
| 4 | `reverse_operation_node` | 急停/回退/机械臂复位/任务撤销/手动接管 | → `/reverse/cmd_vel` |
| 5 | `safety_manager_node` | 障碍/数据/电量/定位/急停 | → `/center/safety_state`、`/safety/cmd_vel_override` |
| 6 | `task_manager_node` | 状态机、VLA 任务、建图/导航/搜索任务编排 | → `/center/status`、`/center/mission_status` |
| 7 | `velocity_mux_node` | 多源速度仲裁 | → `/cmd_vel`、`/center/velocity_status` |
| 8 | `localization_node` | 位姿选择、定位健康度、初始位姿 | → `/center/pose`、`/center/localization_status` |
| 9 | `map_manager_node` | 地图消费、前沿、存取、建图控制 | → `/center/map_info`、`/center/frontiers` |
| 10 | `navigation_node` | `/center/navigate_to`，Nav2/直航后端 | → `/nav/cmd_vel`、`/center/nav_status` |
| 11 | `target_detector_node` | 颜色/YOLO 检测、测距、map 投影、跟踪 | → `/center/target_detections` |
| 12 | `object_search_node` | 前沿探索、目标确认、接近 | → `/center/search_status` |
| 12a | `voice_scenario_node` | 场景编排（L3）：语音文本 → 解析 → 搜索 → TTS 播报、取消/切换/急停 | ↔ `/edge/voice_command`、`/center/scenario_status`、`/body/say`、`/center/search_object` |
| 13 | `edge_server_node` | Pi Edge Server：80 网页/API、8080 MJPEG、STM32 ASCII 串口、运动仲裁、悬崖遥测 | ↔ `/cmd_vel`、`/odom`、`/joint_states`、`/base/status`、`/body/cliff_status`、`/center/safety_state`、`/api/voice` |
| 13a | `lidar_node` | LD19/D6 UART 驱动，47 字节帧解析/分箱 | → `/scan`、`/lidar/status`、`base_link→laser_link` TF |
| 13b | `base_bridge_node` | 旧版二进制串口桥（legacy） | ↔ `/odom`、`/imu/data`、`/battery_state` |
| 13c | `esp32_bridge_node` | 旧 ESP32 Wi-Fi HTTP 桥（legacy） | ↔ 旧 ESP32 协议 |
| 14 | `imu_node` | Pi I2C 直连 ICM-20948 | → `/imu/data` |
| 15 | `health_monitor_node` | 节点/设备健康、HTTP 网关 | → `/center/health`、`/center/healthcheck`、`:8090` |

### 4.4 启动命令

```bash
# 仿真/桌面（默认 config.yaml）
ros2 launch edge_server edge_server.launch.py

# 实车（robot.yaml + 底盘串口）
ros2 launch edge_server robot_bringup.launch.py

# 可选外部栈
ros2 launch edge_server slam_stack.launch.py
ros2 launch edge_server nav2_stack.launch.py map:=/path/to/map.yaml
```

> 实机启停、systemd 单元、发布与排障见[部署与运维手册](../deploy/DEPLOY.md)。

---

## 5. 模块详细设计

### 5.1 通信与软同步（`center_bridge_node`）

v1 用 5 路严格/近似同步，真实机器人没有麦克风时会阻塞整条链路。v2 改为：

- `image + laser + odom` 三路 `ApproximateTimeSynchronizer`（窗口 100–120 ms）；
- `audio / joint_states / imu` 采用最近值缓存，缺失时填空消息；
- 以图像帧为主参考，发布 `SyncedSensorData`；
- `RobotState` 单独发布 odom + joint。

QoS：

- 传感器流 `SensorDataQoS`（best effort），兼容真实驱动；
- `/center/status`、`/center/safety_state` 用 TRANSIENT_LOCAL，后启动节点也能拿到最后状态；
- `/map` 发布 TRANSIENT_LOCAL，订阅 VOLATILE（兼容 slam_toolbox 与 map_server）。

### 5.2 多模态预处理（`data_preprocessor.py`）

- 图像：BGR/RGB/gray → 224×224 → float32 CHW [0,1]；
- 激光：`LaserScan.ranges` → float32 一维；
- 音频：PCM s16le → log-mel（numpy 实现，避免 librosa 依赖）；
- 状态：odom `x/y/yaw`、关节字典；
- 打包为 `SyncedDTO` 给 VLA。

### 5.3 VLA 推理（`vla_engine.py`）

- 规则桩：停止/前进/后退/左转/右转/左移/右移；
- 任务意图：建图/探索/搜索/导航/回家；
- `predict(data) -> ActionIntentDTO`，字段：`task_id, target, intent, value, confidence, reverse_request, task_type, object_label, goal_x/y/yaw, waypoint, source`；
- 真实模型替换 `_load_model()` 与 `predict()` 即可，接口不变；
- 置信度 < `confidence_threshold` → `needs_confirmation` → `PAUSED`。

**远程 VLA（v3）**：真实 SmolVLA 推理由 `software/center_server` 提供，`vla_engine.backend` 三档：

| backend | 行为 |
|---|---|
| `auto` | 发现 Center 则远程推理；超时/报错/未发现时回退规则桩 |
| `remote` | 只走 Center；`remote.fallback_to_rules=false` 时失败发 `stop`（confidence 0） |
| `stub` | 只用规则桩，不访问网络 |

- **地址发现**：Center 启动/心跳时 `POST /api/center/handshake`（JSON 或表单）上报 IP、端口、`infer_url`；Edge 记录快照，`vla_inference_node` 每 `remote.discovery_interval_sec`（默认 5s）从 `remote.discovery_url`（默认 `http://127.0.0.1/api/center`）拉取地址，调用 Center 的 `/v1/infer`；
- Center 返回 `base_cmd`（第一步速度）→ 按幅值映射成单个意图：前进/后退/左移/右移/左转/右转/停止；`ActionIntentDTO.source` 标记 `stub`/`remote`；
- 远程调用在 `vla_inference_node` 的 `ThreadPoolExecutor(1)` 里执行：单飞 + 只保留最新帧，不阻塞 ROS 定时器；急停/切模式后丢弃过期结果；
- 失败后进入 `retry_after_sec`（默认 5s）降级窗口并告警；`auto` 在恢复后自动回到远程推理；
- 配置：`vla_engine.remote.*`（`timeout_sec`、`max_rate_hz`、`api_key`、`image_key`、`confidence`、`fallback_to_rules` 等）；握手字段与部署见 `software/edge_server/README.md` §5.5.1 和 `software/center_server/deploy/README.md`。

### 5.4 指令解析与任务编排（`instruction_parser.py` + `task_manager_node`）

文本 → `MissionSpec`：

| 关键词 | mission_type | 说明 |
|---|---|---|
| 建图/地图/slam/map | `map` | 建图 + 探索 + 保存 |
| 探索/巡检/explore | `explore` | 前沿探索到覆盖率 |
| 找/寻找/搜索/find | `search` | 目标标签从 `object_labels` 提取 |
| 导航/前往/去/navigate | `navigate` | 命名航点或 `x=.. y=.. yaw=..` |
| 回家/返回/home | `return_home` | `home_pose` |
| 其它 | `vla` | 单步动作 |

航点支持别名：

```yaml
instruction:
  waypoints:
    kitchen: {x: 1.2, y: 0.8, yaw: 0.0, aliases: ["厨房", "kitchen"]}
```

任务入口：

- `ExecuteTask`：VLA 单步；指令命中任务关键词时自动转长任务（已验证“找杯子”）；
- `ExecuteMission`：显式 `map/explore/search/navigate/return_home`；
- `/center/mission_request`：HMI/语音后台任务，`TaskRequest` 带 `task_type`/目标字段。

状态机（`state_machine.py`）：

```text
IDLE ──recv_data──> RECEIVING_DATA ──start_infer──> INFERENCING ──infer_ok──> EXECUTING
  │                       │                              │
  │                       └──── start_mapping/nav/search ─────> MAPPING / NAVIGATING / SEARCHING
  │                                                                   │
  └────────────── reverse_req ────────────────────────────────────────┴──> REVERSING
                                                                          │
                                              task_done/task_failed ──────┴──> TASK_SUCCESS / TASK_FAILED
```

- `MAPPING/NAVIGATING/SEARCHING` 与 `EXECUTING` 互斥；
- 反向操作/急停/模式切换可抢占；
- 任务超时/低置信度/数据超时 → `ERROR`/`PAUSED`/`IDLE`。

### 5.5 SLAM 与地图（`map_manager_node`）

- 订阅 `/map`（slam_toolbox/cartographer/map_server 均可）；
- 每秒发布 `MapInfo`（分辨率、尺寸、原点、known_ratio、frontier_count、map_file）；
- 提取 frontier：free 与 unknown 的 8 邻域边界，BFS 聚类，按 `size/(distance+0.5)` 选点；大图自动下采样（`frontier_max_cells`）；
- `MapControl` 服务：
  - `start`：进入建图状态；
  - `stop`：停止建图；
  - `save`：优先 slam_toolbox `SaveMap`，失败回退内置 PGM+YAML；
  - `load`：读 PGM+YAML，发布 `/map` 供 AMCL/Nav2；
  - `reset`：清空并调用 slam_toolbox `Reset`（若可用）。
- PGM/YAML 与 map_server 兼容：occupied=0、free=254、unknown=205；YAML 含 resolution/origin/thresholds；
- 地图目录：`map_manager.map_dir`（默认 `~/.ros/rush_mog/maps`，实车 `~/rush_mog/maps`）。

### 5.6 定位与 ICM-20948（`localization_node` + `imu_node` + EKF）

#### 5.6.1 坐标树

```text
map ──(AMCL / slam_toolbox)──> odom ──(robot_localization EKF)──> base_link
                                                                  ├── laser_link
                                                                  ├── camera_link
                                                                  └── imu_link
```

- 建图阶段：slam_toolbox 发布 `map→odom`；
- 定位阶段：AMCL 发布 `map→odom`；
- EKF 发布 `odom→base_link`，融合 `/odom` + `/imu/data`；
- 两者不要同时发布 `map→odom`。

#### 5.6.2 EKF 配置（`config/ekf.yaml`）

- `two_d_mode: true`，`world_frame: odom`；
- `odom0`：`x,y,yaw,vx,vy,vyaw`；
- `imu0`：`yaw,vyaw`，`imu0_relative: true`；
- 磁力计不直接融合；由 `ImuMonitor` 判断 `mag_ok` 后做慢速互补。

#### 5.6.3 ICM-20948 驱动（`drivers/icm20948.py`）

- I2C 地址 0x68/0x69，WHO_AM_I=0xEA；
- Bank2 配置：GYRO_CONFIG_1/ACCEL_CONFIG、采样率、DLPF；
- Bank0 读 0x2D 起 14 字节（accel+gyro+temp）；
- AK09916 通过 `INT_PIN_CFG.BYPASS_EN` 直连 0x0C；CNTL2=0x08（100Hz）；
- 单位：accel m/s²、gyro rad/s、mag µT、temp °C；
- 校准：`accel_bias/gyro_bias/mag_bias/mag_scale`；
- 输出 `sensor_msgs/Imu`，`orientation_covariance[0] = -1`（交给 EKF）。

#### 5.6.4 定位健康度

`ImuMonitor` 计算：

- 频率/gap、加速度模长、陀螺饱和、磁力计模长与基线偏差；
- 静止时估计 z 轴零偏；
- 平面航向：陀螺积分 + 磁航向慢速校正（`mag_blend`）；
- `quality` 0..1，`mag_ok`、`online`、`yaw_variance`。

`localization_node`：

- 位姿来源优先级：`/amcl_pose`（<1s）→ TF `map→base_link` → `/odom`（降级）；
- 发布 `/center/pose`、`/center/localization_status`；
- `quality`：amcl 0.95、tf 0.75、odom 0.35，按 IMU/odom 频率扣分；
- 导航 goal 要求 `quality >= navigation.min_localization_quality`（默认 0.35）；
- `/center/set_initial_pose` → 转发 `/initialpose`，重置航向估计。

#### 5.6.5 磁力计使用原则

- 电机/电池大电流会让磁力计失真；`mag_ok=false` 时不参与校正；
- 只用慢速互补，不硬切 yaw；
- 校准硬铁/软铁；安装远离大电流线；
- ICM-20948 已 NRND，可用 ICM-42688-P + 磁力计替代。

### 5.7 导航（`navigation_node` + `nav_supervisor.py`）

Action：`/center/navigate_to`（`NavigateTo`）。

| backend | 行为 |
|---|---|
| `nav2` | 转发到 Nav2 `/navigate_to_pose`，反馈/取消/超时 |
| `direct` | 内置直航，发布 `/nav/cmd_vel`；带目标校验、障碍减速、停滞检测 |
| `auto` | Nav2 可用则 Nav2，否则 direct |

直航控制：

```text
dx,dy = goal - pose
dist = hypot(dx,dy)
yaw_err = wrap(atan2(dy,dx) - yaw)
if |yaw_err| > 0.8: 原地转
else:
  body = R(-yaw) * (dx,dy)
  vx = k*body.x*scale, vy = k*body.y*scale (holonomic)
  wz = k_ang*yaw_err
  近距离减速；前方障碍停车/减速
```

安全：

- goal 非有限、容差非法、地图外、占据栅格 → 拒绝；
- map 系 goal 要求 `map_ready` 且定位质量达标；
- 目标落在障碍附近时吸附到最近自由栅格；
- 停滞（`progress_monitor`）、超时、急停、取消 → 回零；
- 无激光数据且 `require_scan` → 拒绝运动。

Nav2 集成：

- `nav2_msgs` 是可选依赖（懒加载）；未安装时自动 direct；
- Nav2 最终速度必须重映射到 `/nav/cmd_vel`（`launch/nav2_stack.launch.py` 用 `SetRemap`），否则会和 mux 抢 `/cmd_vel`；
- `nav2_params.yaml` 为麦轮/2D 激光起步配置，需实车调参。

### 5.8 目标检测与定位（`target_detector_node`）

```text
图像 -> 检测器（颜色/YOLO） -> bbox
     -> 相机模型：bearing = atan((u-cx)/fx)
     -> 距离：激光方位窗口最近值 / bbox 高度估距
     -> 目标点 base_link -> 用 /center/pose 投影到 map
     -> IoU 跟踪器 -> track_id + 稳定性
     -> TargetList
```

- 颜色检测：HSV 阈值 + 形态学 + 轮廓；无模型依赖；
- YOLO：`ultralytics` 可选，`model_path` 配置；
- 激光窗口默认 ±3°；取最近有效距离（目标更近时优先目标，可能被前景遮挡）；
- bbox 高度估距：`d = fy * real_height / bbox_height_px`；
- 目标位置方差：激光 0.05²、bbox 0.20²、无距离 1.0²；
- 跟踪器：IoU 匹配、`max_age`、`hits`、位置历史；用于“确认不是闪烁检测”。

### 5.9 目标搜索与自主探索（`object_search_node` + `object_search.py`）

Action：

- `/center/search_object`：搜索指定标签；
- `/center/explore`：纯探索到覆盖率阈值。

状态机：

```text
WAITING_MAP
    │ map ready
    ▼
EXPLORING ──候选──> CONFIRMING ──确认──> APPROACHING ──到达──> FOUND
    │                   │                    │
    │ 无前沿/超时        └──丢失──> EXPLORING  └──不可达──> FAILED / 重试
    ▼
FAILED
```

规则：

- 前沿选择：`size/(distance+0.5)`；过滤太远/太小/黑名单；
- 确认：同一 track `hits >= confirm_hits` 且持续 `confirm_time`，位置稳定；
- 发现候选先取消探索导航，停车确认，避免开过去错过；
- 接近点：目标沿“目标→机器人”方向退 `standoff_distance`，朝向目标；
- 导航失败的前沿进入黑名单（`blacklist_sec`），换点；
- 目标丢失超过 `target_lost_timeout` → 回到探索；
- 到达距离 ≤ `standoff + approach_tolerance` → FOUND；
- 覆盖率 ≥ `explore_coverage` 或超时 → FAILED/FOUND；
- 支持 `explore_if_not_found`：无地图时先调用 `map_control start`。

### 5.10 速度仲裁（`velocity_mux_node`）

| 优先级 | 源 | 话题 | 超时 |
|---|---|---|---|
| 0 | 安全覆盖 | `/safety/cmd_vel_override` | 0.6s |
| 1 | 反向/手动 | `/reverse/cmd_vel` | 0.6s |
| 2 | 遥控 | `/teleop/cmd_vel` | 0.5s |
| 3 | Nav2/直航 | `/nav/cmd_vel` | 0.6s |
| 4 | VLA 单步 | `/cmd_vel_raw` | 0.6s |

- 同一时刻只有最高优先级活跃源生效；
- 所有源超时 → 零速度；
- 急停状态直接零速度，源名 `emergency`；
- 输出限幅（`max_linear/max_angular`），防御上游绕过 SafetyChecker；
- 发布 `/center/velocity_status` 便于调试。

### 5.11 安全（`safety_manager_node`）

| 检查 | 触发 | 处理 |
|---|---|---|
| 前方/后方/侧向障碍 | 最近距离 < `obstacle_stop_distance` | 急停，持续覆盖零速度 |
| 传感器超时 | `/center/synced_data` 或 `/scan` 超时 | `data_ok=false`，拒绝运动 |
| 低电量 | `battery_percent < battery_low_percent` | 急停 |
| 定位丢失 | 导航/搜索阶段 `localization_ok=false` 超过 grace | 急停 |
| 手动急停 | `/center/emergency_stop` | 急停/解除 |

- 障碍恢复有滞回：`obstacle_clear_distance > obstacle_stop_distance`；
- 急停覆盖发布到 `/safety/cmd_vel_override`，由 mux 以最高优先级执行；
- `SafetyState` 带给 `task_manager`/`action_generator`/`velocity_mux` 门控。

### 5.12 反向操作（`reverse_operation_node`）

| operation_type | 行为 |
|---|---|
| 0 EMERGENCY_STOP | 零速度 + 机械臂保持 |
| 1 REVERSE_MOVE | 按距离/速度回退，定时停止 |
| 2 ARM_RESET | 发布回 home 的 JointTrajectory |
| 3 TASK_CANCEL | 零速度 + 任务取消 |
| 4 MANUAL_OVERRIDE | 切换手动，暂停自动 |

- 优先级高于 Nav2/VLA；
- 回退期间按 10Hz 重复发布，覆盖普通速度；
- `task_manager` 收到反向操作会取消当前 mission 的 sub-action。

### 5.13 底盘桥（`edge_server_node` + STM32 ASCII）

Pi 上的 `edge_server_node` 直接经 UART 连接 STM32 USART3（115200 8N1）。主机侧逻辑在 `edge_server/edge_control.py`（纯逻辑，可单测）：

- 按钮/摇杆消息映射与 `software/api.yaml` 完全一致（映射表见 §8.2）；
- 网页手动命令按下后保持，`edge_server_node` 每 `motion_keepalive_ms`（100–600ms）重发；摇杆只重发方向字节，避免覆盖速度/微调；`edge.motion.resend_manual=true` 可恢复旧的周期性重发；
- ROS `/cmd_vel` 量化为方向+PWM（`body_drive.twist_to_body_command`），保留第二字节并带 1s 租约；调用方停止续租后自动发 `0`；
- 网页手动控制优先于 ROS `/cmd_vel`，网页松开/停止后导航恢复；
- 急停/安全状态、串口断开、遥测超时都会回零；
- 遥测经麦轮正运动学积分发布 `/odom`、`/joint_states`、`/base/status`。

MJPEG 在 8080 端口，订阅 ROS `/camera/color/image_raw`，由 `cv_bridge` 转 JPEG；相机未就绪返回 503。80 端口 `/stream` 返回 302 到 8080。协议细节见 §8，HTTP 路由见 §7.5。

### 5.14 健康检查（`health_monitor_node` + `health_monitor.py`）

目标：给同机 Edge Server/dashboard 一个只读接口，回答“ROS/设备是否健康、能否继续接任务”。

```text
/scan /camera /imu /audio /joint_states /odom /base/status /battery_state
        │
        ▼
health_monitor_node
  ├─ 设备注册表（health.devices）→ 订阅/超时/频率/指标校验
  ├─ ROS graph：get_node_names() + count_publishers()
  ├─ /center/safety_state（TRANSIENT_LOCAL）→ 急停/安全超时
  ├─ /center/health（HealthStatus，1 Hz，TRANSIENT_LOCAL）
  ├─ /center/healthcheck（HealthCheck 服务）
  └─ HTTP :8090（/ros/health 等，stdlib ThreadingHTTPServer）
```

状态语义：

- 设备：`ok` / `degraded` / `error` / `unknown` / `disabled`；
- 整体：`ok` / `starting` / `degraded` / `error` / `disabled`；
- 判定优先级：必需设备 error > 期望节点缺失 > 安全状态超时 > 宽限期 > 设备 degraded > 可选设备 error > 急停 > `strict_unknown` > ok；
- `ready = (ok|degraded) && !emergency_stop`；
- 启动宽限期避免“节点刚启动还没来得及发布”被误报。

关键设计：

1. **纯逻辑与 ROS 解耦**：`core/health_monitor.py` 不 import rclpy，29 项单测覆盖状态机；
2. **配置驱动**：设备 topic/message_type/频率/超时/required 全部在 YAML，新增设备无需改代码；
3. **诚实报告 unknown**：机械臂当前无反馈话题报 `unknown`；音箱已发布 `/body/audio_status`，`online=false` 报 `degraded`，不伪造 `ok`；
4. **HTTP 线程安全**：节点按 1 Hz 预计算 JSON 快照，HTTP 线程只读快照，不调用 rclpy；
5. **失败降级**：HTTP 端口占用只记录 `http.bound=false`，ROS 话题/服务仍工作；单次快照异常不退出节点；
6. **默认安全**：代码和 `robot.yaml` 默认监听 `127.0.0.1`，由同机 Edge Server 代理；跨机监控才改 `0.0.0.0` 并启用 `api_key`；
7. **晚加入兼容**：健康话题/安全状态均用 TRANSIENT_LOCAL，后启动的 HMI 也能拿到最后一帧；
8. **参数可调**：`/ros/params` 暴露健康频率/宽限期/设备频率/超时/required，修改在运行时生效，重启回 YAML。

接口契约、JSON 字段、设备反馈协议和 Edge Server/dashboard 接入示例见[健康检查契约](../HEALTHCHECK.md)。

### 5.15 语音找东西场景（`voice_scenario_node` + `core/scenarios/`）

在能力层之上增加 L3 场景层，避免把“语音交互 + 搜索 + 播报”写进 `object_search_node` 或 `task_manager_node`。分层机制见 §2.3。

**纯逻辑：**

- `VoiceIntentResolver`：文本 → `find / cancel / ignore`；复用 `InstructionParser`，支持“拖鞋在哪”“帮我找一下拖鞋”“find my slippers”；
- `FindObjectScenario`：`idle → searching → succeeded/failed/canceled`；输入语音、搜索进度、搜索结果、急停；输出 `ScenarioEffect`（`speak` / `start_search` / `cancel_search`）；不 import rclpy，可单测；
- `InstructionParser` 增加自由 label 兜底：`找充电器` → `充电器`，便于接开放词表检测器；别名匹配优先于自由抽取。

**场景状态机：**

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

**输入事件：**

| 事件 | 来源 | 场景处理 |
|---|---|---|
| 语音文本 | `/edge/voice_command` | 解析 find/cancel/ignore |
| 搜索进度 | `/center/search_status` | 更新 detail；里程碑播报 |
| 搜索结果 | `SearchObject` Action Result | 成功/失败播报，终止状态 |
| 动作不可用 | ActionClient 等待超时 | 播报“寻找功能未就绪” |
| 急停 | `/center/safety_state` | 取消搜索 + 播报 |
| 新目标 | `/edge/voice_command` | 取消旧搜索，启动新搜索 |

**ROS 适配（`voice_scenario_node`）：**

| 方向 | 接口 | 说明 |
|---|---|---|
| 订阅 | `/edge/voice_command` | 网页/HTTP/外部 ASR 的文本 |
| 订阅 | `/center/search_status` | 搜索状态/进度，用于确认/接近里程碑播报 |
| 订阅 | `/center/safety_state` | 急停取消 |
| 订阅 | `/center/pose` | 计算找到目标后的距离播报 |
| 发布 | `/body/say` | TTS 播报 |
| 发布 | `/center/scenario_status` | JSON 状态，1 Hz + 变化即发，TRANSIENT_LOCAL |
| Action | `/center/search_object` | 调用搜索能力；支持 cancel |

**HTTP 与网页：**

- `POST /api/voice`：raw 文本 / form `text=...` / JSON `{"text": "..."}`；`edge_server_node` 发布 `/edge/voice_command`，并缓存 `/center/scenario_status` 供 `GET /api/voice`；
- 网页 `audio.js` 用 Web Speech API 识别；命中“找/在哪”走场景接口，命中“别找了/停止寻找”取消并同时停车；普通方向/按键指令仍走原 `/cmd`。

**配置：** `scenario.*`（voice/search/say/status topic、时长、standoff、播报开关/间隔）；`instruction.object_labels` 维护 canonical label 与中文别名；拖鞋 canonical label 为 `slipper`，需要 YOLO-World/自定义 YOLO 输出同名 label。

**失败与降级**见 §10.2。

---

## 6. 关键链路与数据流

### 6.1 按钮点击（网页 → STM32）

以按住“↑”为例，消息是 `button:up:down`：

| 步骤 | 位置 | 做什么 |
|---|---|---|
| 1 | `web/index.html` `pressButton()` → `sendRawCmd()` | `fetch('/cmd', {method:'POST', body:'button:up:down'})`，纯文本 |
| 2 | `edge_server/edge_web.py` `do_POST()` | 识别 `/cmd`，调 `app.control(text)` |
| 3 | `EdgeHttpApp.control()` | 长度校验，调 `runtime.handle_command(text)` |
| 4 | `ros_nodes/edge_server_node.py` `handle_command()` | 急停检查，`dispatch_control_message()` |
| 5 | `edge_server/edge_control.py` `dispatch_control_message()` | 拆 `button:key:event` |
| 6 | `edge_control.button_command()` | `up → "1"`；松开 `up → "0"` |
| 7 | `MotionState.apply_manual()` | 记录持续运动状态；松开时清状态并发 `"0"` |
| 8 | `EdgeServerNode._send_command()` | 0.15s 内相同命令去重；`force=True` 时跳过 |
| 9 | `SerialLink.write()` | `serial.write("1\n")`；端口未打开直接返回 False |
| 10 | STM32 `ps2_receiver.c` `USART3_IRQHandler()` | 收字节，遇 `\n`/`\r` 组帧，`send_to_queue_isr()` |
| 11 | `task_manager.c` `exe_task_from_queue()` | 从队列取命令 |
| 12 | `controller.c` `execute_command()` | **`LED = !LED`**，查命令表，调对应执行器 |

关键点：

- 命令是 **ASCII + 换行**，不是二进制帧；`SerialLink.write()` 负责补 `\n`；
- 网页按钮/摇杆**按下时只发一次**，松开/停止时再发一次；STM32 会锁存运动状态，按住期间不需要重发；
- 松开方向键发 `"0"`；`square/circle` 松开发 `"I"`（直行）；页面隐藏用 `sendBeacon('/cmd', 'button:stop:down')` 兜底；
- `handle_command()` 只在急停时返回空；**串口没打开也会返回命令**（HTTP 仍回 `ok`），所以“接口 200”不等于“STM32 收到了”；
- 每条命令、串口连接/断开、急停、租约超时都会写入节点内的 120 条环形缓冲（`_record_event`），dashboard 通过 `/api/events` 展示；服务重启后清空。

### 6.2 遥测 / 里程计（STM32 → ROS）

| 步骤 | 位置 | 说明 |
|---|---|---|
| 1 | `encoder_sensor.c` `encoder_telemetry_task()` | 每 50ms（20Hz）组一帧 |
| 2 | `encoder_uart_write()` | USART3 PB10 阻塞发送，115200 约 4~5ms |
| 3 | `SerialLink._loop()` | `readline()` 收行，UTF-8 解码 |
| 4 | `_on_serial_line()` | `parse_body_line()` 校验 |
| 5 | `_update_odom()` | 四轮 mm/s → `WheelOdometry` |
| 6 | `on_odom_timer()` / `on_status_timer()` | 发布 `/odom`、`/joint_states`、`/base/status` |

当前固件每帧 9 段：

```text
$B,<seq>,<uptime_ms>,<fl_mm_s>,<fr_mm_s>,<rl_mm_s>,<rr_mm_s>,<distance_mm>,<cliff_mask>\n
```

- 兼容旧 8 段帧（无 `cliff_mask`，`cliff_available=false`）；
- 解析失败（段数不对、数字非法、速度越界 ±5000、悬崖掩码越界）整帧丢弃；`/status` 里 `encoder_ok=true` 表示最近 2 秒收到过合法帧。

里程计几何在 `edge_server/core/odometry.py`，参数来自 `robot.yaml`：

- `wheel_radius`、`wheelbase_x`、`track_y`、`encoder_cpr`；
- `sign_fl/fr/rl/rr` 修正轮子安装方向；
- `publish_tf` 默认 false（由 EKF 发 `odom→base_link`）。

### 6.3 VLA 单步

```text
edge/Pi sensors → bridge_node → /center/synced_data
    → vla_inference_node → /center/action_intent
    → action_generator_node → /cmd_vel_raw
    → velocity_mux_node → /cmd_vel → edge_server_node → STM32
```

### 6.4 建图

```text
/scan + /odom + /imu/data → slam_toolbox → /map
    → map_manager_node → /center/map_info + /center/frontiers
    → task_manager (map) → object_search (explore) → navigation_node → /nav/cmd_vel
    → mux → /cmd_vel → edge_server_node → STM32
```

### 6.5 目标搜索

```text
/camera/color/image_raw + /camera/depth/image_raw + /scan + /center/pose
    → target_detector_node → /center/target_detections
    → object_search_node (SearchCoordinator)
        ├─ /center/frontiers + /center/map_info → 探索目标
        ├─ /center/navigate_to → navigation_node → /nav/cmd_vel
        └─ 确认目标 → 接近 → SearchObject.Result(target_pose)
    → task_manager → ExecuteTask/ExecuteMission.Result
```

### 6.6 急停

```text
急停按钮/服务/障碍/低电量/定位丢失
    → safety_manager_node → /center/safety_state + /safety/cmd_vel_override
    → velocity_mux_node → /cmd_vel=0
    → task_manager/action_generator 门控
```

安全链路始终独立存在：

```text
safety_manager_node → /safety/cmd_vel_override → velocity_mux_node → /cmd_vel
```

场景层不能绕过它；急停时场景节点也会收到 `/center/safety_state` 并取消搜索。

### 6.7 语音找东西端到端

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

---

## 7. 接口设计

### 7.1 话题

**传感器与外部输入**

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/scan`、`/edge/radar` | `sensor_msgs/LaserScan` | 发布/订阅 | 激光；实车由 `lidar_node` 发布 |
| `/lidar/status` | `std_msgs/String` | 发布 | LD19/D6 串口、CRC、扫描状态（1 Hz） |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 发布/订阅 | Astra RGB / MJPEG / 目标检测 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 发布/订阅 | Astra 深度 / 目标测距 |
| `/edge/video` | `sensor_msgs/Image` | 发布/订阅 | 兼容旧图像入口 |
| `/edge/audio` | `custom_msgs/AudioData` | 发布/订阅 | HFP 录音时 10 Hz PCM（s16le/8k/mono） |
| `/odom`、`/robot/odometry` | `nav_msgs/Odometry` | 订阅 | 里程计 |
| `/imu/data` | `sensor_msgs/Imu` | 订阅 | IMU |
| `/joint_states` | `sensor_msgs/JointState` | 订阅 | 关节 |
| `/battery_state` | `sensor_msgs/BatteryState` | 订阅 | 电池 |
| `/map` | `nav_msgs/OccupancyGrid` | 订阅/发布 | 地图 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 订阅 | AMCL |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 发布 | 初始位姿 |

**内部状态与任务**

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/center/synced_data` | `custom_msgs/SyncedSensorData` | 发布 | 软同步 |
| `/center/robot_state` | `custom_msgs/RobotState` | 发布 | 状态 |
| `/center/action_intent` | `custom_msgs/ActionIntent` | 发布 | 意图 |
| `/center/inference_result` | `custom_msgs/InferenceResult` | 发布 | 推理结果 |
| `/center/status` | `custom_msgs/SystemStatus` | 发布 | 状态机 |
| `/center/mission_status` | `custom_msgs/MissionStatus` | 发布 | 长任务进度 |
| `/center/safety_state` | `custom_msgs/SafetyState` | 发布 | 安全 |
| `/center/map_info` | `custom_msgs/MapInfo` | 发布 | 地图摘要 |
| `/center/frontiers` | `custom_msgs/FrontierList` | 发布 | 前沿 |
| `/center/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 发布 | map 位姿 |
| `/center/localization_status` | `custom_msgs/LocalizationStatus` | 发布 | 定位健康 |
| `/center/nav_status` | `custom_msgs/NavStatus` | 发布 | 导航状态 |
| `/center/target_detections` | `custom_msgs/TargetList` | 发布 | 目标 |
| `/center/search_status` | `custom_msgs/SearchStatus` | 发布 | 搜索状态 |
| `/center/velocity_status` | `custom_msgs/VelocityMuxStatus` | 发布 | 速度源 |
| `/center/health` | `custom_msgs/HealthStatus` | 发布 | 系统/设备健康（1 Hz，TRANSIENT_LOCAL） |
| `/task/request` | `custom_msgs/TaskRequest` | 发布/订阅 | 内部任务 |
| `/center/mission_request` | `custom_msgs/TaskRequest` | 订阅 | 外部任务 |

**底盘、交互与执行器**

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/base/status` | `std_msgs/String` | 发布/订阅 | STM32 遥测 JSON（底盘/健康，含 cliff） |
| `/body/cliff_status` | `std_msgs/String` | 发布 | 悬崖左右/触发状态 JSON（1 Hz） |
| `/arm/joint_states` | `sensor_msgs/JointState` | 订阅 | 机械臂回读（待接入） |
| `/body/say` | `std_msgs/String` | 订阅 | 文本 → TTS → 蓝牙音箱 |
| `/edge/voice_command` | `std_msgs/String` | 订阅 | 语音文本 → 场景编排层；HTTP `/api/voice` 与外部 ASR 共用 |
| `/center/scenario_status` | `std_msgs/String` | 发布 | 场景状态 JSON（1 Hz，TRANSIENT_LOCAL）；`GET /api/voice` 数据源 |
| `/body/audio_status` | `std_msgs/String` | 发布 | 音箱状态 JSON（1 Hz） |
| `/body/mic_status` | `std_msgs/String` | 发布 | HFP 麦克风状态 JSON（1 Hz） |
| `/body/power_status` | `std_msgs/String` | 发布 | 树莓派电源状态 JSON（1 Hz） |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 最终速度 |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | 发布 | VLA 原始 |
| `/nav/cmd_vel` | `geometry_msgs/Twist` | 发布 | 导航速度 |
| `/reverse/cmd_vel` | `geometry_msgs/Twist` | 发布 | 反向速度 |
| `/safety/cmd_vel_override` | `geometry_msgs/Twist` | 发布 | 安全覆盖 |
| `/teleop/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 遥控 |
| `/arm/trajectory` | `trajectory_msgs/JointTrajectory` | 发布 | 机械臂 |

### 7.2 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `/center/set_mode` | `custom_msgs/SetMode` | auto/manual/reverse |
| `/center/emergency_stop` | `custom_msgs/EmergencyStop` | 急停/解除 |
| `/center/map_control` | `custom_msgs/MapControl` | start/stop/save/load/reset |
| `/center/set_initial_pose` | `custom_msgs/SetInitialPose` | 设置初始位姿 |
| `/center/healthcheck` | `custom_msgs/HealthCheck` | 全部/指定设备健康快照 |

### 7.3 Action

| Action | Goal | Result |
|---|---|---|
| `/center/execute_task` | task_id, instruction, task_type, target_* | success, info, target_pose, approach_pose, confidence, elapsed |
| `/center/execute_mission` | mission_id, mission_type, target_*, max_duration, save_map, target_coverage | success, info, target_pose, approach_pose, confidence, elapsed |
| `/center/navigate_to` | target_pose, tolerance, timeout, goal_id, avoid_obstacles | success, info, final_pose, elapsed |
| `/center/explore` | mission_id, max_duration, target_coverage | success, info, coverage, elapsed |
| `/center/search_object` | task_id, target_label, min_confidence, max_duration, standoff, explore_if_not_found, save_map | found, info, target_pose, approach_pose, confidence, elapsed |

### 7.4 消息

`AudioData`、`SyncedSensorData`、`RobotState`、`TaskRequest`、`ActionIntent`、`ReverseOperation`、`InferenceResult`、`SafetyState`、`SystemStatus`、`MapInfo`、`Frontier`、`FrontierList`、`TargetDetection`、`TargetList`、`LocalizationStatus`、`NavStatus`、`SearchStatus`、`MissionStatus`、`VelocityMuxStatus`、`DeviceHealth`、`HealthStatus`。

### 7.5 HTTP / Web 层

80 端口（完整契约见 `software/api.yaml`）：

| 方法 | 路径 | 作用 |
|---|---|---|
| POST | `/cmd` | 纯文本 `button:...` / `stick:...` / `raw:...` |
| POST | `/cmd-button` | 表单 `key`/`type`（兼容旧页面） |
| POST | `/cmd-stick` | 表单 `stickId`/`direct`/`stepValue` |
| GET/POST | `/stop` | 无条件停车 |
| GET | `/status` | 运行状态（含 `motion` 当前指令） |
| GET | `/api/body` | 遥测详情 + `motion` 当前指令/来源/持续时长 |
| GET | `/api/events?limit=N` | 最近 N 条运行事件（命令/串口/急停/超时） |
| GET | `/api/speaker` | 蓝牙音箱/TTS/播放状态 |
| POST | `/api/speaker/{connect,say,play,volume,stop}` | 连接/TTS/播放文件/音量/停止 |
| GET | `/api/microphone` | HFP 麦克风/录音状态 |
| POST | `/api/microphone/{start,stop,play,volume,list}` | 录音/回放/音量/列表 |
| GET | `/api/microphone/recordings[/{name}]` | 录音列表/WAV 下载 |
| GET | `/api/power` | 树莓派欠压/降频/温度/核心电压状态 |
| POST | `/api/voice` | 语音文本 → `/edge/voice_command`（raw/form/JSON） |
| GET | `/api/voice` | 语音场景状态 + `/center/scenario_status` |
| GET | `/api/modules` | 模块注册表 |
| GET | `/api/modules/edge` | Edge 模块详情 |
| GET | `/api/modules/edge/params` | 可调参数（含 `speaker_volume`） |
| POST | `/api/modules/edge/params` | 改参数（keepalive/fps/speaker_volume） |
| GET | `/api/modules/ros/health` | 代理 8090 健康接口 |
| POST | `/api/center/handshake` | Center 启动/心跳握手，记录 IP、端口、`infer_url` |
| GET | `/api/center` | 最近一次 CENTER 握手快照（VLA 节点据此发现推理地址） |
| GET | `/stream` | 302 跳到 `:8080/stream` |
| GET | 其他 | `web/` 静态文件 |

前端文件：

| 文件 | 作用 |
|---|---|
| `web/index.html` | 手柄页；按钮/摇杆事件、`/cmd` 发送、stop beacon |
| `web/ps2.js` / `ps2.css` | 手柄视觉与交互 |
| `web/dashboard.html` / `dashboard.js` | 数字孪生：EDGE + ROS 2 合并节点、BOM 硬件矩阵、蓝牙音箱/语音播报/麦克风录音面板 |
| `web/audio.js` | 音频控制（与运动链路独立） |
| `web/yolo.js` | 视觉检测展示 |

注意：`/cmd` 只接受**纯文本**，不要发 JSON；页面改动后要 `colcon build` + `systemctl restart`，浏览器再 Ctrl-F5。

### 7.6 健康检查接口

对外主接口是 HTTP（供同机 Edge Server/dashboard），ROS 原生接口同时保留：

| 接口 | 类型 | 说明 |
|---|---|---|
| `GET /ros/health` | HTTP | 详细健康快照（别名 `/v1/health`、`/health`、`/healthcheck`） |
| `GET /ros/health/devices/{id}` | HTTP | 单设备健康 |
| `GET/POST /ros/params` | HTTP | 健康监控参数查看/运行时修改 |
| `GET /ros/healthz`、`GET /ros/readyz` | HTTP | 存活/就绪探针（readyz 503 表示未就绪） |
| `/center/health` | `custom_msgs/HealthStatus` | 1 Hz，TRANSIENT_LOCAL |
| `/center/healthcheck` | `custom_msgs/HealthCheck` | 全部/指定设备，支持 `include_metrics` |

整体状态：`ok` / `starting` / `degraded` / `error` / `disabled`；设备状态：`ok` / `degraded` / `error` / `unknown` / `disabled`。

代码与 `robot.yaml` 默认监听 `127.0.0.1:8090`，由同机 Edge Server 代理；跨机监控再改 `0.0.0.0` 并设置 `api_key`。完整字段、设备清单、反馈协议和接入示例见[健康检查契约](../HEALTHCHECK.md)。

---

## 8. 底盘协议与 STM32 固件

### 8.1 ASCII 协议

当前 `body_control` 固件使用 **ASCII 命令 + 行遥测**，Pi 经 UART 连接 STM32 USART3（115200 8N1）。

**接线：** Pi GPIO4 → STM32 PB11，Pi GPIO5 → STM32 PB10，**必须共地**。Pi 侧设备是 `/dev/ttyAMA1`（`dtoverlay=uart3`），不是 `/dev/ttyUSB0`。

| 方向 | 格式 | 说明 |
|---|---|---|
| Pi→STM32 | `1`..`9` / `A`..`C` / `D`..`J` | 方向/转向/变速/微调/测试/复位，`\n` 结束 |
| Pi→STM32 | 两位命令，如 `15` / `95` | 方向/转向 + PWM 档位 1~9 |
| STM32→Pi | `$B,<seq>,<uptime_ms>,<fl>,<fr>,<rl>,<rr>,<distance_mm>,<cliff_mask>` | 四轮 mm/s、总里程 mm、悬崖掩码，约 10Hz |

命令必须以 `\n` 或 `\r\n` 结尾，否则会一直留在接收缓冲里。

### 8.2 网页/摇杆映射与命令字

| 命令 | 网页/摇杆入口 | 含义 | 松开 | 模块 |
|---|---|---|---|---|
| `0` | `stop` / 松开方向 | 停 | — | MOTOR_DIRECT |
| `1`~`4` | `up` / `down` / `left` / `right` | 前/后/左/右 | `0` | MOTOR_DIRECT |
| `5`~`8` | 左摇杆斜向 | 四斜向 | `0` | MOTOR_DIRECT |
| `9` / `A` | `l1` / `r1` | 左/右原地转 | `0` | MOTOR_DIRECT |
| `B` / `C` | `l2` / `r2` | 左/右大转弯 | `0` | MOTOR_DIRECT |
| `D` | `start` | 测试序列 | 不发 | MOTOR_DIRECT |
| `E` / `F` | `triangle` / `cross` | 加速 / 减速 | 不发 | MOTOR_PWM |
| `G` / `H` | `square` / `circle` | 左/右微调 | `I` | MOTOR_PWM |
| `I` | 微调松开 | 直行（微调回正） | — | MOTOR_PWM |
| `J` | `select` | 复位（清队列、停电机） | 不发 | RESET |
| `K`~`P` | — | 音频（当前固件未启用） | — | AUDIO |

- 摇杆示例：`stick:left-analog:uple:5` → `"55"`（方向 5 + PWM 5 档）；
- 第二位可选 `0`~`9` 表示 PWM 档位，例如 `15` = 前进 5 档。

**Twist 量化**（`core/body_drive.py`）：

- 平移按方向分 8 个扇区（1/5/3/7/2/8/4/6），按速度大小取 1~9 档；
- 原地转向 `wz>0 → 9`，`wz<0 → A`；
- 同时有平移和转向：默认 `priority` 取归一化后更大的一项；`time_share` 按 `arc_period_sec` 交替（近似弧线，抖动更大）。

### 8.3 固件结构

**启动顺序**（`main.c` → `controller.c`）：

```text
clock_init / delay_init
init_protocols()       → LED(PC13) 初始化
init_modules()
  ├─ audio_receiver.init()       USART2（JQ6500，音频已停用）
  ├─ ps2_receiver.init()         USART3 115200 + RXNE 中断   ← Pi 链路
  ├─ motor_direct_executor.init()
  ├─ init_freertos()             命令队列 + 消费任务
  ├─ motor_pwm_executor.init()   TIM3 PWM
  ├─ encoder_sensor_init()       PA0~PA7 EXTI + 遥测任务
  └─ cliff_sensor_init()         PB3/PB4，关闭 JTAG 保留 SWD
LED = !LED
vTaskStartScheduler()
```

**任务/中断：**

| 名称 | 位置 | 优先级 | 作用 |
|---|---|---|---|
| `USART3_IRQHandler` | `ps2_receiver.c` | 3 | 收 Pi 命令，按 `\n` 分帧入队 |
| `CONSUMER_TASK` | `task_manager.c` | 2 | 出队并 `execute_command()` |
| `ENC_TLM` | `encoder_sensor.c` | 1 | 50ms 发 `$B` 遥测 |
| 悬崖任务 | `cliff_sensor.c` | — | 10ms 采样，触发即封运动 |

**LED 的含义：** `execute_command()` 第一行就是 `LED = !LED`（PC13）。

- 每成功出队一条命令，LED 翻转一次；
- 上电初始化后 LED 先亮再灭，之后只由命令驱动；
- **LED 不闪 = 命令没到 `execute_command()`**，问题在 USART3 接收/队列，不在电机；
- 网页“按一下”会发两条命令（按下 `1`、松开 `0`），LED 翻转两次；按住期间不再重发，所以不会连闪；
- 固件里留了三个可直接用 ST-Link watch 的计数器（`ps2_receiver.c`）：

| 变量 | 含义 |
|---|---|
| `g_ps2_rx_bytes` | 收到的字节数；不涨 = 接线/波特率/引脚复用问题 |
| `g_ps2_rx_frames` | 完整帧数；bytes 涨而它不涨 = 没收到换行 |
| `g_ps2_rx_last_byte` | 最后一个字节，正常应是 `0x0A`/`0x0D` 或命令字符 |

> 从浏览器到 LED 的分层排查步骤与最小验证命令见[部署与运维手册](../deploy/DEPLOY.md)。

### 8.4 旧二进制协议（legacy）

`base_protocol.py` / `base_bridge_node.py` 保留 v1 二进制帧/CRC 实现，用于未来 STM32 固件升级；当前 ASCII 固件不支持，`base_transport` 默认 `edge_serial`，不要启用 `serial_binary`。

---

## 9. 配置系统

### 9.1 加载优先级

```text
ROS 参数 config_file  >  环境变量 EDGE_SERVER_CONFIG（兼容旧 CENTER_SERVER_CONFIG）
                      >  install/.../config/config.yaml
```

所有节点共享 `edge_server:` 顶层命名空间；日志用 `rclpy`/stdlib logging，不依赖 loguru。

本机 deployment：

```text
/etc/systemd/system/edge_server.service
  Environment=EDGE_SERVER_CONFIG=/opt/edge_server/config/deploy.yaml
  ExecStart=bash -lc '... ros2 launch /opt/edge_server/deploy/edge_server.launch.py'
```

### 9.2 配置文件

| 文件/位置 | 用途 |
|---|---|
| `config/config.yaml` | 仿真/桌面默认；`/edge/*` 话题、`perception.enabled=false`、`base.enabled=false` |
| `config/robot.yaml` | 实车覆盖；`/scan`、`/camera/color/image_raw`、`/camera/depth/image_raw`、Edge Web 80/8080 |
| `config/ekf.yaml` | robot_localization |
| `config/slam_toolbox.yaml` | slam_toolbox |
| `config/nav2_params.yaml` | Nav2 Humble 起步配置 |
| `/opt/edge_server/config/deploy.yaml` | 本机运行配置，远程专用，**不随 rsync 更新** |

| 配置项 | 本机值/说明 |
|---|---|
| 串口 | `base.port` / `base.baud` = `/dev/ttyAMA1` / `115200` |
| 手动控制重发 | `edge.motion.resend_manual` = `false`（按下只发一次；`true`=旧行为） |
| 手动/raw 刷新周期 | `edge.motion.keepalive_ms` = `200` |
| Web / MJPEG | `edge.web` = `0.0.0.0:80` / `8080` |
| 遥测状态文件 | `edge.state_file` = `/var/lib/edge_server/edge_params.json` |
| 健康网关 | `health.http` = `127.0.0.1:8090` |

改 `deploy.yaml` 后必须 `systemctl restart edge_server`；改仓库 `robot.yaml` 不会自动影响这台机器。

### 9.3 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `vla_engine.inference_rate` | 10 | 推理频率 |
| `vla_engine.confidence_threshold` | 0.6 | 低置信度确认 |
| `vla_engine.backend` | auto | auto/remote/stub；Center 不可用时降级 |
| `vla_engine.remote.discovery_url` | http://127.0.0.1/api/center | Edge 注册表（实车 80；仿真 8081） |
| `vla_engine.remote.timeout_sec` / `max_rate_hz` | 1.5 / 5.0 | 远程推理超时与频率上限 |
| `vla_engine.remote.fallback_to_rules` | true | 失败是否回退规则桩 |
| `safety.obstacle_stop_distance` | 0.5/0.35 | 障碍急停距离 |
| `safety.data_timeout_sec` | 2.0 | 传感器超时 |
| `safety.battery_low_percent` | 15 | 低电量 |
| `localization.min_quality` | 0.35 | 定位健康门限 |
| `navigation.min_localization_quality` | 0.35 | 导航 goal 的定位门限 |
| `map_manager.frontier_min_size` | 5 | 前沿最小尺寸 |
| `navigation.backend` | auto | nav2/direct/auto |
| `navigation.stuck_timeout_sec` | 6.0 | 停滞判定 |
| `search.standoff_distance` | 0.6 | 目标前停距 |
| `search.confirm_hits` | 3 | 目标确认次数 |
| `velocity_mux.priorities` | safety>reverse>teleop>nav>vla | 仲裁 |
| `task_manager.execution_hold_sec` | 1.0 | VLA 成功保持窗口 |
| `health.publish_rate` | 1.0 | 健康话题/HTTP 快照频率 |
| `health.startup_grace_sec` | 5.0/8.0 | 启动宽限期（仿真/实车） |
| `health.min_rate_ratio` | 0.5 | 低于期望频率比例判 degraded |
| `health.strict_unknown` | false | 可选设备 unknown 是否降级整体 |
| `health.http.host/port` | 127.0.0.1/8090 | HTTP 网关绑定（Edge Server 同机代理） |
| `base.transport/port/baud` | edge_serial//dev/ttyUSB0/115200 | Pi 直连 STM32（实车覆盖为 ttyAMA1） |
| `edge.web.port/stream_port` | 80/8080 | Edge Web 与 MJPEG |
| `edge.motion.keepalive_ms` | 200 | 手动/raw 命令重发周期 |
| `health.devices.<id>.required` | 按设备 | 缺失是否 error |

---

## 10. 异常处理与降级

### 10.1 运行期异常

| 异常 | 检测 | 处理 |
|---|---|---|
| 图像为空/损坏 | 预处理异常 | 停止推理，发布 error |
| 雷达缺失 | `data_ok=false` / scan 超时 | 停止输出/拒绝运动 |
| IMU 掉线/跳变 | `ImuMonitor` | 降低定位质量；不阻塞 AMCL |
| 磁力计受扰 | 模长偏差 >35% | `mag_ok=false`，不参与校正 |
| 定位质量低 | `LocalizationStatus.quality` | 拒绝 map 系目标；导航回零 |
| 目标点被占 | 栅格/代价地图 | 吸附最近自由点或拒绝 |
| 导航停滞 | 进度监控 | 取消、失败、重新规划 |
| 目标丢失 | 跟踪超时 | 回探索，清空确认 |
| 前沿不可达 | 导航失败 | 黑名单，换点 |
| 急停 | 服务/障碍/电量/定位 | 零速度覆盖，任务取消 |
| 地图保存失败 | 服务/IO | 返回失败，保留内存地图 |
| 串口断开 | 读写异常 | 2s 重连，底盘看门狗回零 |
| 设备掉线 | 健康检查超时/无发布者 | `HealthStatus` 报 `error/degraded`，Edge Server 禁止派发新任务 |
| 设备无反馈 | 可选设备从未收到数据 | 报 `unknown`，不伪造 `ok`；`strict_unknown` 可升级为 degraded |
| 期望节点缺失 | ROS graph 发现 | 宽限期后整体 `error`，`ros.missing_nodes` 给出名单 |
| 健康 HTTP 端口占用 | bind 异常 | ROS 话题/服务继续，`http.bound=false`，不影响运动控制 |
| Center 不可用 | discovery/infer 超时/报错 | `auto` 回退规则桩；`remote`+`fallback_to_rules=false` 发 stop；恢复后自动重连 |

### 10.2 能力降级

| 情况 | 行为 |
|---|---|
| 没有地图 | `search_object.explore_if_not_found=true` → 自动开始建图/探索 |
| 没装 Nav2 | `navigation.backend=auto` → 内置直航 |
| 检测器未配置/模型缺失 | 搜索超时 → 播报“没有找到拖鞋”，网页显示 failed |
| 搜索 Action 未启动 | 2 s 等待后播报“寻找功能还没准备好”，不进入 searching |
| TTS/音箱不可用 | 仍发布状态；网页显示结果，日志记录失败 |
| 急停（语音场景） | 立即取消搜索并播报“已急停，停止寻找” |
| 浏览器不支持 Web Speech | 用 `/api/voice` 文本入口或外部 ASR 发布 topic |
| 浏览器语音需要联网 | 可改用外部离线 ASR（Vosk/Whisper）发布 `/edge/voice_command`，场景层无需改动 |
| 找不到 label | 使用支持该 label 的检测器（YOLO 自定义模型/YOLO-World），并保持 `instruction.object_labels` 的 canonical label 与检测输出一致 |

---

## 11. 实时性、安全与性能

- **速度闭环在车上**：`velocity_mux` + `edge_server_node` + STM32 看门狗；远程 edge_server 断线时底盘自动回零。
- **急停优先级最高**：安全覆盖 > 反向 > 遥控 > 导航 > VLA。
- **超时**：每个速度源 0.5–0.6s 超时；STM32 自 0.5s 看门狗。
- **QoS**：状态用 TRANSIENT_LOCAL；传感器用 best effort；地图发布 TRANSIENT_LOCAL。
- **不要在 Wi-Fi 上跑底盘闭环**：Nav2 建议在 Pi/Orin 上运行，edge_server 远程只做高层任务。
- **硬件急停**：软件之外必须有常闭急停切断电机驱动。

| 指标 | 目标 |
|---|---|
| VLA 推理 | 5–20 Hz |
| 端到端控制延迟 | < 200 ms（车上闭环） |
| 激光 | ≥ 10 Hz，360° |
| IMU | 100 Hz |
| 编码器/里程计 | 30–50 Hz |
| 建图 | 室内 30 m²，5–10 分钟 |
| 导航到点 | 1–5 m，误差 < 0.2 m |
| 目标检测 | 5–10 Hz（颜色/YOLO-lite） |
| 目标搜索 | 30–300 s（场景相关） |
| 急停响应 | < 100 ms（软件）/ 硬件更快 |
| 健康快照 | 1 Hz；HTTP 响应 < 10 ms（本地） |
| 设备故障发现 | ≤ timeout_sec（典型 0.5–3 s） |

---

## 12. 启动顺序（摘要）

实车按“电源 → 传感器 → Edge Server → 定位/建图 → 导航 → 健康检查”顺序启动，完整命令与验证见[部署与运维手册](../deploy/DEPLOY.md)。

| 顺序 | 启动项 | 确认 |
|---|---|---|
| 1 | 上电，确认 12V/5V/6V、共地、急停释放 | 电压正常，急停未按下 |
| 2 | 激光驱动（`lidar_node` 或厂商 launch） | `/scan` 10Hz |
| 3 | Astra 深度相机 | `/camera/color/image_raw`、`/camera/depth/image_raw` |
| 4 | `robot_bringup.launch.py`（含 `edge_server_node`） | `http://<pi-ip>/`、8080 有图、`/odom`、`/base/status` |
| 5 | EKF | `odom→base_link` |
| 6 | 建图（`slam_stack` + `execute_mission` map） | `/map`、`/center/frontiers`、保存成功 |
| 7 | 定位/导航（`nav2_stack` + AMCL） | `/center/pose`、`/center/navigate_to` |
| 8 | 导航/搜索 Action | 结果成功，误差 < 0.2 m |
| 9 | 健康检查 | `/ros/health`、`/ros/readyz` |

仿真/桌面：

```bash
ros2 launch edge_server edge_server.launch.py
python3 tests/fake_edge_publisher.py
python3 tests/end_to_end_check.py       # 7/7
```

SLAM/导航/搜索联调：

```bash
ros2 launch edge_server edge_server.launch.py \
  config_file:=<repo>/software/edge_server/tests/config_test.yaml \
  start_perception:=false start_localization:=false
python3 tests/fake_slam_robot.py
python3 tests/slam_nav_check.py          # 6/6
```

> 注意：`fake_slam_robot` 自己发布 `/center/localization_status`，因此联调时关闭 `localization_node`，避免两个源覆盖质量值。

---

## 13. 测试策略与结果

### 13.1 单元测试

> WSL/容器/沙箱中若 DDS 发现不稳定，测试和节点都加：
> `ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。

```bash
cd software/edge_server
PYTHONPATH=.:$PYTHONPATH python3 -m pytest tests -q
# 179 passed
```

健康检查专项：

```bash
PYTHONPATH=.:$PYTHONPATH python3 -m pytest \
  tests/test_health_monitor.py tests/test_health_http.py -q
# 29 passed
```

覆盖：

- 状态机、安全校验、动作规划、预处理、VLA、反向操作（v1）；
- 指令解析、栅格/前沿/存图、相机/激光定位、跟踪、搜索协调器、导航校验/直航、IMU 监控/航向、速度仲裁、麦轮里程计、串口协议 CRC/分帧、ICM-20948 驱动（v2）；
- 远程 VLA 地址发现/握手、请求构造、`auto` 回退、`remote` 失败停车、degraded 窗口（v3）；
- 语音场景解析/状态机/播报、`/api/voice` 请求体、音箱/麦克风/电源监控、雷达解析、底盘映射（v3）；
- 设备健康状态机、启动宽限期、频率/超时/非法数据、节点缺失、急停、设备过滤、HTTP 存活/就绪/鉴权/单设备/metrics 开关（healthcheck）。

### 13.2 VLA/安全端到端

```bash
python3 tests/end_to_end_check.py
# RESULT: 7/7 passed
```

覆盖：前进、左转、低置信度、手动模式、急停、障碍物、反向回退。

### 13.3 SLAM/导航/搜索联调

```bash
python3 tests/slam_nav_check.py
# RESULT: 6/6 passed
```

覆盖：

1. `NavigateTo` 到点；
2. `Explore` 覆盖率；
3. `SearchObject` 找到 cup；
4. `MapControl` 保存/加载；
5. `ExecuteTask "找杯子"` 自动转长任务；
6. `ExecuteMission map` 建图+保存。

`fake_slam_robot.py` 模拟地图、激光、位姿、定位状态和目标检测，不依赖真实硬件。

### 13.4 语音找东西联调

```bash
python3 tests/voice_scenario_check.py
# RESULT: 3/3 passed
```

覆盖：

1. 语音“找杯子” → `SearchObject` → `/body/say` “找到杯子了”；
2. 语音“别找了” → 取消 Action → `/body/say` “好的，停止寻找”；
3. `/center/scenario_status` 的 `searching/succeeded/canceled` 状态链。

前置：`fake_slam_robot.py` + `edge_server.launch.py`（见 §12）。`tests/test_voice_scenario.py` 另有 9 项纯逻辑单测，覆盖解析、状态机、播报文案；`/api/voice` 的 raw/form/JSON 请求体在 `test_edge_web.py` 覆盖。

---

## 14. 当前限制与后续工作

### 14.1 当前限制

1. **VLA 依赖 Center Server**：仓库不带权重；`backend: auto` 由 Center 提供真实 SmolVLA，离线自动回退规则桩；`remote` + `fallback_to_rules: false` 时失败发 stop（需人工确认）。
2. **STM32 固件未升级**：主机侧二进制协议已实现，固件仍 ASCII；需按 §8 实现编码器/IMU 回传。
3. **Nav2 参数是起步值**：`robot_radius`、速度、膨胀半径、DWB critics 需实车调参。
4. **深度相机可选**：接入 Astra Pro 后优先深度测距，未接入/深度无效时回退激光/bbox 高度。
5. **ICM-20948 磁力计需实测校准**：硬铁/软铁/安装偏航；电机干扰下只慢速校正。
6. **单地图/单楼层**：未实现多地图切换/电梯/多层。
7. **动态目标**：搜索假设目标基本静止；移动目标跟踪需扩展预测/重规划。
8. **目标检测模型**：颜色检测只适合演示；YOLO 需模型/算力；可接 Orin Nano。
9. **地图保存回退**：slam_toolbox 不可用时 PGM/YAML 精度取决于当前 `/map`。
10. **未做硬件在环**：`fake_slam_robot` 验证了软件链路，真实串口/IMU/激光需实车联调。
11. **健康反馈缺口**：机械臂 `/arm/joint_states` 尚未发布，健康接口报 `unknown`；音箱已发布 `/body/audio_status`，接入新设备仍无需改 `health_monitor_node`。
12. **健康 HTTP 无 TLS/限流**：默认仅回环；跨机需 `api_key` + 防火墙/反向代理。

### 14.2 后续优先级

1. STM32 固件：4 路编码器 + 4 路 PWM + 轮速 PID + 二进制协议；
2. 实车 TF/标定：轮半径/CPR、IMU 安装、激光/相机外参；
3. Nav2 调参 + 真实地图闭环；
4. YOLO 接入；深度相机已接入 Astra Pro；
5. 多地图/动态目标/远程 HMI。
