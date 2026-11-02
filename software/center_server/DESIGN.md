# center_server 详细设计（基于 ROS2 的 VLA 推理中心）

## 1. 设计目标

`center_server` 是 Rush-MoG 系统中的**云端/中心侧智能推理节点**，主要承担以下职责：

- 接收 `edge_server` 上传的多模态感知数据
- 基于 ROS2 构建稳定的分布式通信与推理链路
- 提供 **VLA（Vision-Language-Action）推理能力**
- 将推理结果转换为小车运动控制指令与机械臂运动指令
- 支持对小车的反向操作，例如紧急停止、远程控制、任务撤销等
- 为上层任务调度、人机交互与日志追踪提供统一服务

---

## 2. 在总体架构中的位置

```text
┌─────────────────────────────────────────────────────┐
│                 上层任务编排 / HMI / 调度系统        │
└──────────────────────┬──────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│              center_server（VLA 推理中心）           │
│  ┌─────────────┐  ┌─────────────┐  ┌────────────┐ │
│  │ ROS2 通信层  │  │ VLA 推理引擎│  │ 指令生成层 │ │
│  └─────────────┘  └─────────────┘  └────────────┘ │
└──────────┬──────────────┬──────────────┬──────────┘
           │              │              │
    ┌──────┴──────┐  ┌─────┴─────┐  ┌─────┴──────┐
    │ edge_server │  │ 模型服务   │  │ 小车底盘    │
    │ 雷达/视频/语音│  │           │  │ 机械臂      │
    └─────────────┘  └───────────┘  └─────────────┘
```

`center_server` 位于 `edge_server` 与执行器之间，是系统的**决策中心**。

---

## 3. 核心能力边界

### 3.1 输入能力

`center_server` 接收来自 `edge_server` 的数据，主要包括：

| 数据类型 | 来源 | ROS2 建议接口 | 说明 |
|---|---|---|---|
| 雷达数据 | edge_server | `LaserScan` / `PointCloud2` | 用于环境感知、障碍物检测 |
| 视频图像 | edge_server | `Image` / `CompressedImage` | 用于视觉理解、VLA 推理 |
| 语音数据 | edge_server | `AudioData` / 自定义语音消息 | 用于语音指令识别 |
| 机器人状态 | edge_server | `Odometry`、`JointState`、自定义状态消息 | 用于决策反馈 |
| 任务请求 | 上层 / HMI | 自定义 `TaskRequest` | 用于发起任务 |

### 3.2 输出能力

`center_server` 输出以下控制指令：

| 输出类型 | 目标对象 | ROS2 建议接口 | 说明 |
|---|---|---|---|
| 小车移动指令 | 小车底盘 | `Twist` / `AckermannDrive` | 控制线速度、角速度 |
| 机械臂移动指令 | 机械臂 | `JointTrajectory` / `PoseArray` | 控制关节运动或末端位姿 |
| 反向操作指令 | 小车 / 机械臂 | 自定义 `ReverseOperation` | 急停、回退、撤销动作 |
| 推理结果反馈 | 上层 / edge_server | 自定义 `InferenceResult` | 返回动作意图与执行状态 |

---

## 4. 总体模块划分

```text
center_server/
├── ros2_communication/       # ROS2 通信适配层
├── data_preprocessing/        # 多模态数据预处理
├── vla_engine/               # VLA 推理引擎
├── action_planner/            # 动作规划与指令生成
├── reverse_operation/        # 反向操作模块
├── safety_manager/            # 安全管理
├── task_state_machine/        # 任务状态机
├── logger_monitor/            # 日志与监控
└── config/                    # 配置文件
```

---

## 5. 模块详细设计

## 5.1 ROS2 通信层

### 职责

负责 `center_server` 与 `edge_server`、底盘、机械臂、上层系统之间的 ROS2 通信。

### 设计内容

#### Subscriber

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/edge/radar` | `sensor_msgs/LaserScan` | 雷达扫描数据 |
| `/edge/video` | `sensor_msgs/Image` | 视频帧数据 |
| `/edge/audio` | `audio_common_msgs/AudioData` | 语音数据 |
| `/robot/odometry` | `nav_msgs/Odometry` | 小车里程计 |
| `/robot/joint_states` | `sensor_msgs/JointState` | 机械臂关节状态 |
| `/task/request` | `custom_msgs/TaskRequest` | 任务请求 |

#### Publisher

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 小车速度控制 |
| `/arm/trajectory` | `trajectory_msgs/JointTrajectory` | 机械臂轨迹 |
| `/reverse/operation` | `custom_msgs/ReverseOperation` | 反向操作指令 |
| `/center/inference_result` | `custom_msgs/InferenceResult` | 推理结果 |

#### Service / Action

| 类型 | 名称 | 说明 |
|---|---|---|
| Service | `/center/set_mode` | 设置工作模式：自动/手动/反向 |
| Service | `/center/emergency_stop` | 紧急停止服务 |
| Action | `/center/execute_task` | 执行任务的 Action 接口 |

---

## 5.2 多模态数据预处理模块

### 职责

将 `edge_server` 上传的原始数据转换为 VLA 模型可处理的输入格式。

### 处理流程

```text
原始数据 → 格式转换 → 时间同步 → 数据清洗 → 特征编码 → VLA 输入
```

### 处理内容

| 数据类型 | 处理方式 |
|---|---|
| 雷达 | 转换为极坐标栅格、目标距离、障碍物信息 |
| 视频 | 图像缩放、归一化、关键帧采样 |
| 语音 | 音频切片、特征提取、语音文本转换 |
| 状态 | 里程计、关节状态、任务状态统一编码 |

### 时间同步建议

由于雷达、视频、语音来自不同传感器，需要通过时间戳对齐。

建议策略：

- 使用 ROS2 `message_filters` 进行同步
- 以图像帧为主参考，关联最近雷达帧与语音片段
- 对延迟过高的数据进行丢弃或补零处理

---

## 5.3 VLA 推理引擎

### 职责

`center_server` 的核心模块，负责将视觉、语言、语音和机器人状态映射为可执行动作。

### 输入

| 输入 | 说明 |
|---|---|
| 图像 | 当前环境视觉信息 |
| 雷达信息 | 障碍物与空间约束 |
| 语音 / 文本指令 | 用户任务意图 |
| 机器人状态 | 小车位姿、机械臂关节状态 |

### 输出

| 输出 | 说明 |
|---|---|
| 小车动作意图 | 前进、后退、转向、停止等 |
| 机械臂动作意图 | 移动到某位置、抓取、放置等 |
| 动作置信度 | 推理结果的置信评分 |
| 任务完成状态 | 成功、失败、进行中、需人工确认 |

### 推理流程

```text
多模态输入
    ↓
特征提取（视觉、语音、状态编码）
    ↓
跨模态融合
    ↓
VLA 策略推理
    ↓
动作候选生成
    ↓
安全约束检查
    ↓
动作指令输出
```

### VLA 推理接口建议

```python
class VLAEngine:
    def predict(self, image, radar, audio, state):
        """
        输入：
            image: 当前图像
            radar: 雷达信息
            audio: 语音特征或文本
            state: 机器人状态
        输出：
            action: 小车与机械臂动作
            confidence: 置信度
            status: 任务状态
        """
```

---

## 5.4 动作规划与指令生成模块

### 职责

将 VLA 输出的高层动作意图转换为底层可执行的 ROS2 控制指令。

### 小车控制指令

| 动作意图 | 输出 Twist | 说明 |
|---|---|---|
| 前进 | `linear.x > 0` | 小车向前移动 |
| 后退 | `linear.x < 0` | 小车向后移动 |
| 左转 / 右转 | `angular.z > 0 / < 0` | 差速转向 |
| 停止 | `linear.x = 0`, `angular.z = 0` | 紧急停止或任务结束 |

### 机械臂控制指令

| 动作意图 | 输出形式 | 说明 |
|---|---|---|
| 移动到指定位姿 | `JointTrajectory` | 控制关节轨迹 |
| 末端移动 | `PoseArray` | 控制末端执行器位置 |
| 抓取 / 释放 | 自定义 Gripper 消息 | 夹爪开合 |

### 指令生成策略

建议采用两级控制：

1. **高层动作层**：VLA 输出动作意图
2. **底层执行层**：转换为速度、轨迹或关节指令

```text
VLA 动作意图
    ↓
动作语义解析
    ↓
小车速度分解 / 机械臂轨迹生成
    ↓
安全校验
    ↓
发布 ROS2 控制指令
```

---

## 5.5 反向操作模块

### 职责

支持对小车的反向操作，确保系统具备撤销、回退和紧急干预能力。

### 反向操作类型

| 操作类型 | 触发条件 | 输出 |
|---|---|---|
| 紧急停止 | 危险场景、人工触发、异常检测 | `Twist = 0`，机械臂暂停 |
| 小车回退 | 用户指令、任务失败后回退 | 反向移动一段距离 |
| 机械臂复位 | 任务结束、异常恢复 | 回到预设姿态 |
| 任务撤销 | 用户取消、超时、冲突检测 | 停止当前任务，恢复空闲状态 |
| 手动接管 | 人工控制模式开启 | 暂停自动推理输出 |

### 反向操作消息建议

```msg
# custom_msgs/ReverseOperation.msg

int32 operation_type
# 0: EMERGENCY_STOP
# 1: REVERSE_MOVE
# 2: ARM_RESET
# 3: TASK_CANCEL
# 4: MANUAL_OVERRIDE

float64 distance    # 回退距离，可选
string reason       # 操作原因
```

### 反向操作优先级

反向操作模块的优先级应高于普通 VLA 推理结果。

```text
反向操作请求 > 安全约束 > VLA 推理结果
```

---

## 5.6 安全管理模块

### 职责

防止系统输出危险指令，保障小车、机械臂和周围环境安全。

### 安全检查项

| 检查项 | 说明 |
|---|---|
| 速度限制 | 限制小车最大线速度、角速度 |
| 关节限位 | 限制机械臂关节范围 |
| 障碍物约束 | 雷达检测到障碍物时减速或停止 |
| 异常数据处理 | 图像、语音、雷达异常时停止推理 |
| 紧急停止 | 支持外部触发与内部异常触发 |

### 安全策略

```text
生成动作指令
    ↓
安全约束检查
    ├─ 通过 → 发布指令
    └─ 不通过 → 停止 / 减速 / 进入保护状态
```

---

## 5.7 任务状态机

### 职责

管理 `center_server` 的任务执行状态，避免混乱的指令输出。

### 状态定义

| 状态 | 说明 |
|---|---|
| `IDLE` | 空闲，等待任务 |
| `RECEIVING_DATA` | 正在接收传感器数据 |
| `INFERENCING` | 正在执行 VLA 推理 |
| `EXECUTING` | 正在执行小车或机械臂指令 |
| `REVERSING` | 正在执行反向操作 |
| `PAUSED` | 暂停 |
| `ERROR` | 异常状态 |
| `TASK_SUCCESS` | 任务成功完成 |
| `TASK_FAILED` | 任务失败 |

### 状态转移建议

```text
IDLE → RECEIVING_DATA → INFERENCING → EXECUTING → TASK_SUCCESS
                              ↓
                      异常 / 用户取消
                              ↓
                         REVERSING → IDLE
```

---

## 6. ROS2 节点设计

## 6.1 节点划分

建议 `center_server` 采用多节点结构：

| 节点 | 职责 |
|---|---|
| `center_bridge_node` | 与 edge_server 通信，接收多模态数据 |
| `vla_inference_node` | 执行 VLA 推理 |
| `action_generator_node` | 生成小车与机械臂指令 |
| `reverse_operation_node` | 处理反向操作 |
| `safety_manager_node` | 安全校验与紧急停止 |
| `task_manager_node` | 任务状态管理 |

### 推荐运行方式

```bash
ros2 launch center_server center_server.launch.py
```

---

## 6.2 关键节点详细设计

### center_bridge_node

```text
输入：
  /edge/radar
  /edge/video
  /edge/audio
  /robot/odometry
  /robot/joint_states

输出：
  /center/synced_data
  /center/robot_state
```

功能：

- 接收边缘数据
- 进行时间同步
- 发布统一的中心侧数据话题

---

### vla_inference_node

```text
输入：
  /center/synced_data
  /center/robot_state
  /task/request

输出：
  /center/action_intent
  /center/inference_result
```

功能：

- 调用 VLA 推理引擎
- 输出动作意图
- 发布推理结果

---

### action_generator_node

```text
输入：
  /center/action_intent

输出：
  /cmd_vel
  /arm/trajectory
```

功能：

- 将动作意图转换为 `Twist`
- 将机械臂意图转换为 `JointTrajectory`
- 支持参数化速度与加速度配置

---

### reverse_operation_node

```text
输入：
  /reverse/operation
  /center/emergency_stop

输出：
  /cmd_vel（覆盖型停止指令）
  /arm/trajectory（复位或暂停）
  /center/status
```

功能：

- 处理紧急停止
- 处理小车回退
- 处理机械臂复位
- 处理任务撤销

---

## 7. 数据流设计

## 7.1 正常任务数据流

```text
edge_server
    ↓
雷达 / 视频 / 语音 → center_bridge_node
    ↓
时间同步与预处理
    ↓
vla_inference_node
    ↓
动作意图 action_intent
    ↓
action_generator_node
    ↓
小车指令 / 机械臂指令
    ↓
小车底盘 / 机械臂执行器
```

## 7.2 反向操作数据流

```text
用户 / 安全模块 / 异常检测
    ↓
reverse_operation_node
    ↓
安全校验
    ↓
停止 / 回退 / 复位 / 撤销
    ↓
覆盖普通控制指令
```

---

## 8. 控制指令格式设计

## 8.1 小车移动指令

基于 `geometry_msgs/Twist`：

```yaml
linear:
  x: 0.5   # 前进速度 m/s
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2   # 转向速度 rad/s
```

## 8.2 机械臂移动指令

基于 `trajectory_msgs/JointTrajectory`：

```yaml
joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
points:
  - positions: [0.0, 1.57, 0.0, 1.0, 0.5]
    velocities: [0.1, 0.1, 0.1, 0.1, 0.1]
    time_from_start: 2.0
```

## 8.3 反向操作消息

```yaml
operation_type: 0
distance: -0.5
reason: "obstacle_detected"
```

---

## 9. 配置设计

建议采用 ROS2 参数化配置。

### 9.1 VLA 推理配置

```yaml
center_server:
  vla_engine:
    model_name: "vla_policy_v1"
    inference_rate: 10
    confidence_threshold: 0.6
    device: "cuda"

  action_generator:
    max_linear_speed: 0.8
    max_angular_speed: 1.0
    acceleration_limit: 0.3

  reverse_operation:
    reverse_speed: 0.3
    reverse_distance: 0.5
    emergency_stop_on_high_risk: true

  safety:
    obstacle_stop_distance: 0.5
    max_twist_x: 1.0
    max_twist_z: 1.5
```

---

## 10. 接口设计建议

## 10.1 TaskRequest

```msg
# custom_msgs/TaskRequest.msg

string task_id
string language_instruction
string voice_instruction
int32 priority
bool enable_reverse
```

## 10.2 InferenceResult

```msg
# custom_msgs/InferenceResult.msg

string task_id
string action_type
float64 confidence
string status
string description
```

## 10.3 ActionIntent

```msg
# custom_msgs/ActionIntent.msg

string target
string intent
float64 value
bool reverse_request
```

---

## 11. 异常处理设计

| 异常类型 | 检测方式 | 处理策略 |
|---|---|---|
| 数据超时 | 传感器数据长时间未到达 | 停止输出，进入 IDLE |
| 图像异常 | 图像为空、全黑、损坏 | 停止推理，上报错误 |
| 语音异常 | 音频噪声过大或无效 | 忽略语音，保留视觉推理 |
| 雷达异常 | 雷达数据缺失 | 降低速度或停止 |
| 推理置信度低 | 模型输出置信度低于阈值 | 停止执行，请求确认 |
| 障碍物逼近 | 雷达检测距离小于阈值 | 减速、停止、反向操作 |
| 机械臂超限 | 关节超出限位 | 暂停并复位 |

---

## 12. 部署架构建议

```text
edge_server（小车端）
    │
    │ 雷达、视频、语音、状态数据
    ▼
center_server（服务器 / 高性能计算端）
    │
    │ 小车指令、机械臂指令、反向操作
    ▼
小车底盘 / 机械臂执行器
```

### 部署建议

- `center_server` 部署在性能较强的计算平台上
- `edge_server` 部署在小车端，负责传感器采集与本地转发
- ROS2 通信通过局域网或低延迟网络连接
- 对大流量视频数据建议使用压缩传输
- VLA 模型推理建议使用 GPU 加速

---

## 13. 性能要求

| 指标 | 建议要求 |
|---|---|
| 推理频率 | 5–20 Hz |
| 端到端延迟 | < 500 ms |
| 视频输入 | 224×224 或 256×256 |
| 语音采样 | 16 kHz 或 8 kHz |
| 雷达频率 | 10 Hz 以上 |
| 反向操作响应 | < 100 ms |

---

## 14. 模块交互时序

```text
edge_server → center_server: 上传雷达、视频、语音
center_server → center_server: 时间同步、预处理
center_server → vla_engine: 多模态推理请求
vla_engine → center_server: 返回动作意图
center_server → action_generator: 生成控制指令
action_generator → 小车/机械臂: 发布控制指令
reverse_operation_node → 小车/机械臂: 覆盖执行反向操作
```

---

## 15. 核心伪代码

```python
class CenterServer:
    def __init__(self):
        self.bridge = CenterBridgeNode()
        self.vla = VLAEngine()
        self.action_gen = ActionGenerator()
        self.reverse = ReverseOperationNode()
        self.safety = SafetyManager()
        self.state = TaskState.IDLE

    def run(self):
        while ros2_ok():
            sensor_data = self.bridge.get_synced_data()
            robot_state = self.bridge.get_robot_state()

            if self.reverse.has_pending_request():
                self.state = TaskState.REVERSING
                self.reverse.execute(robot_state)
                continue

            task_request = self.bridge.get_task_request()
            action_intent = self.vla.predict(
                image=sensor_data.image,
                radar=sensor_data.radar,
                audio=sensor_data.audio,
                state=robot_state
            )

            if not self.safety.check(action_intent):
                self.reverse.emergency_stop()
                continue

            cmd_vel = self.action_generator.generate_twist(action_intent)
            arm_traj = self.action_generator.generate_arm_trajectory(action_intent)

            self.bridge.publish_cmd_vel(cmd_vel)
            self.bridge.publish_arm_trajectory(arm_traj)
```

---

## 16. 后续扩展方向

| 扩展方向 | 说明 |
|---|---|
| 多车协同 | 支持多个小车任务分配与协同 |
| 远程人机交互 | 支持人工确认与远程接管 |
| 数字孪生 | 将虚拟仿真环境与真实小车同步 |
| 任务记忆 | 记录历史动作，支持长任务推理 |
| 模型热更新 | 在线更新 VLA 策略模型 |
| 日志回溯 | 记录传感器、推理与指令数据，便于调试 |

---

## 17. 总结

`center_server` 是 Rush-MoG 系统中基于 ROS2 的**VLA 推理中心**，它通过接收 `edge_server` 的雷达、视频、语音与机器人状态数据，完成多模态融合推理，并输出小车移动与机械臂移动指令。同时，`center_server` 还通过反向操作模块支持小车回退、紧急停止、任务撤销等功能，是连接感知、决策与执行的核心枢纽。
