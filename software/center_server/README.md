# center_server（ROS2 VLA 推理中心）

设计文档：[`DESIGN.md`](DESIGN.md)。

本包已按“可构建、可启动、可联调”的目标修复，并通过 ROS2 Humble 本地冒烟测试
（`fake_edge_publisher` 仿真数据 + 7 项端到端用例）。当前 VLA 为规则桩（未包含
模型权重），机械臂真实轨迹规划待模型接入。

## 1. 环境

- Ubuntu 22.04 + ROS2 Humble
- Python 3.10
- 依赖：`rclpy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`trajectory_msgs`、
  `std_msgs`、`cv_bridge`、`message_filters`、`python3-numpy`、`python3-opencv`、`python3-yaml`

> 原代码依赖 `audio_common_msgs`、`torch`、`librosa`、`pydantic`、`loguru`，这些包不在
> Humble 基础安装中，直接导致 `colcon build` / 节点启动失败。现已改为：
> - 自定义 `custom_msgs/AudioData`（带 `std_msgs/Header`，可参与 message_filters 同步）
> - numpy 实现图像/雷达/音频特征与规则 VLA 桩
> - rclpy / stdlib logging

## 2. 包结构

ROS2 不支持一个包嵌套另一个包，因此消息包 `custom_msgs` 已从
`software/center_server/custom_msgs` 移到同级目录 `software/custom_msgs`：

```text
software/
├── center_server/          # ament_python 包
│   ├── center_server/      # core / utils / config
│   ├── ros_nodes/          # 6 个 ROS2 节点
│   ├── launch/
│   ├── resource/
│   ├── tests/              # 单元测试 + 仿真发布器 + 端到端测试
│   ├── package.xml
│   └── setup.py
└── custom_msgs/            # ament_cmake 消息/服务/Action 包
```

## 3. 构建

```bash
cd /path/to/rush-mog
mkdir -p ~/rush_mog_ws/src
ln -sfn "$(pwd)/software/center_server" ~/rush_mog_ws/src/center_server
ln -sfn "$(pwd)/software/custom_msgs" ~/rush_mog_ws/src/custom_msgs

cd ~/rush_mog_ws
colcon build --packages-select custom_msgs center_server
source install/setup.bash
```

修改源码后重新执行 `colcon build --packages-select custom_msgs center_server`。

## 4. 启动

```bash
ros2 launch center_server center_server.launch.py
# 指定其他配置：
ros2 launch center_server center_server.launch.py config_file:=/path/to/config.yaml
```

启动 6 个节点：

| 节点 | 职责 |
|---|---|
| `center_bridge_node` | 订阅 5 路 edge 数据，近似时间同步后发布 `/center/synced_data`、`/center/robot_state` |
| `vla_inference_node` | 调用 VLA 引擎，发布 `/center/action_intent`、`/center/inference_result` |
| `action_generator_node` | 意图 -> `/cmd_vel`、`/arm/trajectory`，发布前做安全校验 |
| `reverse_operation_node` | `/reverse/operation` -> 急停/回退/机械臂复位/任务撤销/手动接管 |
| `safety_manager_node` | 障碍物/数据超时监控，`/center/safety_state`，提供 `/center/emergency_stop` |
| `task_manager_node` | 状态机、`/center/set_mode`、`/center/execute_task` Action、`/center/status` |

## 5. 本地联调

终端 A：

```bash
source ~/rush_mog_ws/install/setup.bash
ros2 launch center_server center_server.launch.py
```

终端 B（模拟 edge_server）：

```bash
source ~/rush_mog_ws/install/setup.bash
python3 ~/rush_mog_ws/src/center_server/tests/fake_edge_publisher.py
```

终端 C（一键端到端测试，覆盖任务/模式/急停/障碍物/反向操作）：

```bash
source ~/rush_mog_ws/install/setup.bash
python3 ~/rush_mog_ws/src/center_server/tests/end_to_end_check.py
# 期望输出：RESULT: 7/7 passed
```

手动验证常用命令：

```bash
# 前进 / 左转 / 停止（任务成功后会下发零速度）
ros2 action send_goal /center/execute_task custom_msgs/action/ExecuteTask \
  "{task_id: demo1, instruction: 前进}" --feedback
ros2 action send_goal /center/execute_task custom_msgs/action/ExecuteTask \
  "{task_id: demo2, instruction: 左转}" --feedback
ros2 action send_goal /center/execute_task custom_msgs/action/ExecuteTask \
  "{task_id: demo3, instruction: 停止}" --feedback

# 查看同步数据、动作意图、控制指令
ros2 topic echo /center/synced_data --once
ros2 topic echo /center/action_intent
ros2 topic echo /cmd_vel

# 回退 0.3 m（约 1 秒）
ros2 topic pub --once /reverse/operation custom_msgs/msg/ReverseOperation \
  "{operation_type: 1, distance: 0.3, reason: smoke_test}"

# 急停 / 解除
ros2 service call /center/emergency_stop custom_msgs/srv/EmergencyStop "{trigger: true}"
ros2 service call /center/emergency_stop custom_msgs/srv/EmergencyStop "{trigger: false}"

# 手动 / 自动模式
ros2 service call /center/set_mode custom_msgs/srv/SetMode "{mode: manual}"
ros2 service call /center/set_mode custom_msgs/srv/SetMode "{mode: auto}"

# 模拟正前方 0.2 m 障碍 -> 安全模块自动急停
ros2 topic pub --once /sim/obstacle_distance std_msgs/msg/Float32 "{data: 0.2}"
# 清除障碍（需大于 obstacle_clear_distance=0.7）
ros2 topic pub --once /sim/obstacle_distance std_msgs/msg/Float32 "{data: 3.0}"
```

## 6. 接口

### 话题

| 话题 | 类型 | 方向 |
|---|---|---|
| `/edge/radar` | `sensor_msgs/LaserScan` | 订阅 |
| `/edge/video` | `sensor_msgs/Image` | 订阅 |
| `/edge/audio` | `custom_msgs/AudioData` | 订阅 |
| `/robot/odometry` | `nav_msgs/Odometry` | 订阅 |
| `/robot/joint_states` | `sensor_msgs/JointState` | 订阅 |
| `/task/request` | `custom_msgs/TaskRequest` | 订阅/发布（Action 内部下发） |
| `/center/synced_data` | `custom_msgs/SyncedSensorData` | 发布 |
| `/center/robot_state` | `custom_msgs/RobotState` | 发布 |
| `/center/action_intent` | `custom_msgs/ActionIntent` | 发布 |
| `/center/inference_result` | `custom_msgs/InferenceResult` | 发布 |
| `/center/status` | `custom_msgs/SystemStatus` | 发布 |
| `/center/safety_state` | `custom_msgs/SafetyState` | 发布 |
| `/reverse/operation` | `custom_msgs/ReverseOperation` | 订阅 |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 |
| `/arm/trajectory` | `trajectory_msgs/JointTrajectory` | 发布 |

`ReverseOperation.operation_type`：`0` 急停、`1` 回退、`2` 机械臂复位、`3` 任务撤销、
`4` 手动接管（与 DESIGN.md 5.5 一致）。

### 服务 / Action

| 名称 | 类型 | 说明 |
|---|---|---|
| `/center/set_mode` | `custom_msgs/srv/SetMode` | `auto` / `manual` / `reverse` |
| `/center/emergency_stop` | `custom_msgs/srv/EmergencyStop` | 急停/解除（由 safety_manager 提供） |
| `/center/execute_task` | `custom_msgs/action/ExecuteTask` | 单步任务下发 |

## 7. 配置

默认配置安装到 `share/center_server/config/config.yaml`，与 DESIGN.md §9 一致使用
`center_server:` 顶层命名空间。主要参数：

- `vla_engine.inference_rate`：推理频率（默认 10 Hz）
- `vla_engine.confidence_threshold`：低置信度进入 `needs_confirmation`
- `safety.obstacle_stop_distance` / `obstacle_clear_distance`：障碍物急停/恢复阈值
- `safety.data_timeout_sec`：传感器数据超时，停止输出
- `safety.joint_limits`：机械臂关节限位
- `task_manager.task_timeout_sec`：Action 等待推理/指令的超时
- `task_manager.execution_hold_sec`：检测到非零控制指令后保持执行的时间

## 8. 测试

```bash
cd software/center_server
PYTHONPATH=.:$PYTHONPATH python3 -m pytest tests -q
```

当前 15 个单元测试覆盖状态机、安全校验、动作规划、预处理、VLA 桩与反向操作。
`tests/end_to_end_check.py` 为端到端测试，需要先启动 launch 与 `fake_edge_publisher.py`。

## 9. 与 DESIGN.md 对照的修复点

1. **构建文件**：`package.json` 实为 `package.xml` 内容且位置错误，已改为标准
   `package.xml`；补全 `setup.py`、`setup.cfg`、`resource/center_server` 和 6 个
   console_scripts。
2. **消息包嵌套**：`custom_msgs` 移为同级包，否则 colcon 只识别 `center_server`。
3. **Action 格式**：`ExecuteTask.action` 原缺少 `---` 且 Goal/Result/Feedback 顺序错误，
   已按 ROS2 的 Goal/Result/Feedback 顺序修正。
4. **缺失节点**：补全 `safety_manager_node`、`task_manager_node`，实现
   `/center/set_mode`、`/center/emergency_stop`、`/center/execute_task`。
5. **安全链路**：动作生成前做速度/障碍物/关节限位校验；无雷达数据拒绝运动；
   急停、手动模式、数据超时、反向操作均会抑制普通动作。
6. **反向操作优先级**：反向节点在回退期间按 10 Hz 重复发布，覆盖普通控制指令。
7. **时间同步**：由严格 `TimeSynchronizer` 改为 `ApproximateTimeSynchronizer`。
8. **时钟鲁棒性**：控制/安全循环使用 `STEADY_TIME` 定时器，避免墙钟回拨导致定时器
   长时间不触发；回退与超时判断使用 `time.monotonic()`。
9. **任务成功条件**：`ExecuteTask` 不再“收到推理结果即成功”，而是等 task_manager
   观察到 action_generator 真正发布非零 `/cmd_vel` 后，再保持 `execution_hold_sec`
   并返回成功。

## 10. 当前限制

1. **VLA 为规则桩**：未包含模型权重与 GPU 推理；`VLAEngine.predict` 根据任务指令
   关键词输出 `forward/backward/turn_left/turn_right/stop`。接入真实模型时替换
   `_load_model` 与 `predict` 即可，接口与 DESIGN.md §5.3 一致。
2. **音频消息**：因 `audio_common_msgs` 不在 Humble 基础安装中，使用自定义
   `custom_msgs/AudioData`，话题与同步流程不变。
3. **机械臂**：当前只实现急停保持与复位轨迹；真实抓取/放置轨迹需 IK 与模型输出。
4. **单步任务**：`ExecuteTask` 验证的是一次动作下发；长任务闭环需要 VLA 返回任务
   完成状态。
