# center_server（ROS2：VLA + SLAM + 导航 + 目标搜索 + 健康检查）

设计文档：[`DESIGN.md`](DESIGN.md)。
健康检查接口契约：[`HEALTHCHECK.md`](HEALTHCHECK.md)。
硬件方案与缺口分析：[`hardware/SLAM_NAV.md`](../../hardware/SLAM_NAV.md)。

本包在 v1 的 VLA 推理/安全/反向操作之上，补齐：

- **SLAM 建图**：slam_toolbox 消费、地图保存/加载、前沿提取；
- **定位**：ICM-20948 + 轮式里程计 EKF、AMCL/TF、定位健康度与初始位姿；
- **自主导航**：Nav2 或内置直航，目标校验、停滞检测、超时/急停；
- **目标搜索**：颜色/YOLO 检测、激光测距、map 投影、前沿探索、确认、接近；
- **速度仲裁**：VLA/Nav2/反向/安全统一到唯一 `/cmd_vel`；
- **底盘桥**：Pi ↔ STM32 二进制串口协议、里程计、电池、IMU 转发；
- **健康检查**：ROS 节点/话题 + 相机/雷达/IMU/麦克风/音箱/机械臂等设备，
  HTTP 供 `edge_center` 调用，另有 ROS 话题/服务。

验证状态（ROS2 Humble，本仓库）：

| 项目 | 结果 |
|---|---|
| `colcon build --packages-select custom_msgs center_server` | 通过 |
| `pytest tests -q` | 59 passed |
| `tests/end_to_end_check.py` | 7/7 passed |
| `tests/slam_nav_check.py` + `fake_slam_robot.py` | 6/6 passed |

> VLA 仍是规则桩（无模型权重）；STM32 固件仍需按 `DESIGN.md §6.13` 升级二进制协议；
> Nav2/建图参数是起步值，需实车标定。

---

## 1. 环境

- Ubuntu 22.04 + ROS2 Humble
- Python 3.10
- 必需：`rclpy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`trajectory_msgs`、
  `std_msgs`、`action_msgs`、`cv_bridge`、`message_filters`、`tf2_ros`、
  `python3-numpy`、`python3-opencv`、`python3-yaml`、`python3-serial`
- 可选：
  - `python3-smbus2`（Pi I2C 直连 ICM-20948）
  - `nav2_bringup`、`nav2_msgs`（Nav2 后端）
  - `slam_toolbox`（建图）
  - `robot_localization`（EKF）
  - `ultralytics`（YOLO 检测）
- 消息包：`software/custom_msgs`（ament_cmake）

> 原 v1 依赖 `audio_common_msgs`、`torch`、`librosa`、`pydantic`、`loguru`，这些不在
> Humble 基础安装中。现已改为自定义 `AudioData`、numpy 特征、rclpy/stdlib logging。

---

## 2. 构建

```bash
cd /path/to/rush-mog
mkdir -p ~/rush_mog_ws/src
ln -sfn "$(pwd)/software/ros" ~/rush_mog_ws/src/center_server
ln -sfn "$(pwd)/software/custom_msgs" ~/rush_mog_ws/src/custom_msgs

cd ~/rush_mog_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_msgs center_server --symlink-install
source install/setup.bash
```

修改源码后重新 `colcon build --packages-select custom_msgs center_server`。

---

## 3. 启动

### 3.1 仿真/桌面联调（默认 config.yaml）

```bash
ros2 launch center_server center_server.launch.py
# 或指定配置：
ros2 launch center_server center_server.launch.py config_file:=/path/to/config.yaml
```

启动 15 个节点（可选节点按 launch 参数裁剪）：

```bash
# 只跑核心 VLA 链路
ros2 launch center_server center_server.launch.py \
  start_perception:=false start_navigation:=false start_object_search:=false \
  start_map_manager:=false start_localization:=false
```

### 3.2 实车（robot.yaml + STM32 串口桥）

```bash
ros2 launch center_server robot_bringup.launch.py
# IMU 直连 Pi I2C 时：
ros2 launch center_server robot_bringup.launch.py start_imu:=true
# 指定配置：
ros2 launch center_server robot_bringup.launch.py config_file:=/path/to/robot.yaml
```

`robot_bringup.launch.py` 会启动 `base_bridge_node`（`base.enabled=true`）。
启动前确认：
- 串口 udev 规则（示例 `/etc/udev/rules.d/99-rush-mog.rules`）：
  ```
  SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="rush_base"
  ```
- `python3-serial` 已安装；
- 激光驱动已发布 `/scan`；
- 相机驱动已发布 `/camera/image_raw`；
- 12V/5V/6V 共地，急停释放。

### 3.3 外部 SLAM/Nav2 栈（可选）

```bash
# 建图
ros2 launch center_server slam_stack.launch.py

# 定位/导航（注意：Nav2 最终速度被重映射到 /nav/cmd_vel）
ros2 launch center_server nav2_stack.launch.py map:=$HOME/rush_mog/maps/rush_mog.yaml

# EKF（单独启动）
ros2 run robot_localization ekf_node --ros-args \
  --params-file <install>/share/center_server/config/ekf.yaml
```

若未安装 Nav2，`navigation.backend: auto` 会自动回退到内置直航，只需 `/scan`、`/center/pose`。

---

## 4. 节点

| 节点 | 职责 |
|---|---|
| `center_bridge_node` | image/laser/odom 软同步；audio/joint/imu 最近值缓存 |
| `vla_inference_node` | 多模态预处理 + VLA 规则桩，发布意图/结果 |
| `action_generator_node` | 单步意图 → `/cmd_vel_raw`，发布前安全校验 |
| `reverse_operation_node` | 急停/回退/机械臂复位/任务撤销/手动接管 → `/reverse/cmd_vel` |
| `safety_manager_node` | 障碍/数据/电量/定位/急停 → `/center/safety_state`、`/safety/cmd_vel_override` |
| `task_manager_node` | 状态机、`/center/execute_task`、`/center/execute_mission`、`/center/set_mode` |
| `velocity_mux_node` | 多源速度仲裁 → `/cmd_vel` |
| `localization_node` | 位姿选择、定位健康度、`/center/set_initial_pose` |
| `map_manager_node` | `/map` 消费、前沿、`/center/map_control`、PGM/YAML 存图 |
| `navigation_node` | `/center/navigate_to`，Nav2/直航后端 |
| `target_detector_node` | 颜色/YOLO 检测、激光/bbox 测距、map 投影、跟踪 |
| `object_search_node` | `/center/explore`、`/center/search_object` |
| `base_bridge_node` | Pi ↔ STM32 串口、`/odom`、`/imu/data`、`/battery_state` |
| `imu_node` | Pi I2C 直连 ICM-20948 → `/imu/data` |
| `health_monitor_node` | 节点/设备健康、`/center/health`、`/center/healthcheck`、HTTP `:8090` |

---

## 5. 接口

### 5.1 话题（主要）

| 话题 | 类型 | 方向 |
|---|---|---|
| `/scan`、`/edge/radar` | `sensor_msgs/LaserScan` | 订阅 |
| `/camera/image_raw`、`/edge/video` | `sensor_msgs/Image` | 订阅 |
| `/edge/audio` | `custom_msgs/AudioData` | 订阅 |
| `/odom`、`/robot/odometry` | `nav_msgs/Odometry` | 订阅 |
| `/imu/data` | `sensor_msgs/Imu` | 订阅 |
| `/joint_states` | `sensor_msgs/JointState` | 订阅 |
| `/arm/joint_states` | `sensor_msgs/JointState` | 订阅（健康检查，待接入） |
| `/body/audio_status` | `std_msgs/String` | 订阅（健康检查，待接入） |
| `/battery_state` | `sensor_msgs/BatteryState` | 订阅 |
| `/base/status` | `std_msgs/String` | 发布/订阅（STM32 遥测 JSON） |
| `/map` | `nav_msgs/OccupancyGrid` | 订阅/发布 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 订阅 |
| `/center/synced_data` | `custom_msgs/SyncedSensorData` | 发布 |
| `/center/robot_state` | `custom_msgs/RobotState` | 发布 |
| `/center/action_intent` | `custom_msgs/ActionIntent` | 发布 |
| `/center/inference_result` | `custom_msgs/InferenceResult` | 发布 |
| `/center/status` | `custom_msgs/SystemStatus` | 发布 |
| `/center/mission_status` | `custom_msgs/MissionStatus` | 发布 |
| `/center/safety_state` | `custom_msgs/SafetyState` | 发布 |
| `/center/map_info` | `custom_msgs/MapInfo` | 发布 |
| `/center/frontiers` | `custom_msgs/FrontierList` | 发布 |
| `/center/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 发布 |
| `/center/localization_status` | `custom_msgs/LocalizationStatus` | 发布 |
| `/center/nav_status` | `custom_msgs/NavStatus` | 发布 |
| `/center/target_detections` | `custom_msgs/TargetList` | 发布 |
| `/center/search_status` | `custom_msgs/SearchStatus` | 发布 |
| `/center/velocity_status` | `custom_msgs/VelocityMuxStatus` | 发布 |
| `/center/health` | `custom_msgs/HealthStatus` | 发布（1 Hz，TRANSIENT_LOCAL） |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布（唯一最终出口） |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | 发布（VLA） |
| `/nav/cmd_vel` | `geometry_msgs/Twist` | 发布（导航） |
| `/reverse/cmd_vel` | `geometry_msgs/Twist` | 发布（反向） |
| `/safety/cmd_vel_override` | `geometry_msgs/Twist` | 发布（安全） |
| `/arm/trajectory` | `trajectory_msgs/JointTrajectory` | 发布 |
| `/task/request` | `custom_msgs/TaskRequest` | 发布/订阅 |
| `/center/mission_request` | `custom_msgs/TaskRequest` | 订阅 |

### 5.2 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `/center/set_mode` | `custom_msgs/SetMode` | `auto` / `manual` / `reverse` |
| `/center/emergency_stop` | `custom_msgs/EmergencyStop` | 急停/解除 |
| `/center/map_control` | `custom_msgs/MapControl` | `start`/`stop`/`save`/`load`/`reset` |
| `/center/set_initial_pose` | `custom_msgs/SetInitialPose` | 设置 AMCL 初始位姿 |
| `/center/healthcheck` | `custom_msgs/HealthCheck` | 全部/指定设备健康快照 |

### 5.3 Action

| Action | 说明 |
|---|---|
| `/center/execute_task` | VLA 单步；指令命中任务关键词时自动转长任务 |
| `/center/execute_mission` | `map`/`explore`/`search`/`navigate`/`return_home` |
| `/center/navigate_to` | 到点导航（Nav2/直航） |
| `/center/explore` | 前沿探索到覆盖率 |
| `/center/search_object` | 搜索目标并接近 |

示例：

```bash
# 前进（VLA）
ros2 action send_goal /center/execute_task custom_msgs/action/ExecuteTask \
  "{task_id: demo1, instruction: 前进}" --feedback

# 找杯子（自动转 search 长任务）
ros2 action send_goal /center/execute_task custom_msgs/action/ExecuteTask \
  "{task_id: find_cup, instruction: 找杯子}" --feedback

# 建图并保存
ros2 action send_goal /center/execute_mission custom_msgs/action/ExecuteMission \
  "{mission_id: map1, mission_type: map, target_coverage: 0.9, save_map: true}" --feedback

# 到点导航
ros2 action send_goal /center/navigate_to custom_msgs/action/NavigateTo \
  "{target_pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}, goal_id: g1, timeout_sec: 60.0}" --feedback

# 搜索杯子
ros2 action send_goal /center/search_object custom_msgs/action/SearchObject \
  "{task_id: s1, target_label: cup, max_duration: 120.0}" --feedback
```

### 5.4 健康检查（供 edge_center）

详细契约、字段语义和接入示例见 [`HEALTHCHECK.md`](HEALTHCHECK.md)。主接口：

```bash
# 详细健康（HTTP 200 只代表数据可读；机器健康看 body.status/healthy/ready）
curl -s http://127.0.0.1:8090/ros/health | python3 -m json.tool

# 存活/就绪探针
curl -s http://127.0.0.1:8090/ros/healthz
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/ros/readyz

# 单设备 / 不返回 metrics
curl -s http://127.0.0.1:8090/ros/health/devices/lidar | python3 -m json.tool
curl -s 'http://127.0.0.1:8090/ros/health?include_metrics=false' | python3 -m json.tool
```

ROS 原生接口：

```bash
ros2 topic echo /center/health --once
ros2 service call /center/healthcheck custom_msgs/srv/HealthCheck \
  "{device_ids: [camera, lidar], include_metrics: false}"
```

- 默认只监听 `127.0.0.1:8090`；跨机调用改 `health.http.host: 0.0.0.0` 并设置 `health.http.api_key`。
- 设备清单在 `health.devices` 配置：camera/lidar/imu/microphone/speaker/arm/base/battery/odom。
- 音箱 `/body/audio_status`、机械臂 `/arm/joint_states` 当前无反馈时显示 `unknown`，不伪造 `ok`；
  接入后无需改代码。
- 整体状态：`ok` / `starting` / `degraded` / `error` / `disabled`；
  `ready=true` 表示可接新任务（`ok`/`degraded` 且未急停）。

---

## 6. 配置

默认配置安装到 `share/center_server/config/`：

| 文件 | 用途 |
|---|---|
| `config.yaml` | 仿真/桌面默认；`/edge/*` 话题；`perception.enabled=false`；`base.enabled=false` |
| `robot.yaml` | 实车；`/scan`、`/camera/image_raw`、`base.enabled=true`；完整健康设备清单 |
| `ekf.yaml` | `robot_localization` |
| `slam_toolbox.yaml` | `slam_toolbox` |
| `nav2_params.yaml` | Nav2 Humble 起步配置 |

关键参数：

- `vla_engine.inference_rate` / `confidence_threshold`
- `safety.obstacle_stop_distance` / `data_timeout_sec` / `battery_low_percent`
- `localization.min_quality`
- `map_manager.frontier_min_size` / `map_dir`
- `navigation.backend`（auto/nav2/direct） / `stuck_timeout_sec`
- `search.standoff_distance` / `confirm_hits` / `explore_coverage`
- `velocity_mux.priorities`
- `task_manager.execution_hold_sec`
- `health.publish_rate` / `startup_grace_sec` / `strict_unknown`
- `health.http.host` / `port` / `api_key`
- `health.devices.<id>` 的 `topic` / `message_type` / `expected_rate_hz` / `timeout_sec` / `required`

---

## 7. 本地联调与测试

> **DDS 提示（WSL/容器/沙箱常见）**：如果节点/动作发现不稳定、`wait_for_server` 超时，
> 可强制本机 UDP 发现：
>
> ```bash
> export ROS_DOMAIN_ID=77
> export ROS_LOCALHOST_ONLY=1
> export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
> ```
>
> `RTPS_TRANSPORT_SHM ... Permission denied` 在沙箱中是已知噪声，不一定会阻断通信。

### 7.1 单元测试

```bash
cd software/ros
PYTHONPATH=.:$PYTHONPATH python3 -m pytest tests -q
# 59 passed
```

单独跑健康检查（不依赖 ROS 图）：

```bash
PYTHONPATH=.:$PYTHONPATH python3 -m pytest tests/test_health_monitor.py tests/test_health_http.py -q
# 24 passed
```

### 7.2 VLA/安全端到端

终端 A：

```bash
source ~/rush_mog_ws/install/setup.bash
ros2 launch center_server center_server.launch.py
```

终端 B：

```bash
python3 tests/fake_edge_publisher.py
```

终端 C：

```bash
python3 tests/end_to_end_check.py
# RESULT: 7/7 passed
```

### 7.3 SLAM/导航/搜索联调

本联调不需要真实硬件，`fake_slam_robot.py` 会模拟地图/激光/位姿/定位/目标。

终端 A：

```bash
export ROS_HOME=/tmp/roshome ROS_LOG_DIR=/tmp/roslog
ros2 launch center_server center_server.launch.py \
  config_file:=$(pwd)/tests/config_test.yaml \
  start_perception:=false start_localization:=false
```

> 必须关闭 `localization_node`，因为 `fake_slam_robot` 自己发布
> `/center/localization_status`；两个源会互相覆盖质量值。

终端 B：

```bash
python3 tests/fake_slam_robot.py
```

终端 C：

```bash
python3 tests/slam_nav_check.py
# RESULT: 6/6 passed
```

覆盖：直航到点、探索覆盖率、搜索 cup、地图保存/加载、`ExecuteTask "找杯子"`、
`ExecuteMission map`。

### 7.4 常用调试命令

```bash
ros2 topic echo /center/map_info --once
ros2 topic echo /center/frontiers --once
ros2 topic echo /center/localization_status
ros2 topic echo /center/nav_status
ros2 topic echo /center/target_detections
ros2 topic echo /center/health --once
ros2 topic echo /cmd_vel
curl -s http://127.0.0.1:8090/ros/health | python3 -m json.tool
curl -s http://127.0.0.1:8090/ros/health/devices/camera | python3 -m json.tool
ros2 service call /center/healthcheck custom_msgs/srv/HealthCheck "{}"
ros2 service call /center/map_control custom_msgs/srv/MapControl "{command: save, map_name: test}"
ros2 service call /center/emergency_stop custom_msgs/srv/EmergencyStop "{trigger: true}"
```

---

## 8. 实车落地步骤

1. **计算平台**：Pi 4B/5 或 Orin Nano 装 Ubuntu 22.04 + ROS2 Humble。
2. **激光**：LD19/LD06 接 USB，驱动发布 `/scan`。
3. **编码器 + 主控**：升级 STM32F103RCT6/F411，4 路编码器 + 4 路 PWM + 轮速 PID。
4. **ICM-20948**：Pi I2C（`imu.enabled=true`）或 STM32 转发（`base_bridge_node`）。
5. **EKF**：`robot_localization` 融合 `/odom` + `/imu/data`，确认 TF。
6. **SLAM**：`slam_stack.launch.py` + `ExecuteMission map`，保存地图。
7. **定位**：AMCL/map_server，设置初始位姿。
8. **导航/搜索**：Nav2 或直航；目标检测/搜索。
9. **安全**：硬件急停、INA226、悬崖/碰撞、看门狗。
10. **健康检查**：`robot_bringup.launch.py` 已启动 `health_monitor_node`；确认
    `curl http://127.0.0.1:8090/ros/health` 返回 `status=ok`，并让 `edge_center` 接入。

硬件选型、接线、引脚冲突、标定与验收见 [`hardware/SLAM_NAV.md`](../../hardware/SLAM_NAV.md)。

---

## 9. 当前限制

1. **VLA 为规则桩**：未包含模型权重/GPU 推理；替换 `VLAEngine._load_model/predict` 即可。
2. **STM32 固件未升级**：主机侧二进制协议已实现，固件仍 ASCII；需按 `DESIGN.md §6.13` 实现。
3. **Nav2 参数起步值**：需实车调 `robot_radius`、速度、膨胀半径、DWB critics。
4. **无深度相机**：目标距离靠激光/bbox 高度；遮挡/高度变化精度有限。
5. **磁力计需校准**：ICM-20948 受电机干扰时只慢速校正，不硬切航向。
6. **单地图/单楼层**：未实现多地图切换/电梯/多层。
7. **动态目标**：搜索假设目标基本静止；移动目标需预测/重规划。
8. **颜色检测仅演示**：真实场景建议 YOLO/深度相机/Orin Nano。
9. **健康反馈缺口**：音箱 `/body/audio_status`、机械臂 `/arm/joint_states` 尚未发布，
   健康接口显示 `unknown`；接入后自动变为 `ok`/`degraded`/`error`。

---

## 10. 目录

```text
ros/
├── center_server/
│   ├── config/          # config.yaml / robot.yaml / ekf.yaml / slam_toolbox.yaml / nav2_params.yaml
│   ├── core/            # 纯算法，可单测（含 health_monitor.py）
│   ├── drivers/         # ICM-20948
│   ├── http_health.py   # 健康检查 HTTP 网关（stdlib）
│   └── utils/
├── ros_nodes/           # 15 个 ROS2 节点（含 health_monitor_node）
├── launch/              # center_server / robot_bringup / slam_stack / nav2_stack
├── tests/               # 单元 + 仿真 + 联调（含 test_health_*.py）
├── DESIGN.md
├── HEALTHCHECK.md       # 健康检查接口契约（HTTP + ROS）
├── README.md
├── package.xml
└── setup.py
```
