# edge_server（ROS2：Pi Edge Server + VLA + SLAM + 导航 + 目标搜索 + 健康检查）

设计文档：[`docs/DESIGN.md`](docs/DESIGN.md)。
代码结构与调用链：[`docs/CODE_MAP.md`](docs/CODE_MAP.md)。
健康检查接口契约：[`docs/HEALTHCHECK.md`](docs/HEALTHCHECK.md)。
部署与发布说明：[`docs/DEPLOY.md`](docs/DEPLOY.md)。
**实机启停/日志/排障：[`docs/OPERATIONS.md`](docs/OPERATIONS.md)。**
硬件方案与缺口分析：[`hardware/SLAM_NAV.md`](../../hardware/SLAM_NAV.md)。

本包在 v1 的 VLA 推理/安全/反向操作之上，补齐：

- **SLAM 建图**：slam_toolbox 消费、地图保存/加载、前沿提取；
- **定位**：ICM-20948 + 轮式里程计 EKF、AMCL/TF、定位健康度与初始位姿；
- **自主导航**：Nav2 或内置直航，目标校验、停滞检测、超时/急停；
- **目标搜索**：颜色/YOLO 检测、激光测距、map 投影、前沿探索、确认、接近；
- **速度仲裁**：VLA/Nav2/反向/安全统一到唯一 `/cmd_vel`；
- **Pi Edge Server**：80 端口网页/API、8080 MJPEG、树莓派直连 STM32 USART3（115200 ASCII）、
  网页手动控制与 ROS `/cmd_vel` 共用串口出口；旧 ESP32 HTTP 桥仅保留历史兼容；
- **蓝牙耳机（音箱 + 麦克风）**：自动连接 `newmine`（BlueZ + PipeWire A2DP），
  TTS（espeak-ng 中文）与本地音频排队播放；录音时切 HFP、停止后恢复 A2DP，
  提供 `/api/speaker/*`、`/api/microphone/*`、`/body/say`、`/body/audio_status`、
  `/body/mic_status`、`/edge/audio`，dashboard 可直接播报/录音；
- **树莓派电源监控**：`vcgencmd get_throttled` + `rpi_volt` 欠压告警，
  驱动 dashboard“电源及转换板”模块，当前正常即报 `ok`，欠压/降频报
  `degraded`/`error`，`/api/power` 可直接查询；
- **健康检查**：ROS 节点/话题 + 相机/雷达/IMU/麦克风/音箱/机械臂/电源等设备，
  HTTP 供 Pi 上的 Edge Server/dashboard 代理，另有 ROS 话题/服务。

验证状态（ROS2 Humble，本仓库）：

| 项目 | 结果 |
|---|---|
| `colcon build --packages-select custom_msgs edge_server` | 通过 |
| `pytest tests -q` | 125 passed |
| `tests/end_to_end_check.py` | 7/7 passed |
| `tests/slam_nav_check.py` + `fake_slam_robot.py` | 6/6 passed |

> VLA 仍是规则桩（无模型权重）；当前 STM32 ASCII 固件已由 `edge_server_node` 直连，若需二进制闭环再按 `DESIGN.md §6.13` 的 legacy 小节升级；
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
  - `pipewire`、`wireplumber`、`libspa-0.2-bluetooth`、`pulseaudio-utils`、
    `alsa-utils`、`espeak-ng`（蓝牙耳机播放/麦克风 + 中文 TTS；见 §5.6/§5.7）
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
ln -sfn "$(pwd)/software/edge_server" ~/rush_mog_ws/src/edge_server
ln -sfn "$(pwd)/software/custom_msgs" ~/rush_mog_ws/src/custom_msgs

cd ~/rush_mog_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_msgs edge_server --symlink-install
source install/setup.bash
```

修改源码后重新 `colcon build --packages-select custom_msgs edge_server`。

修改增量构建部署：
```bash
cd /mnt/f/CODE/rush-mog

rsync -avz --delete \
  --exclude='__pycache__' --exclude='.pytest_cache' \
  software/edge_server/ \
  ubuntu@192.168.3.26:/opt/edge_server/src/edge_server/

rsync -avz --delete \
  --exclude='__pycache__' --exclude='.pytest_cache' \
  software/custom_msgs/ \
  ubuntu@192.168.3.26:/opt/edge_server/src/custom_msgs/

# 注意：远端登录 shell 是 zsh，必须显式用 bash -lc 包一层

ssh ubuntu@192.168.3.26 'bash -lc "
  source /opt/ros/humble/setup.bash &&
  cd /opt/edge_server &&
  colcon build --packages-select custom_msgs edge_server &&
  sudo systemctl restart edge_server
"'

# 只改了 `edge_server` 的 Python/网页、没有改消息时，可以只构建 `edge_server`：

ssh ubuntu@192.168.3.26 'bash -lc "
  source /opt/ros/humble/setup.bash &&
  cd /opt/edge_server &&
  colcon build --packages-select edge_server &&
  sudo systemctl restart edge_server
"'

```

---

## 3. 启动

### 3.1 仿真/桌面联调（默认 config.yaml）

```bash
ros2 launch edge_server edge_server.launch.py
# 或指定配置：
ros2 launch edge_server edge_server.launch.py config_file:=/path/to/config.yaml
```

启动核心节点（可选节点按 launch 参数裁剪）：

```bash
# 只跑核心 VLA 链路
ros2 launch edge_server edge_server.launch.py \
  start_perception:=false start_navigation:=false start_object_search:=false \
  start_map_manager:=false start_localization:=false
```

### 3.2 实车（robot.yaml + Pi Edge Server 直连 STM32）

```bash
ros2 launch edge_server robot_bringup.launch.py
# IMU 直连 Pi I2C 时：
ros2 launch edge_server robot_bringup.launch.py start_imu:=true
# 指定配置：
ros2 launch edge_server robot_bringup.launch.py config_file:=/path/to/robot.yaml
```

`robot_bringup.launch.py` 默认启动 `edge_server_node`（`base_transport:=edge_serial`）：
- 80 端口提供手柄页、dashboard、`/cmd`、`/status`、`/api/*`；8080 提供 MJPEG；
- 树莓派 UART 直连 STM32 USART3，115200 8N1，发送 ASCII 方向+PWM 命令，
  解析 `$B,...` 四轮编码器遥测；
- 发布 `/odom`、`/joint_states`、`/base/status`；订阅 `/cmd_vel` 并量化成
  `方向+PWM` 命令；
- 网页按钮/摇杆按下时下发一次、松开/停止时再下发一次，默认不周期性重发
  （`edge.motion.resend_manual=true` 可恢复旧行为）；ROS `raw:` 命令保留
  PWM 第二字节，按 `keepalive_ms` 刷新并带 1s 租约；网页手动控制优先于 ROS 导航，
  直到网页松开/停止；
- `/cmd_vel` 超时、急停或 STM32 遥测超时都会自动停车。

串口 udev 规则（示例 `/etc/udev/rules.d/99-rush-mog.rules`）：
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="rush_base"
```
`robot.yaml` 的 `base.port` 可改为 `/dev/rush_base`；并确认 `python3-serial` 已安装。

80 端口绑定权限二选一：
```bash
# 使用 systemd 时由 root 启动，或：
sudo setcap cap_net_bind_service=+ep /usr/bin/python3
```

网络：树莓派用 `hostapd` + `dnsmasq` 提供 `RUSH-MOG` AP（`192.168.4.1`），
或接入现有局域网；手机/PC 打开 `http://<pi-ip>/`，视频流为
`http://<pi-ip>:8080/stream`。

其他启动前确认：
- 激光驱动已发布 `/scan`；
- 相机驱动已发布 `/camera/image_raw`（Edge Server 订阅并转成 MJPEG）；
- 12V/5V/6V 共地，急停释放。

旧模式仍可用（不推荐）：
```bash
# 旧二进制串口桥（需把 robot.yaml 的 base.baud 改回 921600）
ros2 launch edge_server robot_bringup.launch.py base_transport:=serial_binary
# 旧 ESP32 HTTP 桥（需有 ESP32 固件/网络）
ros2 launch edge_server robot_bringup.launch.py base_transport:=esp32_http
```

### 3.3 外部 SLAM/Nav2 栈（可选）

```bash
# 建图
ros2 launch edge_server slam_stack.launch.py

# 定位/导航（注意：Nav2 最终速度被重映射到 /nav/cmd_vel）
ros2 launch edge_server nav2_stack.launch.py map:=$HOME/rush_mog/maps/rush_mog.yaml

# EKF（单独启动）
ros2 run robot_localization ekf_node --ros-args \
  --params-file <install>/share/edge_server/config/ekf.yaml
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
| `edge_server_node` | Pi Edge Server：80 网页/API、8080 MJPEG、STM32 串口、蓝牙音箱/语音播报、HFP 录音与 `/edge/audio`、树莓派电源监控、`/odom`、`/joint_states`、`/base/status`、`/body/audio_status`、`/body/mic_status`、`/body/power_status` |
| `base_bridge_node` | 旧版 Pi ↔ STM32 二进制串口桥（legacy） |
| `imu_node` | Pi I2C 直连 ICM-20948 → `/imu/data` |
| `health_monitor_node` | 节点/设备健康、`/center/health`、`/center/healthcheck`、HTTP `:8090` |

---

## 5. 接口

### 5.1 话题（主要）

| 话题 | 类型 | 方向 |
|---|---|---|
| `/scan`、`/edge/radar` | `sensor_msgs/LaserScan` | 订阅 |
| `/camera/image_raw`、`/edge/video` | `sensor_msgs/Image` | 订阅 |
| `/edge/audio` | `custom_msgs/AudioData` | 发布（录音时 10 Hz PCM）/订阅（bridge） |
| `/odom`、`/robot/odometry` | `nav_msgs/Odometry` | 订阅 |
| `/imu/data` | `sensor_msgs/Imu` | 订阅 |
| `/joint_states` | `sensor_msgs/JointState` | 订阅 |
| `/arm/joint_states` | `sensor_msgs/JointState` | 订阅（健康检查，待接入） |
| `/body/say` | `std_msgs/String` | 订阅（文本 -> TTS -> 蓝牙音箱） |
| `/body/audio_status` | `std_msgs/String` | 发布（音箱状态 JSON，1 Hz；健康检查 speaker） |
| `/body/mic_status` | `std_msgs/String` | 发布（麦克风状态 JSON，1 Hz；健康检查 microphone） |
| `/body/power_status` | `std_msgs/String` | 发布（树莓派电源状态 JSON，1 Hz；健康检查 power） |
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

### 5.4 健康检查（供 Edge Server / dashboard）

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

参数查看/修改（运行时生效，重启回 YAML）：

```bash
curl -s http://127.0.0.1:8090/ros/params | python3 -m json.tool
curl -X POST http://127.0.0.1:8090/ros/params \
  -H 'Content-Type: application/json' \
  -d '{"updates":{"publish_rate":2.0,"device.lidar.timeout_sec":2.0}}'
```

ROS 原生接口：

```bash
ros2 topic echo /center/health --once
ros2 service call /center/healthcheck custom_msgs/srv/HealthCheck \
  "{device_ids: [camera, lidar], include_metrics: false}"
```

- 代码默认监听 `127.0.0.1:8090`；`robot.yaml` 也保持回环，由同机 Edge Server 代理；跨机监控再改 `0.0.0.0` 并设置 `health.http.api_key`。
- 设备参数（频率/超时/必需）与 health monitor 参数可在 dashboard 的 ROS 参数抽屉直接修改。
- 设备清单在 `health.devices` 配置：camera/lidar/imu/microphone/speaker/arm/base/battery/power/odom。
- 音箱由 `edge_server_node` 发布 `/body/audio_status`，`online=false` 时显示 `degraded`；
  机械臂 `/arm/joint_states` 无反馈时显示 `unknown`，不伪造 `ok`；接入后无需改代码。
- 整体状态：`ok` / `starting` / `degraded` / `error` / `disabled`；
  `ready=true` 表示可接新任务（`ok`/`degraded` 且未急停）。

### 5.5 Edge Server HTTP API（80/8080）

手柄页、dashboard 和移动端只访问 Pi Edge Server：

| 路径 | 说明 |
|---|---|
| `GET /`、`GET /dashboard.html` | 手柄页 / 数字孪生页面及静态资源 |
| `POST /cmd` | 统一控制入口：`button:` / `stick:` / `raw:` |
| `GET/POST /cmd-button`、`/cmd-stick`、`/stop` | 旧页面兼容接口 |
| `GET /status` | Pi 运行状态、串口/ROS/相机/运动状态（含当前指令 `motion`） |
| `GET /api/body` | STM32 编码器遥测快照 + 当前指令/来源/持续时长 |
| `GET /api/events?limit=N` | 最近 N 条运行事件（命令、串口连接、急停、租约超时） |
| `GET /api/speaker` | 蓝牙音箱连接/A2DP sink/TTS/播放队列状态 |
| `POST /api/speaker/{connect,say,play,volume,stop}` | 连接音箱、TTS 播报、播放 `audio_dir` 文件、音量、停止 |
| `GET /api/microphone` | HFP 麦克风能力/输入源/录音状态/最近录音 |
| `POST /api/microphone/{start,stop,play,volume,list}` | 开始/停止录音、回放、音量、录音列表 |
| `GET /api/microphone/recordings[/{name}]` | 录音列表 / 下载 WAV |
| `GET /api/power` | 树莓派欠压/降频/温度/核心电压状态 |
| `GET /api/modules` | Edge + 同机 ROS 子系统 + 外部模块（CENTER） |
| `GET/POST /api/modules/edge/params` | Edge 参数（运动保活、图传帧率） |
| `GET /api/modules/{id}/health|params` | 代理 ROS（同机）/CENTER（外部） |
| `POST /api/modules/report` | CENTER 等外部模块上报（`edge`/`ros` 保留） |
| `GET /stream`（8080） | ROS 相机的 MJPEG；80 端口返回 302 |

完整 OpenAPI 定义见 [`software/api.yaml`](../api.yaml)；实现见
`edge_server/edge_web.py`、`edge_server/edge_control.py` 和
`ros_nodes/edge_server_node.py`。

### 5.6 蓝牙音箱与语音播报（`newmine`）

`edge_server_node` 内置 `SpeakerManager`，启动后自动搜索/配对/信任/连接
`edge.speaker.device_name`（默认 `newmine`），连接成功后等待 PipeWire 创建
A2DP sink 并设为默认输出；断线后每 `connect_interval_sec` 秒重试。

主机依赖（Ubuntu 22.04 / Pi 4B）：

```bash
sudo apt-get install -y pipewire pipewire-pulse wireplumber \
  libspa-0.2-bluetooth pulseaudio-utils alsa-utils espeak-ng
sudo loginctl enable-linger ubuntu
sudo -u ubuntu XDG_RUNTIME_DIR=/run/user/1000 \
  systemctl --user enable --now pipewire pipewire-pulse wireplumber
```

`edge_server.service` 需要把用户会话环境传给播放器（部署机已配置）：

```ini
Environment=XDG_RUNTIME_DIR=/run/user/1000
Environment=PULSE_SERVER=unix:/run/user/1000/pulse/native
Environment=DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
```

配置（`edge.speaker`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `enabled` | `false`（`robot.yaml` 为 `true`） | 总开关 |
| `device_name` | `newmine` | 按名称发现音箱 |
| `device_mac` | 空 | 填 MAC 可跳过扫描、快速重连 |
| `auto_connect` | `true` | 后台自动重连 |
| `connect_interval_sec` | `20` | 巡检/重试周期 |
| `volume` | `70` | 首次连接音量 0..100 |
| `audio_dir` | `~/.local/share/rush-mog/audio` | `POST /api/speaker/play` 允许的目录 |
| `say_topic` / `status_topic` | `/body/say` / `/body/audio_status` | ROS 话题 |
| `tts.backend` | `auto` | `espeak-ng` / `edge-tts` / `none` |
| `tts.voice` | `cmn` | espeak-ng 中文音色；英文用 `en-us` |

接口示例：

```bash
curl -s http://192.168.3.26/api/speaker | python3 -m json.tool
curl -X POST http://192.168.3.26/api/speaker/connect
curl -X POST http://192.168.3.26/api/speaker/say \
  -H 'Content-Type: application/json' -d '{"text":"机器人已启动"}'
curl -X POST http://192.168.3.26/api/speaker/volume \
  -H 'Content-Type: application/json' -d '{"volume":60}'
ros2 topic pub --once /body/say std_msgs/msg/String "{data: 已到达目标点}"
```

dashboard 的「蓝牙音箱与语音播报」面板提供连接、停止、音量、文字播报和快捷短语；
状态同时发布到 `/body/audio_status`，health monitor 用 `online` 字段判断音箱
`ok`/`degraded`。

### 5.7 蓝牙麦克风（HFP）

`newmine` 是 headset：A2DP 只负责播放，麦克风在 HFP/HSP profile 下才可用。
`MicrophoneManager` 在录音开始前把 PipeWire card 从 `a2dp-sink-*` 切到
`headset-head-unit`，等待 `bluez_input.<MAC>.headset-head-unit` 输入源，
用 `parec` 读原始 PCM（s16le / 8000 Hz / mono），同时：

- 写 WAV 到 `edge.speaker.microphone.record_dir`（默认 `<audio_dir>/mic`）；
- 每 `chunk_ms`（默认 100ms，10 Hz）发布 `custom_msgs/AudioData` 到 `/edge/audio`；
- 停止后自动恢复原 A2DP profile，语音播报恢复高音质输出。

录音期间输出会临时变成 HFP mono（8 kHz），这是蓝牙协议限制；如果此时播 TTS，
会走 HFP sink。录音最长 `max_duration_sec`（默认 60s），到点自动停止。

配置（`edge.speaker.microphone`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `enabled` | `true`（默认 config 为 false） | 总开关 |
| `record_dir` | 空 = `<audio_dir>/mic` | 录音目录，只允许此目录内文件 |
| `sample_rate` / `channels` | `8000` / `1` | HFP 窄带 PCM |
| `chunk_ms` | `100` | ROS AudioData 帧长度 |
| `max_duration_sec` | `60` | 单次录音上限 |
| `volume` | `100` | 麦克风输入音量 0..100 |
| `topic` / `status_topic` | `/edge/audio` / `/body/mic_status` | ROS 话题 |

```bash
curl -s http://192.168.3.26/api/microphone | python3 -m json.tool
curl -X POST http://192.168.3.26/api/microphone/start   -H 'Content-Type: application/json' -d '{"duration":10}'
curl -X POST http://192.168.3.26/api/microphone/stop
curl -s http://192.168.3.26/api/microphone/recordings
curl -OJ http://192.168.3.26/api/microphone/recordings/mic-20261114-123901-694.wav
curl -X POST http://192.168.3.26/api/microphone/play   -H 'Content-Type: application/json' -d '{"file":"mic-20261114-123901-694.wav"}'
```

ROS 侧录音时可以直接用 `ros2 topic hz /edge/audio`（约 10 Hz）确认；
`health_monitor_node` 用 `/body/mic_status` 的 `online` 判断麦克风
`ok`/`degraded`。dashboard 的同一面板提供开始/停止/下载/回放。

### 5.8 树莓派电源监控（电源及转换板）

“电源及转换板”模块的状态直接取树莓派电源监控：

- `vcgencmd get_throttled`：firmware 的欠压/降频/温度限制位；
- `/sys/class/hwmon/hwmon*/in0_lcrit_alarm`（`rpi_volt`）：当前低电压告警；
- `vcgencmd measure_volts core`、`measure_temp`：辅助展示；
- `/proc/device-tree/model`：板卡型号。

状态语义：

| 条件 | status |
|---|---|
| 当前欠压（`0x1` 或 alarm=1） | `error` |
| 当前降频/限流/软温度限制 | `degraded` |
| 仅历史位（`0x10000`/`0x40000`/`0x80000`） | `ok` + `warning=true` |
| 无欠压/降频 | `ok` |

`0x50000` 是本次部署实测值：表示启动后曾发生过欠压和降频，但当前已恢复，
所以 dashboard 电源模块显示 `ok`，详情标注“历史欠压已恢复”，
`/api/power` 的 `warning=true`、`throttled_raw="0x50000"`。

配置（`edge.power`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `enabled` | `false`（robot/deploy 为 true） | 总开关 |
| `sample_interval_sec` | `2.0` | 读取缓存周期 |
| `status_rate_hz` | `1.0` | `/body/power_status` 发布频率 |
| `status_topic` | `/body/power_status` | ROS 状态 JSON |
| `model_path` / `alarm_path` | Pi 默认路径 | 可覆盖 sysfs 路径 |
| `vcgencmd` | `vcgencmd` | 命令名/路径 |

```bash
curl -s http://192.168.3.26/api/power | python3 -m json.tool
# 关键字段：status, healthy, warning, undervoltage, throttled_raw,
#           core_voltage_v, temperature_c
```

dashboard 的 BOM「电源及转换板」卡片使用该状态；health monitor 的 `power`
设备读取 `/body/power_status`，电源异常时整体健康会降级。

---

## 6. 配置

默认配置安装到 `share/edge_server/config/`：

| 文件 | 用途 |
|---|---|
| `config.yaml` | 仿真/桌面默认；`/edge/*` 话题；`base.enabled=false`；Edge Web 使用 8081/8082 |
| `robot.yaml` | 实车；`/scan`、`/camera/image_raw`；`base.transport=edge_serial`；Edge Web 80/8080 |
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
- `base.transport` / `port` / `baud` / `body_command.*`
- `edge.web.port` / `stream_port` / `edge.camera.fps` / `edge.motion.keepalive_ms`
- `edge.speaker.*`：`enabled` / `device_name` / `device_mac` / `auto_connect` /
  `volume` / `audio_dir` / `tts.*`（见 §5.6）
- `edge.speaker.microphone.*`：`enabled` / `record_dir` / `sample_rate` / `channels` /
  `chunk_ms` / `max_duration_sec` / `volume` / `topic` / `status_topic`（见 §5.7）
- `edge.power.*`：`enabled` / `sample_interval_sec` / `status_rate_hz` /
  `status_topic` / `model_path` / `alarm_path` / `vcgencmd`（见 §5.8）
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
cd software/edge_server
PYTHONPATH=.:$PYTHONPATH python3 -m pytest tests -q
# 125 passed
```

单独跑健康检查（不依赖 ROS 图）：

```bash
PYTHONPATH=.:$PYTHONPATH python3 -m pytest tests/test_health_monitor.py tests/test_health_http.py -q
# 29 passed
```

### 7.2 VLA/安全端到端

终端 A：

```bash
source ~/rush_mog_ws/install/setup.bash
ros2 launch edge_server edge_server.launch.py
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
ros2 launch edge_server edge_server.launch.py \
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
4. **ICM-20948**：Pi I2C（`imu.enabled=true`）或 STM32 转发（旧 `base_bridge_node`）。
5. **EKF**：`robot_localization` 融合 `/odom` + `/imu/data`，确认 TF。
6. **SLAM**：`slam_stack.launch.py` + `ExecuteMission map`，保存地图。
7. **定位**：AMCL/map_server，设置初始位姿。
8. **导航/搜索**：Nav2 或直航；目标检测/搜索。
9. **安全**：硬件急停、INA226、悬崖/碰撞、看门狗。
10. **健康检查**：`robot_bringup.launch.py` 已启动 `health_monitor_node`；确认
    `curl http://127.0.0.1:8090/ros/health` 返回 `status=ok`，dashboard 经 Edge Server 读取。

硬件选型、接线、引脚冲突、标定与验收见 [`hardware/SLAM_NAV.md`](../../hardware/SLAM_NAV.md)。

---

## 9. 当前限制

1. **VLA 为规则桩**：未包含模型权重/GPU 推理；替换 `VLAEngine._load_model/predict` 即可。
2. **STM32 固件仍为 ASCII**：Pi Edge Server 已直连 ASCII 协议；`DESIGN.md §6.13` 的二进制协议尚未在固件实现。
3. **Nav2 参数起步值**：需实车调 `robot_radius`、速度、膨胀半径、DWB critics。
4. **无深度相机**：目标距离靠激光/bbox 高度；遮挡/高度变化精度有限。
5. **磁力计需校准**：ICM-20948 受电机干扰时只慢速校正，不硬切航向。
6. **单地图/单楼层**：未实现多地图切换/电梯/多层。
7. **动态目标**：搜索假设目标基本静止；移动目标需预测/重规划。
8. **颜色检测仅演示**：真实场景建议 YOLO/深度相机/Orin Nano。
9. **健康反馈缺口**：机械臂 `/arm/joint_states` 尚未发布，健康接口显示 `unknown`；
   音箱/麦克风已接入 `/body/audio_status`、`/body/mic_status`，蓝牙未连接时显示
   `degraded`，连接并支持 HFP 后恢复 `ok`。

---

## 10. 目录

```text
software/edge_server/          # ROS 包根目录
├── edge_server/               # Python 包
│   ├── config/                # config.yaml / robot.yaml / ekf.yaml / slam_toolbox.yaml / nav2_params.yaml
│   ├── core/                  # 纯算法，可单测（含 health_monitor.py）
│   ├── drivers/               # ICM-20948
│   ├── edge_control.py        # 按钮/摇杆/运动保活/STM32 遥测纯逻辑
│   ├── core/speaker.py        # 蓝牙音箱自动连接 + TTS/音频排队播放
│   ├── core/microphone.py     # HFP 麦克风录音 + WAV/ROS AudioData
│   ├── core/power.py          # 树莓派 vcgencmd/rpi_volt 电源监控
│   ├── edge_web.py            # Edge 网页/API/MJPEG（stdlib）
│   ├── http_health.py         # 健康检查 HTTP 网关（stdlib）
│   └── utils/
├── ros_nodes/                 # 16 个 ROS2 节点（含 edge_server_node / health_monitor_node）
├── web/                       # 手柄页、dashboard、静态资源
├── launch/                    # edge_server / robot_bringup / slam_stack / nav2_stack
├── tests/                     # 单元 + 仿真 + 联调（含 test_edge_*.py / test_health_*.py）
├── DESIGN.md
├── HEALTHCHECK.md             # 健康检查接口契约（HTTP + ROS）
├── README.md
├── package.xml
└── setup.py
```
