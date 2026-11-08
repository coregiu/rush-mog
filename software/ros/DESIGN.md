# center_server 详细设计（ROS2：VLA + SLAM + 导航 + 目标搜索）

> 版本：v2.0（2026-11）
> 对应硬件目标：树莓派 4B/5 或 Jetson Orin Nano + STM32 + 2D 激光 + ICM-20948 + 4 路编码器。
> 硬件缺口分析与选型见 [`hardware/SLAM_NAV.md`](../../hardware/SLAM_NAV.md)。

---

## 0. 版本与验证状态

| 项目 | 状态 |
|---|---|
| 代码 | `software/ros`（ament_python，ROS2 Humble） |
| 消息/动作 | `software/custom_msgs`（v0.2.0） |
| 构建 | `colcon build --packages-select custom_msgs center_server` 通过 |
| 单元测试 | `pytest tests -q`：64 passed |
| 健康检查 | `test_health_monitor.py` + `test_health_http.py`：29 passed；ROS/HTTP 手工联调通过 |
| VLA/安全端到端 | `end_to_end_check.py`：7/7 passed |
| SLAM/导航/搜索联调 | `slam_nav_check.py` + `fake_slam_robot.py`：6/6 passed |
| 真实 VLA 模型 | 未包含权重；规则桩，接口固定 |
| STM32 二进制协议固件 | 主机侧已实现；固件仍为 ASCII 命令，需按 §6.13 升级 |

---

## 1. 设计目标

`center_server` 是 Rush-MOG 的**决策与任务中心**，在 v1 的 VLA 推理/安全/反向操作之上，
增加五类能力：

1. **SLAM 建图**：消费 2D 激光与里程计，控制 slam_toolbox 建图、保存/加载地图、发布前沿。
2. **定位**：消费 ICM-20948、轮式里程计、AMCL/TF，输出定位健康度与 map 系位姿，门控导航。
3. **自主导航**：把“去某点/探索/回家”转成 Nav2 或内置直航的速度指令，带目标校验、停滞检测、超时。
4. **目标搜索**：相机检测 + 激光测距 + 地图投影 + 前沿探索 + 目标确认 + 接近，返回目标位姿。
5. **健康检查**：ROS 节点/话题 + 全部连接设备，HTTP 供 `edge_center` 调用。

设计原则：

- **分层**：任务层（mission）→ 导航层（nav）→ 速度层（mux）→ 执行器（STM32）。
- **单一速度出口**：所有来源经 `velocity_mux_node` 仲裁后只发布一个 `/cmd_vel`。
- **可降级**：没有 Nav2 用内置直航；没有 slam_toolbox 用内置 PGM/YAML 存图；没有深度相机用激光/bbox 估距。
- **可观测**：每个子系统都有状态话题，便于 HMI/日志/故障定位。
- **安全优先**：急停、障碍、数据超时、低电量、定位丢失都能覆盖普通任务。

---

## 2. 目标硬件与部署拓扑

### 2.1 三层

```text
┌──────────────────────────────────────────────────────────────┐
│ 云端 / PC（可选）                                             │
│   center_server：VLA、任务编排、HMI、日志、地图管理             │
└──────────────────────────┬───────────────────────────────────┘
                           │ DDS / Wi-Fi（建议 5GHz 或有线）
┌──────────────────────────▼───────────────────────────────────┐
│ 机载 Linux（Pi 4B/5 或 Orin Nano）                             │
│   ROS2 Humble：激光/相机驱动、base_bridge、EKF、slam_toolbox、  │
│   Nav2、center_server 的导航/搜索/定位节点                     │
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

### 2.2 部署选项

| 选项 | 车上 | 云端/PC | 说明 |
|---|---|---|---|
| A（推荐） | Pi 跑驱动/EKF/SLAM/Nav2/搜索 | VLA/任务编排可远程 | 实时闭环在车上，网络差也可用 |
| B | Orin Nano 跑全部 | 无 | 本地 VLA，成本高 |
| C | 仅 ESP32/STM32 | center_server 远程 | 仅演示；Wi-Fi 抖动会影响导航 |

代码对 A/B/C 相同；区别只是节点放在哪台机器、DDS 网络怎么配。

### 2.3 与 edge_server 的边界

`edge_server`（ESP32-S3）保留：

- Wi-Fi AP、网页手柄、MJPEG 图传；
- 遥控串口桥（ESP32 UART3 ↔ STM32）；
- 车身遥测桥（STM32 编码器 → ESP32 `/api/body`）；
- 运动命令桥（ROS `/cmd_vel` → ESP32 `/cmd` `raw:` → STM32）；
- 可选 OV2640 图传。

`center_server` 不依赖 ESP32 的算力。实车推荐：

- 激光、相机、IMU 直接接 Pi；
- 默认 Pi 通过 Wi-Fi HTTP 与 ESP32 通信（`esp32_bridge_node`）：
  `GET /api/body` 取编码器遥测，`POST /cmd raw:<cmd>` 下发电机命令；
- 仍保留 Pi 通过 UART 直连 STM32（`base_bridge_node`，`base_transport:=serial`）；
- ESP32 负责 AP、编码器遥测转发、运动命令转发、图传/遥控；
- 手机可连 ESP32 AP；Pi 也可连 ESP32 AP 使用 `192.168.4.1`。

---

## 3. 核心能力边界

### 3.1 输入

| 类型 | 话题 | 说明 |
|---|---|---|
| 激光 | `/scan` 或 `/edge/radar` | 2D LaserScan，10Hz+ |
| 图像 | `/camera/image_raw` 或 `/edge/video` | bgr8，目标检测/VLA |
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
| 健康 | `/center/health`、`/center/healthcheck`、HTTP `:8090/ros/health` | 系统/设备健康，供 edge_center |
| 位姿 | `/center/pose` | map 系，供导航/搜索/目标投影 |

---

## 4. 软件结构

```text
software/
├── custom_msgs/                      # 消息/服务/Action（v0.2）
│   ├── msg/*.msg
│   ├── srv/*.srv
│   └── action/*.action
└── ros/
    ├── center_server/
    │   ├── config/
    │   │   ├── config.yaml           # 仿真/桌面默认
    │   │   ├── robot.yaml            # 实车覆盖
    │   │   ├── ekf.yaml              # robot_localization
    │   │   ├── slam_toolbox.yaml
    │   │   └── nav2_params.yaml
    │   ├── core/
    │   │   ├── action_planner.py
    │   │   ├── base_protocol.py      # STM32 二进制帧/CRC
    │   │   ├── data_preprocessor.py
    │   │   ├── health_monitor.py      # 设备/节点健康状态机（纯逻辑）
    │   │   ├── imu_monitor.py        # ICM-20948 健康度/航向
    │   │   ├── instruction_parser.py # 文本 -> 任务
    │   │   ├── nav_supervisor.py     # 目标校验/停滞/直航
    │   │   ├── object_detector.py    # 颜色/YOLO
    │   │   ├── object_search.py      # 探索/搜索状态机
    │   │   ├── occupancy_map.py      # 栅格/前沿/PGM+YAML
    │   │   ├── odometry.py           # 麦轮运动学/里程计
    │   │   ├── pose_utils.py
    │   │   ├── reverse_handler.py
    │   │   ├── safety_checker.py
    │   │   ├── state_machine.py
    │   │   ├── target_localizer.py   # bbox/激光 -> map 坐标
    │   │   ├── target_tracker.py
    │   │   ├── velocity_mux.py
    │   │   └── vla_engine.py
    │   ├── http_health.py            # 健康检查 HTTP 网关（stdlib）
    │   ├── drivers/
    │   │   └── icm20948.py           # 寄存器级 I2C 驱动
    │   └── utils/
    ├── ros_nodes/                    # 15 个节点
    ├── launch/
    │   ├── center_server.launch.py
    │   ├── robot_bringup.launch.py
    │   ├── slam_stack.launch.py
    │   └── nav2_stack.launch.py
    ├── tests/
    │   ├── test_core.py
    │   ├── test_vla_engine.py
    │   ├── test_slam_nav_core.py
    │   ├── fake_edge_publisher.py
    │   ├── fake_slam_robot.py
    │   ├── end_to_end_check.py
    │   └── slam_nav_check.py
    ├── DESIGN.md
    └── README.md
```

---

## 5. 节点总览

| # | 节点 | 职责 | 关键输入/输出 |
|---|---|---|---|
| 1 | `center_bridge_node` | 图像/激光/里程计软同步，缓存音频/关节/IMU | → `/center/synced_data`、`/center/robot_state` |
| 2 | `vla_inference_node` | 多模态预处理 + VLA 规则桩 | → `/center/action_intent`、`/center/inference_result` |
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
| 13 | `esp32_bridge_node` | Pi ↔ ESP32 HTTP、`/api/body` 遥测、`/cmd` 运动 | ↔ `/odom`、`/joint_states`、`/base/status` |
| 13b | `base_bridge_node` | 旧版 Pi ↔ STM32 串口、里程计、电池、IMU 转发 | ↔ `/odom`、`/imu/data`、`/battery_state` |
| 14 | `imu_node` | Pi I2C 直连 ICM-20948 | → `/imu/data` |
| 15 | `health_monitor_node` | 节点/设备健康、HTTP 网关 | → `/center/health`、`/center/healthcheck`、:8090 |

启动：

```bash
# 仿真/桌面（默认 config.yaml）
ros2 launch center_server center_server.launch.py

# 实车（robot.yaml + 底盘串口）
ros2 launch center_server robot_bringup.launch.py

# 可选外部栈
ros2 launch center_server slam_stack.launch.py
ros2 launch center_server nav2_stack.launch.py map:=/path/to/map.yaml
```

---

## 6. 模块详细设计

### 6.1 通信与软同步（`center_bridge_node`）

v1 用 5 路严格/近似同步，真实机器人没有麦克风时会阻塞整条链路。v2 改为：

- `image + laser + odom` 三路 `ApproximateTimeSynchronizer`（窗口 100–120 ms）；
- `audio / joint_states / imu` 采用最近值缓存，缺失时填空消息；
- 以图像帧为主参考，发布 `SyncedSensorData`；
- `RobotState` 单独发布 odom + joint。

QoS：

- 传感器流 `SensorDataQoS`（best effort），兼容真实驱动；
- `/center/status`、`/center/safety_state` 用 TRANSIENT_LOCAL，后启动节点也能拿到最后状态；
- `/map` 发布 TRANSIENT_LOCAL，订阅 VOLATILE（兼容 slam_toolbox 与 map_server）。

### 6.2 多模态预处理（`data_preprocessor.py`）

- 图像：BGR/RGB/gray → 224×224 → float32 CHW [0,1]；
- 激光：`LaserScan.ranges` → float32 一维；
- 音频：PCM s16le → log-mel（numpy 实现，避免 librosa 依赖）；
- 状态：odom `x/y/yaw`、关节字典；
- 打包为 `SyncedDTO` 给 VLA。

### 6.3 VLA 推理（`vla_engine.py`）

- 规则桩：停止/前进/后退/左转/右转/左移/右移；
- 任务意图：建图/探索/搜索/导航/回家；
- `predict(data) -> ActionIntentDTO`，字段：
  `task_id, target, intent, value, confidence, reverse_request, task_type, object_label, goal_x/y/yaw, waypoint`；
- 真实模型替换 `_load_model()` 与 `predict()` 即可，接口不变；
- 置信度 < `confidence_threshold` → `needs_confirmation` → `PAUSED`。

### 6.4 指令解析与任务编排（`instruction_parser.py` + `task_manager_node`）

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

### 6.5 SLAM 与地图（`map_manager_node`）

- 订阅 `/map`（slam_toolbox/cartographer/map_server 均可）；
- 每秒发布 `MapInfo`（分辨率、尺寸、原点、known_ratio、frontier_count、map_file）；
- 提取 frontier：free 与 unknown 的 8 邻域边界，BFS 聚类，按 `size/(distance+0.5)` 选点；
  大图自动下采样（`frontier_max_cells`）；
- `MapControl` 服务：
  - `start`：进入建图状态；
  - `stop`：停止建图；
  - `save`：优先 slam_toolbox `SaveMap`，失败回退内置 PGM+YAML；
  - `load`：读 PGM+YAML，发布 `/map` 供 AMCL/Nav2；
  - `reset`：清空并调用 slam_toolbox `Reset`（若可用）。
- PGM/YAML 与 map_server 兼容：occupied=0、free=254、unknown=205；YAML 含 resolution/origin/thresholds。
- 地图目录：`map_manager.map_dir`（默认 `~/.ros/rush_mog/maps`，实车 `~/rush_mog/maps`）。

### 6.6 定位与 ICM-20948（`localization_node` + `imu_node` + EKF）

#### 6.6.1 坐标树

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

#### 6.6.2 EKF 配置（`config/ekf.yaml`）

- `two_d_mode: true`，`world_frame: odom`；
- `odom0`：`x,y,yaw,vx,vy,vyaw`；
- `imu0`：`yaw,vyaw`，`imu0_relative: true`；
- 磁力计不直接融合；由 `ImuMonitor` 判断 `mag_ok` 后做慢速互补。

#### 6.6.3 ICM-20948 驱动（`drivers/icm20948.py`）

- I2C 地址 0x68/0x69，WHO_AM_I=0xEA；
- Bank2 配置：GYRO_CONFIG_1/ACCEL_CONFIG、采样率、DLPF；
- Bank0 读 0x2D 起 14 字节（accel+gyro+temp）；
- AK09916 通过 `INT_PIN_CFG.BYPASS_EN` 直连 0x0C；CNTL2=0x08（100Hz）；
- 单位：accel m/s²、gyro rad/s、mag µT、temp °C；
- 校准：`accel_bias/gyro_bias/mag_bias/mag_scale`；
- 输出 `sensor_msgs/Imu`，`orientation_covariance[0] = -1`（交给 EKF）。

#### 6.6.4 定位健康度

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

#### 6.6.5 磁力计使用原则

- 电机/电池大电流会让磁力计失真；`mag_ok=false` 时不参与校正；
- 只用慢速互补，不硬切 yaw；
- 校准硬铁/软铁；安装远离大电流线；
- ICM-20948 已 NRND，可用 ICM-42688-P + 磁力计替代。

### 6.7 导航（`navigation_node` + `nav_supervisor.py`）

Action：`/center/navigate_to`（`NavigateTo`）。

后端：

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
- Nav2 最终速度必须重映射到 `/nav/cmd_vel`（`launch/nav2_stack.launch.py` 用 `SetRemap`），
  否则会和 mux 抢 `/cmd_vel`；
- `nav2_params.yaml` 为麦轮/2D 激光起步配置，需实车调参。

### 6.8 目标检测与定位（`target_detector_node`）

流程：

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

### 6.9 目标搜索与自主探索（`object_search_node` + `object_search.py`）

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

### 6.10 速度仲裁（`velocity_mux_node`）

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

### 6.11 安全（`safety_manager_node`）

检查项：

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

### 6.12 反向操作（`reverse_operation_node`）

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

### 6.13 底盘串口桥（`base_bridge_node` + `base_protocol.py`）

帧格式：

```text
magic(0xAA55) | version(1) | type(1) | seq(1) | len(1) | payload(len) | crc16(2)
crc16: CRC-16/CCITT-FALSE，覆盖 version..payload
```

| type | 方向 | payload |
|---|---|---|
| 0x01 CMD_VEL | Pi→STM32 | `vx(mm/s) i16, wz(mrad/s) i16, flags u8` |
| 0x02 MODE | Pi→STM32 | `mode u8, flags u8` |
| 0x03 PING | Pi→STM32 | 空 |
| 0x81 TELEMETRY | STM32→Pi | `seq u32, enc[4] i32, wheel[4] i16 mm/s, battery_mv u16, current_ma i16, flags u8, uptime_ms u32` |
| 0x82 IMU | STM32→Pi | `seq u32, accel(mg)[3] i16, gyro(0.1dps)[3] i16, mag(0.1uT)[3] i16, stamp_us u32` |
| 0x83 ACK | STM32→Pi | `ack_type u8` |
| 0x84 STATUS | STM32→Pi | `status u8` |

flags：bit0 ESTOP、bit1 ARMED、bit2 IMU_OK、bit3 ENCODER_OK、bit4 BATTERY_LOW。

主机侧：

- 30Hz 下发 `CMD_VEL`，0.5s 无新命令回零；
- 读遥测 → 麦轮运动学 → `/odom`、`/joint_states`、`/battery_state`；
- 读 IMU → `/imu/data`；
- 串口断开 2s 重连；
- 状态 JSON 发 `/base/status`。

#### ESP32 HTTP 模式（当前实车默认）

Body Control 已通过 USART3 上报四轮编码器遥测；ESP32 解析后在 `/api/body`
和 `/api/modules` 的 `edge.body` 暴露。`esp32_bridge_node`：

- 10Hz 轮询 `/api/body`，用麦轮正运动学积分 `/odom`，20Hz 发布；
- 发布 `/joint_states` 和 `/base/status`（`encoder_ok`、`distance_mm`、轮速）；
- 订阅 `/cmd_vel`，把 Twist 量化为 8 方向 + 1~9 档 PWM，或 9/A 原地转向；
- 通过 `POST /cmd` 的 `raw:<cmd>` 下发，ESP32 保留 PWM 第二字节并每 200ms 重发；
- raw 命令有 1s 租约，ROS 停止续租后 ESP32 自动停车；
- `/cmd_vel` 0.5s 超时、急停、车身遥测 1s 超时都会停车。

注意：Body Control 的接口是离散方向 + 9 档 PWM，不是连续左右轮差速；
`twist_mode: priority` 会优先平移或转向的较大分量，`time_share` 用时间片近似弧线。
需要平滑弧线时应升级体控固件，增加左右 PWM/四轮直接控制命令。

旧串口二进制协议（上表）仍保留在 `base_protocol.py`/`base_bridge_node.py`；
当前 Body Control ASCII 固件不支持该二进制帧，实车默认不要再启用 `base_transport:=serial`。

### 6.14 配置与日志

- 配置优先级：`config_file` 参数 > `CENTER_SERVER_CONFIG` 环境变量 > 安装目录 `config.yaml`；
- 所有节点共享 `center_server:` 顶层命名空间；
- 日志用 `rclpy`/stdlib logging，不依赖 loguru；
- 提供 `config.yaml`（仿真）与 `robot.yaml`（实车）；
- `ekf.yaml`/`slam_toolbox.yaml`/`nav2_params.yaml` 为外部栈配置。

### 6.15 健康检查（`health_monitor_node` + `health_monitor.py`）

目标：给 `edge_center` 一个只读接口，回答“ROS/设备是否健康、能否继续接任务”。

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

状态机：

- 设备：`ok` / `degraded` / `error` / `unknown` / `disabled`；
- 整体：`ok` / `starting` / `degraded` / `error` / `disabled`；
- 判定优先级：必需设备 error > 期望节点缺失 > 安全状态超时 > 宽限期 > 设备 degraded >
  可选设备 error > 急停 > `strict_unknown` > ok；
- `ready = (ok|degraded) && !emergency_stop`；
- 启动宽限期避免“节点刚启动还没来得及发布”被误报。

关键设计：

1. **纯逻辑与 ROS 解耦**：`core/health_monitor.py` 不 import rclpy，29 项单测覆盖状态机；
2. **配置驱动**：设备 topic/message_type/频率/超时/required 全部在 YAML，新增设备无需改代码；
3. **诚实报告 unknown**：音箱/机械臂当前无反馈话题，报 `unknown`，不伪造 `ok`；
4. **HTTP 线程安全**：节点按 1 Hz 预计算 JSON 快照，HTTP 线程只读快照，不调用 rclpy；
5. **失败降级**：HTTP 端口占用只记录 `http.bound=false`，ROS 话题/服务仍工作；单次快照异常
   不退出节点；
6. **默认安全**：代码默认监听 `127.0.0.1`；dashboard 部署配置为 `0.0.0.0`，建议启用 `api_key` 并限制 AP 内网；
7. **晚加入兼容**：健康话题/安全状态均用 TRANSIENT_LOCAL，后启动的 HMI 也能拿到最后一帧；
8. **参数可调**：`/ros/params` 暴露健康频率/宽限期/设备频率/超时/required，
   修改在运行时生效，重启回 YAML。

接口契约、JSON 字段、设备反馈协议和 `edge_center` 接入示例见
[`HEALTHCHECK.md`](HEALTHCHECK.md)。

---

## 7. 数据流

### 7.1 VLA 单步

```text
edge/Pi sensors → bridge_node → /center/synced_data
    → vla_inference_node → /center/action_intent
    → action_generator_node → /cmd_vel_raw
    → velocity_mux_node → /cmd_vel → base_bridge_node → STM32
```

### 7.2 建图

```text
/scan + /odom + /imu/data → slam_toolbox → /map
    → map_manager_node → /center/map_info + /center/frontiers
    → task_manager (map) → object_search (explore) → navigation_node → /nav/cmd_vel
    → mux → /cmd_vel → STM32
```

### 7.3 目标搜索

```text
/camera/image_raw + /scan + /center/pose
    → target_detector_node → /center/target_detections
    → object_search_node (SearchCoordinator)
        ├─ /center/frontiers + /center/map_info → 探索目标
        ├─ /center/navigate_to → navigation_node → /nav/cmd_vel
        └─ 确认目标 → 接近 → SearchObject.Result(target_pose)
    → task_manager → ExecuteTask/ExecuteMission.Result
```

### 7.4 急停

```text
急停按钮/服务/障碍/低电量/定位丢失
    → safety_manager_node → /center/safety_state + /safety/cmd_vel_override
    → velocity_mux_node → /cmd_vel=0
    → task_manager/action_generator 门控
```

---

## 8. 接口设计

### 8.1 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/scan`、`/edge/radar` | `sensor_msgs/LaserScan` | 订阅 | 激光 |
| `/camera/image_raw`、`/edge/video` | `sensor_msgs/Image` | 订阅 | 图像 |
| `/edge/audio` | `custom_msgs/AudioData` | 订阅 | 音频（可选） |
| `/odom`、`/robot/odometry` | `nav_msgs/Odometry` | 订阅 | 里程计 |
| `/imu/data` | `sensor_msgs/Imu` | 订阅 | IMU |
| `/joint_states` | `sensor_msgs/JointState` | 订阅 | 关节 |
| `/battery_state` | `sensor_msgs/BatteryState` | 订阅 | 电池 |
| `/map` | `nav_msgs/OccupancyGrid` | 订阅/发布 | 地图 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 订阅 | AMCL |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 发布 | 初始位姿 |
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
| `/base/status` | `std_msgs/String` | 发布/订阅 | STM32 遥测 JSON（底盘/健康） |
| `/arm/joint_states` | `sensor_msgs/JointState` | 订阅 | 机械臂回读（待接入） |
| `/body/audio_status` | `std_msgs/String` | 订阅 | 音箱状态 JSON（待接入） |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 最终速度 |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | 发布 | VLA 原始 |
| `/nav/cmd_vel` | `geometry_msgs/Twist` | 发布 | 导航速度 |
| `/reverse/cmd_vel` | `geometry_msgs/Twist` | 发布 | 反向速度 |
| `/safety/cmd_vel_override` | `geometry_msgs/Twist` | 发布 | 安全覆盖 |
| `/arm/trajectory` | `trajectory_msgs/JointTrajectory` | 发布 | 机械臂 |
| `/task/request` | `custom_msgs/TaskRequest` | 发布/订阅 | 内部任务 |
| `/center/mission_request` | `custom_msgs/TaskRequest` | 订阅 | 外部任务 |

### 8.2 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `/center/set_mode` | `custom_msgs/SetMode` | auto/manual/reverse |
| `/center/emergency_stop` | `custom_msgs/EmergencyStop` | 急停/解除 |
| `/center/map_control` | `custom_msgs/MapControl` | start/stop/save/load/reset |
| `/center/set_initial_pose` | `custom_msgs/SetInitialPose` | 设置初始位姿 |
| `/center/healthcheck` | `custom_msgs/HealthCheck` | 全部/指定设备健康快照 |

### 8.3 Action

| Action | Goal | Result |
|---|---|---|
| `/center/execute_task` | task_id, instruction, task_type, target_* | success, info, target_pose, approach_pose, confidence, elapsed |
| `/center/execute_mission` | mission_id, mission_type, target_*, max_duration, save_map, target_coverage | success, info, target_pose, approach_pose, confidence, elapsed |
| `/center/navigate_to` | target_pose, tolerance, timeout, goal_id, avoid_obstacles | success, info, final_pose, elapsed |
| `/center/explore` | mission_id, max_duration, target_coverage | success, info, coverage, elapsed |
| `/center/search_object` | task_id, target_label, min_confidence, max_duration, standoff, explore_if_not_found, save_map | found, info, target_pose, approach_pose, confidence, elapsed |

### 8.4 消息

`AudioData`、`SyncedSensorData`、`RobotState`、`TaskRequest`、`ActionIntent`、`ReverseOperation`、
`InferenceResult`、`SafetyState`、`SystemStatus`、`MapInfo`、`Frontier`、`FrontierList`、
`TargetDetection`、`TargetList`、`LocalizationStatus`、`NavStatus`、`SearchStatus`、
`MissionStatus`、`VelocityMuxStatus`、`DeviceHealth`、`HealthStatus`。

### 8.5 健康检查接口

对外主接口是 HTTP（供 `edge_center`），ROS 原生接口同时保留：

| 接口 | 类型 | 说明 |
|---|---|---|
| `GET /ros/health` | HTTP | 详细健康快照（别名 `/v1/health`、`/health`、`/healthcheck`） |
| `GET /ros/health/devices/{id}` | HTTP | 单设备健康 |
| `GET/POST /ros/params` | HTTP | 健康监控参数查看/运行时修改 |
| `GET /ros/healthz`、`GET /ros/readyz` | HTTP | 存活/就绪探针（readyz 503 表示未就绪） |
| `/center/health` | `custom_msgs/HealthStatus` | 1 Hz，TRANSIENT_LOCAL |
| `/center/healthcheck` | `custom_msgs/HealthCheck` | 全部/指定设备，支持 `include_metrics` |

整体状态：`ok` / `starting` / `degraded` / `error` / `disabled`；
设备状态：`ok` / `degraded` / `error` / `unknown` / `disabled`。
代码默认监听 `127.0.0.1:8090`；仓库配置为 `0.0.0.0` 供 dashboard 经 ESP32 代理访问，跨机建议设置 `api_key`。
完整字段、设备清单、反馈协议和 `edge_center` 接入示例见
[`HEALTHCHECK.md`](HEALTHCHECK.md)。

---

## 9. 配置设计

### 9.1 文件

| 文件 | 用途 |
|---|---|
| `config/config.yaml` | 仿真/桌面默认；`/edge/*` 话题、`perception.enabled=false`、`base.enabled=false` |
| `config/robot.yaml` | 实车覆盖；`/scan`、`/camera/image_raw`、`base.enabled=true`、目标颜色 |
| `config/ekf.yaml` | robot_localization |
| `config/slam_toolbox.yaml` | slam_toolbox |
| `config/nav2_params.yaml` | Nav2 Humble 起步配置 |

### 9.2 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `vla_engine.inference_rate` | 10 | 推理频率 |
| `vla_engine.confidence_threshold` | 0.6 | 低置信度确认 |
| `safety.obstacle_stop_distance` | 0.5/0.35 | 障碍急停距离 |
| `safety.data_timeout_sec` | 2.0 | 传感器超时 |
| `safety.battery_low_percent` | 15 | 低电量 |
| `localization.min_quality` | 0.35 | 导航定位门限 |
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
| `health.http.host/port` | 127.0.0.1/8090 | HTTP 网关绑定 |
| `health.devices.<id>.required` | 按设备 | 缺失是否 error |

---

## 10. 异常处理

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
| 设备掉线 | 健康检查超时/无发布者 | `HealthStatus` 报 `error/degraded`，`edge_center` 禁能力 |
| 设备无反馈 | 可选设备从未收到数据 | 报 `unknown`，不伪造 `ok`；`strict_unknown` 可升级为 degraded |
| 期望节点缺失 | ROS graph 发现 | 宽限期后整体 `error`，`ros.missing_nodes` 给出名单 |
| 健康 HTTP 端口占用 | bind 异常 | ROS 话题/服务继续，`http.bound=false`，不影响运动控制 |

---

## 11. 实时性与安全

- **速度闭环在车上**：`velocity_mux` + `base_bridge` + STM32 看门狗；远程 center_server 断线时底盘自动回零。
- **急停优先级最高**：安全覆盖 > 反向 > 遥控 > 导航 > VLA。
- **超时**：每个速度源 0.5–0.6s 超时；STM32 自 0.5s 看门狗。
- **QoS**：状态用 TRANSIENT_LOCAL；传感器用 best effort；地图发布 TRANSIENT_LOCAL。
- **不要在 Wi-Fi 上跑底盘闭环**：Nav2 建议在 Pi/Orin 上运行，center_server 远程只做高层任务。
- **硬件急停**：软件之外必须有常闭急停切断电机驱动。

---

## 12. 部署与启动顺序

### 12.1 实车

1. 上电，确认 12V/5V/6V、共地、急停释放。
2. 启动激光驱动：`ros2 run <lidar_driver> ...`，确认 `/scan` 10Hz。
3. 启动相机：`/camera/image_raw`。
4. 启动底盘桥：
   ```bash
   ros2 run center_server base_bridge_node --ros-args \
     --params-file <install>/share/center_server/config/robot.yaml
   ```
   确认 `/odom`、`/imu/data`、`/battery_state`。
5. 启动 EKF：
   ```bash
   ros2 run robot_localization ekf_node --ros-args \
     --params-file <install>/share/center_server/config/ekf.yaml
   ```
6. 建图：
   ```bash
   ros2 launch center_server slam_stack.launch.py
   ros2 launch center_server robot_bringup.launch.py
   ros2 action send_goal /center/execute_mission custom_msgs/action/ExecuteMission \
     "{mission_id: map1, mission_type: map, target_coverage: 0.9, save_map: true}"
   ```
7. 定位：停 slam_toolbox，启动 Nav2 localization（map_server+AMCL）：
   ```bash
   ros2 launch center_server nav2_stack.launch.py map:=$HOME/rush_mog/maps/rush_mog.yaml
   ```
8. 导航/搜索：
   ```bash
   ros2 action send_goal /center/navigate_to custom_msgs/action/NavigateTo \
     "{target_pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}, goal_id: g1}"
   ros2 action send_goal /center/execute_task custom_msgs/action/ExecuteTask \
     "{task_id: find_cup, instruction: 找杯子}"
   ```
9. 健康检查（`robot_bringup.launch.py` 已包含 `health_monitor_node`）：
   ```bash
   curl -s http://127.0.0.1:8090/ros/health | python3 -m json.tool
   curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/ros/readyz
   ```
   跨机 `edge_center` 调用时，把 `health.http.host` 改为 `0.0.0.0` 并设置 `health.http.api_key`。

### 12.2 仿真/桌面

```bash
ros2 launch center_server center_server.launch.py
python3 tests/fake_edge_publisher.py
python3 tests/end_to_end_check.py       # 7/7
```

SLAM/导航/搜索联调：

```bash
ros2 launch center_server center_server.launch.py \
  config_file:=<repo>/software/ros/tests/config_test.yaml \
  start_perception:=false start_localization:=false
python3 tests/fake_slam_robot.py
python3 tests/slam_nav_check.py          # 6/6
```

> 注意：`fake_slam_robot` 自己发布 `/center/localization_status`，因此联调时关闭
> `localization_node`，避免两个源覆盖质量值。

---

## 13. 性能指标

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

## 14. 测试策略与结果

### 14.1 单元测试

> WSL/容器/沙箱中若 DDS 发现不稳定，测试和节点都加：
> `ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。

```bash
cd software/ros
PYTHONPATH=.:$PYTHONPATH python3 -m pytest tests -q
# 64 passed
```

健康检查专项：

```bash
PYTHONPATH=.:$PYTHONPATH python3 -m pytest \
  tests/test_health_monitor.py tests/test_health_http.py -q
# 29 passed
```

覆盖：

- 状态机、安全校验、动作规划、预处理、VLA、反向操作（v1）；
- 指令解析、栅格/前沿/存图、相机/激光定位、跟踪、搜索协调器、导航校验/直航、
  IMU 监控/航向、速度仲裁、麦轮里程计、串口协议 CRC/分帧、ICM-20948 驱动（v2）；
- 设备健康状态机、启动宽限期、频率/超时/非法数据、节点缺失、急停、设备过滤、
  HTTP 存活/就绪/鉴权/单设备/metrics 开关（healthcheck）。

### 14.2 VLA/安全端到端

```bash
python3 tests/end_to_end_check.py
# RESULT: 7/7 passed
```

覆盖：前进、左转、低置信度、手动模式、急停、障碍物、反向回退。

### 14.3 SLAM/导航/搜索联调

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

---

## 15. 当前限制与后续工作

1. **VLA 仍是规则桩**：无模型权重；接入真实 VLA 时替换 `_load_model/predict`。
2. **STM32 固件未升级**：主机侧二进制协议已实现，固件仍 ASCII；需按 §6.13 实现编码器/IMU 回传。
3. **Nav2 参数是起步值**：`robot_radius`、速度、膨胀半径、DWB critics 需实车调参。
4. **无深度相机**：目标距离靠激光/bbox 高度；遮挡/高度变化时精度有限。
5. **ICM-20948 磁力计需实测校准**：硬铁/软铁/安装偏航；电机干扰下只慢速校正。
6. **单地图/单楼层**：未实现多地图切换/电梯/多层。
7. **动态目标**：搜索假设目标基本静止；移动目标跟踪需扩展预测/重规划。
8. **目标检测模型**：颜色检测只适合演示；YOLO 需模型/算力；可接 Orin Nano。
9. **地图保存回退**：slam_toolbox 不可用时 PGM/YAML 精度取决于当前 `/map`。
10. **未做硬件在环**：`fake_slam_robot` 验证了软件链路，真实串口/IMU/激光需实车联调。
11. **健康反馈缺口**：音箱 `/body/audio_status`、机械臂 `/arm/joint_states` 尚未发布，
    健康接口报 `unknown`；接入后无需改 `health_monitor_node`，只改配置。
12. **健康 HTTP 无 TLS/限流**：默认仅回环；跨机需 `api_key` + 防火墙/反向代理。

后续优先级：

1. STM32 固件：4 路编码器 + 4 路 PWM + 轮速 PID + 二进制协议；
2. 实车 TF/标定：轮半径/CPR、IMU 安装、激光/相机外参；
3. Nav2 调参 + 真实地图闭环；
4. YOLO/深度相机接入；
5. 多地图/动态目标/远程 HMI。

---

## 16. 总结

`center_server` v2 在 v1 的 VLA 推理、安全、反向操作之上，补齐了 **SLAM 建图、ICM-20948 定位、
Nav2/直航导航、目标检测/搜索/接近、速度仲裁、底盘串口桥、健康检查**。软件链路已通过
64 项单元测试（含 29 项健康检查）、7 项 VLA/安全端到端、6 项 SLAM/导航/搜索联调；
实车落地还差 STM32 固件升级、硬件选型（见 `hardware/SLAM_NAV.md`）、现场标定，以及
音箱/机械臂反馈话题接入。
