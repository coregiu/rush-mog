# 1 整体软件架构
> 统一接口文档（OpenAPI 3.0 / Swagger）：[api.yaml](api.yaml)，按 Edge Server、Center Server、ROS Health Gateway、Body Control (UART) 分组。
RushMog智能系统包括语音控制系统，车体控制系统，视觉分析系统，远程控制中心系统、激光雷达系统、VLA推理系统、机械臂系统、SLAM/导航/目标搜索系统等核心子系统。
- 语音控制系统，识别语音，将语音转换成执行指令，并将指令传给主控制系统，进行控制执行。(原计划采用asr-pro车载控制，但小车电机噪音较大，效果不好。改用终端控制。)
- 车体控制系统，STM32单板控制，接收语音指令、PS2手柄指令对小车行进、声音、机械臂进行控制执行。
- 远程控制中心系统，提供WIFI供终端连接，提供PS2手柄页面的WEB Server，接收摄像头监控图像推送到终端PS2页面上，同时提供模拟PS2手柄的各项指令操作。
- 激光雷达系统。由机载树莓派/Orin 接 2D 激光雷达，发布 `/scan`；`slam_toolbox` 建图，`AMCL` 定位。硬件方案见 [hardware/SLAM_NAV.md](../hardware/SLAM_NAV.md)。
- VLA推理系统。由 `software/center_server` 的 SmolVLA HTTP 服务执行真实策略推理；机载 ROS 负责采集、安全门控与执行，规则桩保留为降级路径。
- 机械臂系统。PCA9685 驱动，`/arm/trajectory` 接口；抓取/放置轨迹待 IK 接入。
- SLAM/导航/目标搜索系统。由 `software/ros` 的 center_server v2 实现：建图、定位健康度、Nav2/直航、目标检测、前沿探索、目标确认与接近。
- 健康检查系统。由 `software/ros` 的 `health_monitor_node` 实现：ROS 节点/话题 + 相机/雷达/IMU/麦克风/音箱/机械臂/底盘/电池等设备健康，HTTP 供 `edge_center` 调用。

![](architechture.png)

> **计算平台说明**：ESP32-S3 没有 Linux，不能运行 ROS2/slam_toolbox/Nav2，只做 Wi-Fi/图传/遥控桥。
> SLAM/导航/目标搜索必须在机载树莓派 4B/5 或 Jetson Orin Nano 上运行。



# 2 控制指令

| 语音命令      | 手柄命令              | 功能                         | 命令ID |    指示灯|
| ------------ | --------------------- | -------------------------- | ------ | ------  |
| 停/别走了     | 松开左侧所有按钮       | 小车停止                      |  0     |         |
| 前进/走      | ↑ 键，或左摇杆向上      | 小车前进                      |  1     |         |
| 后退         | ↓ 键，或左摇杆向下      | 小车后退                      |  2     |         |
| 往左走       | ← 键，或左摇杆向左      | 小车向左横向走                 |  3     |         |
| 往右走       | → 键，或左摇杆向右      | 小车向右横向走                 |  4     |         |
| 往左前走     | 左摇杆向左上            | 小车向左前方向斜着走            |  5     |         |
| 往右前走     | 左摇杆向右上            | 小车向右前方向斜着走            |  6     |         |
| 往左后走     | 左摇杆向左下            | 小车向左后方向斜着走            |  7     |         |
| 往右后走     | 左摇杆向右下            | 小车向右后方向斜着走            |  8     |         |
| 左转         | L1按钮                 | 小车向左转                     | 9     |         |
| 右转         | R1按钮                 | 小车向右转                     | A     |         |
| 左调头       | L2按钮                 | 小车从左侧调头                 | B     |         |
| 右调头       | R2按钮                 | 小车从右侧调头                 | C     |         |
| 测试小车     | START 键               | 测试小车                       | D     |         |
|             |  △ 键                 | 加速                           | E     |         |
|             |  × 键                  | 减速                           | F     |         |
|             |  □ 键                  | 向左微转                       | G     |         |
|             |  ○ 键                  | 向右微转                       | H     |         |
|             |  松开 □ 或 ○ 键        | 恢复直行                        | I     |         |
|             |  SELECT 键             | 小车重新初始化                  | J     |         |


# 3 子系统设计说明
## 3.1 语音控制子系统
原计划采用 asr-pro 车载语音控制；因电机噪声导致识别效果不佳，语音输入改由终端/手机页面完成，原 `audio_control` 代码已移除。

## 3.2 车体控制子系统

[车体控制系统](body_control/README.md)

车体控制子系统功能：
- 1 接收语言控制指令
- 2 驱动小车电机，四个电机分别驱动，能够实现前进/后退/停止等动作
- 3 与树莓派/摄像头交互，传递控制指令，接收图像分析结果
- 4 驱动机械臂运动控制执行

> 当前 STM32 固件仍使用 ASCII 命令协议，没有编码器/IMU 回传。
> `center_server` 已实现二进制协议主机侧（`base_protocol.py` / `base_bridge_node`），
> 要实现 Nav2 闭环，需要同步升级固件：4 路编码器 + 4 路 PWM + 轮速 PID + 协议帧。


## 3.3 远程控制子系统
- [端侧控制子系统](edge_server/README.md)
- [数字孪生监控页](edge_server/data/dashboard.html)（`/dashboard.html`）
- [ROS/CENTER 自动接入与上报](tools/README.md)

远程控制子系统功能：
- 1 连接摄像头，实时采集图像数据
- 2 应用机器学习进行图像识别
- 3 对位置、距离进行识别
- 4 形成控制指令传给主控制系统
- 5 模块注册/上报/代理 API，展示 EDGE/ROS/CENTER 状态并支持参数修改

## 3.4 中心推理子系统
- 机载 ROS 包（SLAM/导航/搜索/安全/健康检查）：[ros/README.md](ros/README.md)、[详细设计](ros/DESIGN.md)、[健康检查契约](ros/HEALTHCHECK.md)
- 中心 SmolVLA HTTP 推理服务：[center_server/README.md](center_server/README.md)、[服务设计](center_server/DESIGN.md)

中心推理子系统功能：
- 1 接收端侧/机载多模态数据（图像、激光、音频、里程计、IMU、地图、定位）
- 2 SmolVLA 推理，输出动作块、底盘速度和机械臂动作
- 3 SLAM 建图/地图存取、定位健康度、Nav2/直航导航
- 4 目标检测、前沿探索、目标确认与接近
- 5 多源速度仲裁、安全门控、反向操作
- 6 中心服务与机载 ROS 通过 HTTP 解耦，可分别部署和降级
- 7 节点/设备健康检查，HTTP `:8090/ros/health` 供 `edge_center` 调用

验证：ROS 包 64 项单元测试（含 29 项健康检查）、7/7 VLA/安全端到端、
6/6 SLAM/导航/搜索联调；`center_server` 服务 38 项单元/HTTP 测试（不依赖 GPU）。


# 4 使用的相关工具软件：

#### OS： Ubuntu 20.04

#### 软件设计开发工具:

- 软件设计工具： StarUML  https://staruml.io/download

