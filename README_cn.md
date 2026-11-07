# Rush Mog

[English Version](README.md)

Rush Mog是一款致敬Unimog的VLA控制的智能万向轮小车。基于树莓派 + 深度相机 + esp32 + stm32f103 + L298N + IMU传感器 + 激光雷达 + 机械臂 + VLA推理，实现支持手机WIFI的PS2模拟器、语音控制和VLA智能控制三种模式，并支持机械臂操作的智能小车。同时提供动作编程，可自定义组合效果的动作指令。

## 功能说明
#### （1）手机模拟PS2控制小车行驶
可通过手机模拟PS2控制小车前进、后退、左转、右转、向左横向行驶，向右横向行驶，向左前、右前、左后、右后方向行驶。

#### （2）摄像头监控及自动识别物体
通过车载摄像头监控和自动识别、分类物体。并将画面实时在手机模拟的PS2控制器上查看。

#### （3）SLAM建图、自主导航与自动寻找目标
通过 2D 激光 + 轮式编码器 + ICM-20948 建立占据栅格地图，使用 AMCL/EKF 定位；通过 Nav2 或内置直航导航到目标点；通过相机检测 + 激光测距 + 前沿探索搜索目标物体，返回目标位姿并保持设定距离接近。支持地图保存/加载、建图任务、探索任务和目标搜索任务。

#### （4）搬运物体
使用VLA模型控制车体移动，可以根据目标物体进行搜索、锁定，并对移动目标物体进行跟踪。使用机械臂对物体进行夹起、转移、放下到指定范围区域等动作，实现物体搬运。搬运路径可规划也可自动学习。


## 设计说明
#### 总体系统架构
![](images/system-arch.png)

**核心模块：**
| 模块 | 位置 | 功能 |
| --- | --- | --- |
| **Edge Server** | 车载模块 | 基于 ESP32 的上位机：支持模拟 PS2 手柄操作小车、整车配置、状态监控；同时提供供 Terminal 访问的上位机页面。未部署 ROS 时，可直接由它手动控制小车。 |
| **ROS** | 车载模块 | 基于树莓派 4B，连接各传感器，提供基于 ROS2 平台的各项业务功能；无 ROS 时降级为通过 Edge Server 手动控制。 |
| **BodyControl** | 车载模块 | 基于 STM32F103C8T6，连接 L298N 驱动小车 JGB37-520 电机，并连接电机编码器实现 EKF 位姿估计。 |
| **Union Power Supply** | 车载模块 | 提供 12V / 5V / 3.3V 稳定电压源；纯硬件，无软件模块。 |
| **Center Server** | 中心服务器模块 | 提供 VLA 推理能力，为整车提供高层决策推理算力。 |
| **Terminal** | 用户使用终端 | 可以是手机、PAD 或 PC 机；通过浏览器打开 Edge Server 提供的上位机页面进行操作。 |


**模块上下文及API：**
| 接口 | 协议 | 客户端 | 服务端 | 功能 |
| --- | --- | --- | --- | --- |
| **上位机接口** | http | Terminal / ROS | Edge Server | 模拟 PS2 手柄操作小车、获取小车速度与里程状态；整车配置、状态监控。小车对外统一控制入口。 |
| **车辆控制接口** | uart | Edge Server | BodyControl | 小车运动命令执行；小车速度、里程状态获取。 |
| **Center Server 接口** | http | ROS | Center Server | 提供 VLA 推理执行。 |
| **ROS 执行接口** | http | Terminal | ROS | ROS 指令接收与执行；传感器信息获取。 |
| **ROS 健康检查接口** | http | edge_center | ROS | 监控 ROS 节点及全部连接设备（相机、雷达、IMU、麦克风、音箱、机械臂、底盘、电池），见 [HEALTHCHECK.md](software/ros/HEALTHCHECK.md)。 |

#### 硬件架构设计
- DIY小车，使用97mm万向轮加JGB37-520电机（带霍尔编码器），L298N驱动板(四驱的，或者用两个两驱的)。导航目标建议换四路独立 PWM 驱动（TB6612/DRV8871/BTS7960）。
- 车体控制使用STM32F103C8T6单板；增加编码器/PWM 余量建议升级 F103RCT6/F411。
- WIFI及图传使用ESP32-S3。
- SLAM/导航使用 2D 激光雷达（LD19/LD06 或 RPLIDAR A1）+ ICM-20948 九轴 IMU，接机载 Linux 计算平台（IMU 也可接 STM32）。
- 车上必须有树莓派 4B/5 或 Jetson Orin Nano 才能跑 ROS2/slam_toolbox/Nav2。

![](hardware/integrate_archetecture.png)

详细参照：[硬件设计说明](hardware/README.md)、[SLAM/导航硬件方案](hardware/SLAM_NAV.md)

#### 软件架构设计
- 软件分车体控制子系统和中心推理/导航子系统。
- 车体控制子系统，由STM32单板承载，驱动小车电机、机械臂、外放，接机载计算机/桥接命令进行执行。
- 机载 Linux 子系统运行 ROS2 驱动、`base_bridge_node`、ICM-20948、EKF、slam_toolbox、Nav2、目标检测和目标搜索。
- `software/ros` 的 center_server v2 增加 SLAM 地图管理、定位健康度门控、自主导航、目标检测/搜索/接近、速度仲裁和 STM32 串口桥。详见 [ROS 包设计](software/ros/DESIGN.md)。
- `health_monitor_node` 通过 HTTP（`:8090/ros/health`）向 `edge_center` 提供 ROS 节点与全部设备健康，同时保留 ROS 话题/服务；见 [健康检查契约](software/ros/HEALTHCHECK.md)。
- 独立部署的 [Center Server SmolVLA 推理服务](software/center_server/README.md) 通过 HTTP 为 ROS 提供 VLA 推理；接口见 [服务设计](software/center_server/DESIGN.md)。

![](software/architechture.png)

详细参照：[软件设计说明和代码](software/README.md)

## 手柄使用办法
![](software/edge_server/ps2.png)

启动小车后，使用手机连接RUSH-MOG WIFI。连接成功后打开手机浏览器，输入192.168.4.1，打开PS2手柄页面，操控小车。

## 成品效果

![](images/AGO.png)

