## 整体设计

![](integrate_archetecture.png)

其它说明：树莓派U口接摄像头。

> **SLAM/导航目标方案与硬件缺口分析见 [SLAM_NAV.md](SLAM_NAV.md)。**
> 现有车能完成遥控、语音、机械臂和视觉演示；要跑 SLAM + Nav2 + 目标搜索，必须补：
> 机载 Linux 计算平台（树莓派/Orin）、2D 激光雷达、轮式编码器闭环。
> ICM-20948 是定位链路的必要补充，但只加 IMU 不能替代激光和编码器。

## 硬件模块及工具清单：

| 模块          | 说明                                                         | 价格(元) |
| ------------- | ------------------------------------------------------------ | -------- |
| STM32f103C8T6 | 控制单板，作为控制中心。**SLAM 目标建议升级为 F103RCT6/F411**（4 路编码器 + 4 路 PWM）。 | 15       |
| 小车底盘       | 金属底盘，JGB37520电机带霍尔效应编码器，麦克纳姆万向轮             | 350       |
| L298N四驱板    | 四驱的L298N电机驱动。**只有 2 路使能，四轮不能独立调速；目标方案换 TB6612/DRV8871/BTS7960。** | 28       |
| 语音控制模块    | ASR PRO语音控制板                                             | 25       |
| 电源及转换板    | 12V电源，DC接口。 12V转5V和3.3V转换板。**目标方案加 5V/5A UBEC、6V/5A 舵机 BEC、保险丝、急停。** | 40       |
| 摄像头         | OV2640摄像头模块；SLAM/目标检测建议加 Pi USB 1080p 相机 | 10       |
| 机械臂         | 机械臂器件带舵机，这里用的最普通的MG996舵机，再好一点用串行总线舵机，再贵一点的用伺服电机，但后面的都比较贵了。                                               | 200       |
| 舵机驱动       | PCA9685 16路舵机驱动                                          | 16       |
| JQ6500       | MP3播放器                                                    | 15       |
| OLED屏       | OLED屏，用来显示调试/监控信息                                 | 10       |
| TOF测距       | OpenMV扩展，弥补OpenMV在测距方面的不足                        | 10       |
| 陀螺仪        | **ICM-20948 9 轴 IMU**（替代 MPU6050），用于 EKF/定位          | 40       |
| ESP32-S3-CAM   | 用来创建WIFI网络，连接手机APP；可做图传/遥控桥，**不跑 SLAM** | 35       |
| 2D 激光雷达    | **LD19/LD06 或 RPLIDAR A1**，360°/12m/10Hz，接树莓派 USB       | 300      |
| 机载计算机     | **树莓派 4B/5 或 Jetson Orin Nano**，跑 ROS2/SLAM/Nav2         | 500      |
| 电机驱动升级   | **TB6612×2 / DRV8871×4 / BTS7960×2**，四轮独立 PWM             | 80       |
| 电量检测       | **INA226/INA219**，电压/电流监测                               | 20       |
| 硬件急停       | **常闭急停按钮**，切断电机驱动使能/电源                        | 15       |
| 悬崖/碰撞      | 红外/超声波悬崖传感器 + 微动碰撞开关                           | 40       |

> 目标增量预算约 ¥900–1500（不含机械臂/底盘）。详细选型、接线、引脚冲突和标定见
> [SLAM_NAV.md](SLAM_NAV.md)。


## 组装效果
![](../images/AGO.png)

## 模块说明

#### 语音识别模块

<img src="modules/asr-pro.jpg" />

使用语音识别模块进行语音识别，转换成控制小车的指令。

#### 控制中心-STM32单板

<img src="modules/stm32f103c8t6.jpg" style="zoom:70%;" />

STM32单板作为中心控制单元，接收语音命令，转换成电机驱动指令下发到电机驱动。
同时对接树莓派，树莓派进行图像识别后传回控制指令到STM32， STM32根据指令驱动机械臂舵机。

IO口分配如下（以 `software/body_control` 固件为准）：

| GPIO         | 对接口                  | 作用说明          | 备注 |
| ------------ | ---------------------- | ---------------- | ---- |
| PA11         | 电机驱动IN1             |  左前电机控制1     | 与 USB D- 复用 |
| PA10         | 电机驱动IN2             |  左前电机控制2     | 与 USART1 RX 复用 |
| PA9          | 电机驱动IN3             |  左后电机控制1     | 与 USART1 TX 复用 |
| PA8          | 电机驱动IN4             |  左后电机控制2     | TIM1_CH1 |
| PB15         | 电机驱动IN5             |  右前电机控制1     | |
| PB14         | 电机驱动IN6             |  右前电机控制2     | 旧文档误写为 PA14（SWD） |
| PB13         | 电机驱动IN7             |  右后电机控制1     | |
| PB12         | 电机驱动IN8             |  右后电机控制2     | 旧文档误写为 PA12 |
| PB0          | 小车左侧电机使能        |  TIM3_CH3 PWM      | |
| PB1          | 小车右侧电机使能        |  TIM3_CH4 PWM      | |
| PA2          | UART2 TX接ASR板PA6 RX  |  语音板串口通信     | 也接 JQ6500 |
| PA3          | UART2 RX接ASR板PA5 TX  |  语音板串口通信     | |
| PB10         | UART3 TX接ESP32 GPIO20 |  遥控/图传串口      | **与 bit-bang I2C 冲突** |
| PB11         | UART3 RX接ESP32 GPIO19 |  遥控/图传串口      | **与 bit-bang I2C 冲突** |
| PB6 / PB7    | I2C1（树莓派/外设）     |  树莓派通信        | 与 PB8/PB9 只能二选一 |
| PB8 / PB9    | I2C1 重映射（PCA9685）  |  舵机驱动通信      | 与 PB6/PB7 只能二选一 |
| PA13 / PA14  | SWDIO / SWCLK          |  下载调试          | **不能当普通 IO** |
| PC13         | LED                    |  调试指示          | |

> ⚠️ 现有 F103C8T6 的引脚/定时器/RAM 不足以直接接 4 路编码器：
> TIM3 已用于 PB0/PB1 PWM，TIM1 的 PA8/PA9 被电机占用，TIM4 的 PB6/PB7 被 I2C 占用，只剩 TIM2。
> 同时 PB10/PB11 同时被 USART3 和 bit-bang I2C 使用，存在硬冲突。
> 目标方案建议升级 STM32F103RCT6/F411，或加编码器计数芯片。详见 [SLAM_NAV.md](SLAM_NAV.md) §4。


#### ESP32-S3-CAM
![esp32-s3-cam.png](modules/esp32-s3-cam.png)

ESP32-S3-CAM 负责 Wi-Fi AP、网页手柄、MJPEG 图传和遥控串口桥。它没有 Linux，
不能运行 ROS2/slam_toolbox/Nav2，因此不做“机载 SLAM 计算”。

#### 2D 激光雷达（SLAM/导航必需）
推荐 LD19/LD06（360°, 12 m, 10 Hz）或 RPLIDAR A1。接树莓派 USB，由 ROS2 驱动发布
`/scan`。安装高度 15–25 cm，360° 无遮挡。不要接 ESP32 再转发：带宽、时间戳和驱动都不合适。
详见 [SLAM_NAV.md](SLAM_NAV.md) §7。

#### 机载计算机
树莓派 4B/5 或 Jetson Orin Nano，跑 Ubuntu 22.04 + ROS2 Humble：
激光/相机驱动、`base_bridge_node`、`robot_localization` EKF、`slam_toolbox`、Nav2、
`center_server` 的导航/搜索节点。需要 5V/5A 供电和散热。

#### ICM-20948 9 轴 IMU
替代 MPU6050。3.3V I2C（0x68/0x69），AK09916 磁力计通过 bypass 访问。安装在底盘中心、
刚性固定、远离电机/电池/大电流线。100 Hz 输出，用于 EKF 融合；磁力计受电机干扰时只做
慢速校正，不硬切航向。驱动和标定见 `software/center_server/center_server/drivers/icm20948.py`
与 [SLAM_NAV.md](SLAM_NAV.md) §6。

> ICM-20948 已进入 NRND/替代周期，新设计可评估 ICM-42688-P + 磁力计组合。

#### 轮式编码器与轮速闭环
JGB37-520 自带霍尔编码器，但**当前未接线、未写固件**。四轮 A/B 相需 4 个定时器编码器模式；
现有 F103C8T6 没有空闲定时器，建议升级 F103RCT6/F411 或加 LS7366R 计数芯片。
编码器 → 轮速 PID → `/odom`，是 EKF 和 Nav2 的基础。详见 [SLAM_NAV.md](SLAM_NAV.md) §9。

#### 小车底盘
小车底盘是拼多多淘的底盘+JGB37 520电机+万向轮组装。拼多多也有成套的，成套的更贵一点。

<img src="modules/board.jpg" style="zoom:60%;" />
<img src="modules/motor.jpg" style="zoom:60%;" />
<img src="modules/wheel.jpg" style="zoom:60%;" />
<img src="modules/vehicle_method.jpg" style="zoom:60%;" />

#### 电机驱动模块
<img src="modules/l298n.jpg" style="zoom:60%;" />

使用L298N电机驱动，控制小车四个电机，四驱控制。

12V输入电压，5V输出电压为控制单板等其他模块供电。

#### 机械臂

机械臂从拼多多购买散件拼装，带6个180度舵机。

<img src="modules/robotic-arm.jpg" />

#### 舵机驱动pca9685
<img src="modules/servo-driver.jpg" />
16路舵机驱动

#### 视觉模块
基础演示可用 OpenMV/OV2640；SLAM/目标搜索建议在树莓派上接 USB 1080p 广角相机，
由 `target_detector_node` 做颜色/YOLO 检测。相机内参和 `base_link→camera_link` 需标定。
<img src="modules/OpenMV4.jpg" />
[教程参照:https://singtown.com/openmv/](https://singtown.com/openmv/)

#### 9轴陀螺仪 ICM-20948
<img src="modules/mpu6050.jpg" />

MPU6050 为旧方案（6 轴）；目标方案使用 ICM-20948（9 轴，含 AK09916 磁力计）。

#### OLED显示屏
<img src="modules/oled.jpg" />

#### GY53L0X TOF测距
<img src="modules/gy530.jpg" />

#### 电源及电压转换板
小车上有12V，5V， 3.3V三种电压，最好用12V供电电源，并用电压转换板转换一下。
<img src="modules/power_module.jpg" />

#### 音频放大器
如果嫌声音小，可以这个放大器进一步放大音量。
<img src="modules/upper.jpg" />

#### 树莓派
<img src="modules/raspberry.jpg" />
树莓派 4B/5（或 Orin Nano）作为机载 ROS2 计算平台：接激光雷达、相机、ICM-20948，
通过 UART 与 STM32 通信，运行 EKF/SLAM/Nav2 与 center_server 的导航/搜索节点。

## SLAM/导航硬件改造清单

完整方案、BOM、接线、引脚冲突、标定与验收见 [SLAM_NAV.md](SLAM_NAV.md)。最小改造：

1. 树莓派 4B/5（5V/5A UBEC）；
2. LD19/LD06 激光雷达（USB，`/scan`）；
3. ICM-20948（Pi I2C 或 STM32 转发，`/imu/data`）；
4. STM32F103RCT6/F411 + 4 路编码器 + 4 路独立 PWM 驱动；
5. 硬件急停、INA226 电量检测、5V/6V 独立电源；
6. 可选：USB 相机、RGB-D、悬崖/碰撞传感器。

## 硬件设计工具KiCAD

安装：https://docs.kicad.org/5.1/zh/getting_started_in_kicad/getting_started_in_kicad.html

## 组装注意事项
- 1 电机驱动与IO口连接及小车方向提前规划好，不要连错口了。
- 2 机械臂组装前一定要调试舵机，使该臂摆动范围在舵机控制范围内。
- 3 asrpro板子有的店家的比较脆弱，供电不能用12V锂电池转的电压，遇到此类情况退换货。
- 4 ASRPRO与STM32以及STM32与PCA9685连接时注意一定要共接地线。
