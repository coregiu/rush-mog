# SLAM 建图 / 自主导航 / 目标搜索：硬件方案与缺口分析

本文回答两个问题：

1. 以“SLAM 建图 + 自主导航去找物体 + ICM-20948 定位”为目标，现有 RUSH MOG 硬件还缺什么？
2. 推荐的实车硬件拓扑、接口、引脚与标定方案是什么？

结论先行：**现有硬件能完成遥控、语音、机械臂和基础视觉演示，但不能直接跑 SLAM + Nav2。**
必须补三块：**机载 Linux 计算平台、2D 激光雷达、轮式编码器闭环**；ICM-20948 是定位链路的必要补充，
但只加 IMU 不足以实现导航。旧 ESP32 不能替代机载计算机；现 ESP32 已与 Pi 合并为 Edge Server。

---

## 1. 目标能力拆解

| 能力 | 需要的输入 | 需要的执行/输出 | 关键硬件 |
|---|---|---|---|
| SLAM 建图 | 2D 激光 + 轮速/IMU 里程计 | 地图（OccupancyGrid）、TF | 激光雷达、编码器、IMU、Linux 计算平台 |
| 自主定位 | 激光 + 地图 + 里程计 + IMU | map→odom→base_link | AMCL、robot_localization、ICM-20948 |
| 自主导航 | 地图、定位、障碍物 | `/cmd_vel`、电机闭环 | Nav2 或内置直航、四轮独立驱动、编码器 |
| 目标搜索 | 相机、激光、地图、定位 | 目标位姿、接近点 | 相机、激光、目标检测、导航 |
| 安全 | 障碍、电量、急停、定位健康 | 急停/回零 | 急停按钮、电压检测、看门狗 |

目标场景是室内 10–30 m²、地面平整、光照可控。若要做室外/多楼层/动态人群，硬件要求更高。

---

## 2. 现有硬件盘点

### 2.1 已有且可复用

| 模块 | 现状 | 对 SLAM/导航的价值 |
|---|---|---|
| STM32F103C8T6 | 车体控制，FreeRTOS，电机/音频/PS2 协议 | 可作为实时运动控制 MCU，但引脚/定时器/RAM 不足，见 §4 |
| L298N 四驱板 | 8 路方向 + 2 路使能 | 能驱动四轮，但只有 2 路 PWM，无法四轮独立调速 |
| JGB37-520 电机 ×4 | 自带霍尔编码器 | 已接入 STM32 PA12/PA15/PB3~PB8，EXTI 四倍频解码，USART3 上报 Pi Edge Server；`edge_server_node` 直接发布 `/odom` |
| 树莓派 Edge Server | Wi-Fi AP、网页手柄、MJPEG 图传、STM32 串口 | 与 ROS 同机；承担网页/底盘/图传，ESP32-S3-CAM 仅旧版回退 |
| PCA9685 + 机械臂 | 16 路舵机驱动 | 抓取执行；不参与导航定位 |
| JQ6500 | 音频播放 | 与导航无关 |
| OLED / GY-53 TOF | 调试/短距测距 | 可做辅助，不能替代 360° 激光 |
| 12V 电池 + 降压板 | 12V/5V/3.3V | 需为新增计算平台和舵机扩容，见 §5 |
| MPU6050 | 6 轴，代码存在但未真正接入主循环 | 可被 ICM-20948 替代；6 轴无磁力计 |
| KiCad PCB | 含 STM32/PCA9685/L298N/ESP32（legacy）/电机/舵机/喇叭 | **没有 IMU、激光、编码器、树莓派接口**，需改板或转接 |

### 2.2 缺失或不足

| 缺口 | 影响 | 结论 |
|---|---|---|
| 无机载 Linux 计算平台 | 旧 ESP32 无 Linux；网页/底盘/ROS 合并后必须由 Pi 承担 | **使用** 树莓派 4B/5 或 Jetson Orin Nano 作为 Edge Server |
| 无 2D 激光雷达 | 无法建图/定位/避障；`edge_server` 只开了 9600 UART，没有解析 | **必须增加** LD19/LD06/RPLIDAR A1 等 |
| 编码器未接线/未闭环 | 无轮式里程计，EKF/Nav2 精度差，麦轮打滑无法校正 | **必须补** 4 路正交编码器 + 轮速 PID |
| STM32F103C8T6 资源不足 | 4 路编码器需要 4 个定时器；引脚冲突多；RAM 仅剩约 2–3 KB | **建议升级** STM32F103RCT6/RET6 或 STM32F411 |
| L298N 只有 2 路使能 | 四轮不能独立调速，麦轮全向运动学无法精确执行 | **建议换** 4 路独立 PWM 驱动（TB6612×2/DRV8871/BTS7960） |
| 无 ICM-20948 | 只有 6 轴 MPU6050，航向漂移快，EKF 缺少可靠角速度 | **增加** ICM-20948（或替代型号），见 §6 |
| 相机为 OV2640 + ESP32 图传 | 分辨率/延迟/曝光一般，目标检测精度有限 | 改用 Pi USB/CSI 相机，由 Edge Server 直接推 MJPEG |
| 无深度/可靠测距 | 2D 激光只测一个平面，目标高度/距离估计受限 | 可选深度相机；否则用“激光方位距离 + bbox 高度”估距 |
| 电源余量不足 | Pi 5V/3A、舵机 6V/5A 峰值容易导致复位/掉电 | 独立 5V/5A UBEC + 6V/5A BEC + 保险丝 |
| 无硬件急停 | 软件急停依赖 Wi-Fi/节点存活 | 增加常闭急停按钮，切断电机使能/驱动电源 |
| 无电量/电流检测 | 自主任务可能突然掉电 | 增加 INA226/INA219 或分压 ADC |
| 无防跌落/碰撞 | 激光低矮盲区、台阶/桌沿风险 | 增加 2–3 路红外/超声波悬崖传感器 + 碰撞开关 |
| 网络架构矛盾 | 旧 ESP32 AP 与 edge_server 连接冲突 | 已由 Pi 统一做 AP/STA；导航在车上闭环 |

---

## 3. 推荐目标架构

### 3.1 三层硬件

```text
┌──────────────────────────────────────────────────────────────┐
│ 云/PC 端（可选）                                              │
│   edge_server：VLA 推理、任务编排、HMI、日志                 │
│   —— 通过 Wi-Fi/以太网与车上 ROS2 通信                        │
└──────────────────────────┬───────────────────────────────────┘
                           │ DDS / Wi-Fi（建议有线或 5GHz）
┌──────────────────────────▼───────────────────────────────────┐
│ 机载 Linux 计算平台（树莓派 4B/5 或 Jetson Orin Nano）         │
│   ROS2 Humble：                                               │
│   激光驱动 / 相机驱动 / base_bridge / EKF / slam_toolbox /    │
│   Nav2 / edge_server（导航与搜索部分）                       │
└───────┬──────────────────┬──────────────────┬────────────────┘
        │ USB/UART         │ USB              │ I2C
┌───────▼────────┐ ┌───────▼────────┐ ┌───────▼────────┐
│ 2D 激光雷达     │ │ USB/CSI 相机    │ │ ICM-20948      │
│ LD19/LD06      │ │ 1080p 广角      │ │ 9 轴 IMU       │
└────────────────┘ └────────────────┘ └────────────────┘
        │
┌───────▼──────────────────────────────────────────────────────┐
│ STM32 实时控制 MCU（建议 F103RCT6/F411）                       │
│   4 路编码器捕获 + 4 路 PWM + 轮速 PID + 看门狗 + 急停          │
│   UART 与 Pi 通信：速度指令 / 编码器 / 电池 / IMU 转发          │
└───────┬──────────────────────────────────────────────────────┘
        │
┌───────▼──────────────────────────────────────────────────────┐
│ 4 路电机驱动（TB6612×2 / DRV8871 / BTS7960） + 12V 电池        │
│ 独立 5V/5A UBEC（Pi） + 6V/5A BEC（舵机） + 保险丝 + 急停       │
└──────────────────────────────────────────────────────────────┘
```

### 3.2 两个部署选项

| 选项 | 车上 | 云端/PC | 适用 |
|---|---|---|---|
| A. 导航在车上（推荐） | Pi 4B/5 跑 ROS2 驱动、EKF、SLAM、Nav2、目标检测 | edge_server 的 VLA/任务编排可远程 | 网络不稳定、需要实时闭环 |
| B. 全在车上 | Jetson Orin Nano 跑全部 edge_server + VLA | 无 | 无网络、要本地 VLA |
| C. 全在 PC（不推荐） | 仅 Pi Edge Server + STM32 | edge_server 远程 | 降级演示；网页/底盘仍在 Pi 本地闭环 |

`edge_server` 的代码同时支持 A/B/C：导航、建图、搜索节点只依赖标准 ROS2 话题/动作；
远程部署时把同一套节点放到 PC 即可，但 `/cmd_vel` 闭环必须在车上或低延迟局域网内。

---

## 4. STM32F103C8T6 引脚/资源冲突（关键）

### 4.1 当前固件实际分配

| 功能 | 引脚 | 说明 |
|---|---|---|
| 电机 IN1–IN4 | PA11, PA10, PA9, PA8 | 左前/左后 |
| 电机 IN5–IN8 | PB15, PB14, PB13, PB12 | 右前/右后 |
| 左/右使能 PWM | PB0 / PB1 | TIM3_CH3/CH4 |
| Pi Edge Server 串口 | PB10 / PB11 | USART3，9600 |
| 语音/JQ6500 | PA2 / PA3 | USART2，9600 |
| LED | PC13 | 调试 |
| SWD | PA13 / PA14 | 下载调试 |

> 硬件 README 旧表把 IN6/IN8 写成 PA14/PA12，与固件不符；以固件为准。
> PA13/PA14 是 SWD，不能当普通 IO，否则无法下载调试。

### 4.2 冲突清单

| 冲突 | 说明 | 处理 |
|---|---|---|
| PB10/PB11 = USART3 vs bit-bang I2C | `iic.c`、OLED、MPU6050 都用 PB10/PB11，和 Pi Edge Server 串口冲突 | I2C 改到 PB6/PB7 或 PB8/PB9；或 UART3 改到其他引脚 |
| PB6/PB7 与 PB8/PB9 都标 I2C1 | STM32F1 的 I2C1 只能选一组（默认或重映射），不能同时 | 只保留一组；另一组改 GPIO/别的外设 |
| PA9/PA10 = USART1 vs 电机 IN3/IN2 | 原理图标注 TX1/RX1，但固件当电机方向脚 | 放弃 USART1；Pi Edge Server 走 USART3 |
| PA11/PA12 = USB D-/D+ vs 电机 IN1/IN8 | 当 GPIO 后 USB 不可用 | 用 ST-Link/UART 下载；或改电机引脚 |
| 4 路编码器无空闲定时器 | TIM3 已用于 PB0/PB1 PWM；TIM1 的 PA8/PA9 被电机占用；TIM4 的 PB6/PB7 被 I2C 占用；只剩 TIM2（PA0/PA1） | **C8T6 无法直接接 4 路编码器**，需升级 MCU 或加计数芯片 |
| RAM 余量 | 当前 `text 15.9KB + data 1.1KB + bss 16.8KB`，20KB RAM 仅剩约 2–3KB | 加编码器/IMU/里程计/协议栈风险高；升级到 48KB/64KB RAM 的型号 |
| Flash 余量 | 64KB 中已用约 17KB | 加功能尚可，但 RAM 是瓶颈 |

### 4.3 升级建议

| 方案 | 优点 | 缺点 |
|---|---|---|
| STM32F103RCT6/RET6（64 pin，256KB Flash/48KB RAM，4 个定时器 + 更多 GPIO） | 同系列 StdPeriph 代码基本可迁移；引脚/定时器够 4 路编码器 | PCB 需重画/转接板 |
| STM32F411CEU6（Black Pill，100MHz，512KB/128KB，5 个 UART） | 性能强、便宜、社区资料多 | 需换 HAL/寄存器层；3.3V |
| 保留 C8T6 + 外部编码器计数芯片（LS7366R ×4，SPI） | 不改主控 | 成本/布线增加；SPI 也要引脚 |
| 保留 C8T6 + 只做 2 路编码器（左右侧） | 改动最小 | 麦轮全向控制/里程计精度差，不推荐 |

推荐 **STM32F103RCT6 + 4 路定时器编码器 + 4 路 PWM**，与现有 FreeRTOS/StdPeriph 工程最兼容。

---

## 5. 电源与安全

### 5.1 电源树

```text
12V 锂电池（建议 3S/4S，≥10Ah，带保护板）
 ├─ 保险丝/总开关
 ├─ 急停（常闭，切断电机驱动使能或 12V）
 ├─ 电机驱动 12V（L298N/TB6612/BTS7960）
 ├─ 5V/5A UBEC → 树莓派/Orin、激光雷达、USB 相机
 ├─ 6V/5A BEC → PCA9685 + MG996 舵机（独立，不与 Pi 共地噪声）
 └─ 12V→3.3V LDO → STM32/IMU/OLED（低噪声）
```

要求：
- Pi 与 STM32、激光、IMU **共地**；舵机电源与逻辑电源分开走线，单点共地。
- 5V 输出至少 5A 峰值；Pi 5 建议 5V/5A，Orin Nano 用官方电源/12V。
- 电池加 INA226/INA219 或分压 ADC，`base_bridge_node` 发布 `/battery_state`。
- 急停按钮建议常闭回路，直接切断电机驱动使能；同时把状态反馈到 MCU GPIO，软件再发零速度。
- 增加保险丝、防反接、TVS。

### 5.2 建议增加的传感器

| 传感器 | 作用 | 接口 |
|---|---|---|
| ICM-20948 | 9 轴 IMU，EKF 角速度/航向 | I2C 0x68/0x69 |
| LD19/LD06 | 360° 2D 激光，SLAM/避障 | USB/UART 230400 |
| USB 相机 | 目标检测/VLA 视觉 | USB/CSI |
| INA226 | 电池电压/电流 | I2C |
| 2–3 路红外/超声波 | 悬崖/低矮障碍 | GPIO/ADC |
| 碰撞开关 | 激光盲区碰撞 | GPIO |
| 硬件急停 | 安全 | 常闭回路 |

---

## 6. ICM-20948 定位方案

### 6.1 为什么加 ICM-20948

- MPU6050 只有 6 轴，yaw 靠陀螺积分，几分钟就漂移；没有磁力计无法绝对校正。
- ICM-20948 = 3 轴加速度 + 3 轴陀螺 + AK09916 三轴磁力计，100 Hz 输出足够 EKF。
- 轮式里程计 + 陀螺仪：短时平滑；激光/AMCL：长期绝对定位；磁力计：静止/低速时慢速校正。

### 6.2 接法（推荐）

| 方案 | 连接 | 适用 | 备注 |
|---|---|---|---|
| A（推荐） | ICM-20948 → 树莓派 I2C1（3.3V，SDA/SCL，INT 可选） | Pi 直接读，100 Hz | 需要 `python3-smbus2`；`imu_node` 已实现 |
| B | ICM-20948 → STM32 I2C（PB6/PB7 或 PB8/PB9） | IMU 与电机控制同板 | 需解决 I2C 引脚冲突；STM32 通过 UART 转发 |
| C | ICM-20948 → 旧 ESP32 I2C | 仅旧版回退 | 不推荐用于 SLAM；当前已并入 Pi |

接线：
- VCC 3.3V（不要 5V），GND 共地。
- SDA/SCL 4.7k 上拉到 3.3V；长线加屏蔽/双绞。
- SA0 接地 = 0x68；接 3.3V = 0x69。
- 安装在底盘中心、刚性固定、远离电机/电池/大电流线；磁力计尤其怕电机磁场。

### 6.3 数据与融合

```text
ICM-20948 (100Hz) ──/imu/data──┐
轮式编码器 (30–50Hz) ──/odom────┼─ robot_localization EKF ── odom→base_link
                               │
2D 激光 + 地图 ── AMCL ────────┴─ map→odom
```

- EKF 配置见 `edge_server/config/ekf.yaml`：`two_d_mode: true`，融合 `x,y,yaw,vx,vy,vyaw` + IMU `yaw,vyaw`。
- `localization_node` 不替代 EKF，只做健康度/质量门控与初始位姿服务。
- 磁力计只在“可信”时慢速校正（`mag_blend=0.02`）；电机电流大/磁场异常时标记 `mag_ok=false`，不硬切航向。
- 磁力计需做硬铁/软铁校准；`imu.calibration.mag_bias/mag_scale` 填入实测值。

### 6.4 供应风险

ICM-20948 已进入 NRND/替代周期，TDK 提供 transition guide；新设计可评估
ICM-42688-P + 磁力计（如 MMC5983MA）组合。当前代码的驱动接口与单位已封装，
替换芯片只需改 `drivers/icm20948.py` 的寄存器映射。

参考：
- [TDK ICM-20948 Design Transition Guide](https://www.symmetryelectronics.com/blog/invensense-icm-20948-design-transition-guide/)
- [ICM-20948 NRND/替代说明](https://pcbsync.com/icm-20948/)

---

## 7. 激光雷达选型

| 型号 | 量程 | 扫描 | 接口 | 价格区间 | 备注 |
|---|---|---|---|---|---|
| LD19 / LD06 (D300) | 0.02–12 m | 360°, 10 Hz | UART/USB 转接 | ¥250–400 | 体积小、ROS2 驱动多，推荐入门 |
| RPLIDAR A1M8 | 0.15–12 m | 360°, 5.5 Hz | USB | ¥400–600 | 生态成熟，扫描频率低 |
| RPLIDAR A2M8/A2M12 | 0.15–16 m | 360°, 10–15 Hz | USB | ¥800+ | 精度更好 |
| YDLIDAR X2/X3 | 0.1–8 m | 360° | UART | ¥200–350 | 便宜，量程短 |

要求：
- 扫描频率 ≥ 10 Hz；角分辨率 ≤ 1°；量程 ≥ 8 m。
- 安装高度避开底盘/轮子遮挡，360° 无遮挡；激光平面尽量低（15–25 cm）以看到桌腿/墙脚。
- 接 Pi USB，udev 固定 `/dev/rplidar`；不要经旧 ESP32 桥转发（带宽/时间戳/驱动都不合适）。
- 驱动输出 `/scan`，frame_id `laser_link`；TF `base_link→laser_link` 必须标定。

参考：[LD19 产品页](https://littlebirdelectronics.com.au/products/dtof-ld19-laser-lidar-sensor-kit-12m)

---

## 8. 相机与目标定位

### 8.1 相机

| 方案 | 优点 | 缺点 |
|---|---|---|
| Pi USB 1080p 广角 | 即插即用、低延迟、可换镜头 | 需 USB 带宽/供电 |
| Pi CSI 相机 | 低延迟、硬件编码 | 排线/型号适配 |
| 旧 ESP32 OV2640 MJPEG | 仅旧版回退 | 延迟/分辨率/曝光有限，不适合精细检测 |

建议：目标检测和图传都用 Pi USB/CSI 相机；OV2640 仅旧版回退。相机 frame `camera_link`，
TF `base_link→camera_link` 标定；内参用棋盘格标定后填入 `perception.camera.fx/fy/cx/cy`。

### 8.2 目标距离估计

`target_detector_node` 当前支持：

1. **2D 激光**：在目标方位 ±3° 内取最近有效距离（`lidar`），精度最好，但目标必须落在激光平面；
2. **bbox 高度**：`distance = fy * real_height / bbox_height_px`（`bbox_height`），需要类别先验高度；
3. **深度相机**（预留）：接 RGB-D 后可在 `TargetLocalizer.estimate` 增加 depth 分支。

局限：激光可能打到目标后面的墙；bbox 高度受姿态/遮挡影响。若要做稳定抓取，建议加
Orbbec Astra Pro Plus / RealSense D435i 等 RGB-D，或用 AprilTag/二维码辅助。

---

## 9. 编码器与轮速闭环

### 9.1 编码器接线

JGB37-520 常见霍尔编码器：VCC 5V、GND、A、B。四轮共 8 路信号 + 电源。
- 每路 A/B 接 STM32 EXTI：左前 PA12/PA15、右前 PB3/PB4、左后 PB5/PB6、右后 PB7/PB8；
  固件关闭 JTAG、保留 SWD，软件四倍频解码。
- 编码器若是 5V 推挽输出，PA12（USB D+）的耐压需按数据手册确认；建议编码器用 3.3V
  供电，或加电平转换/分压。开漏输出可用 STM32 内部上拉（本固件已启用）。
- 串联 100–330Ω + 100nF 到地做 RC 滤波；长线用屏蔽线，屏蔽层单端接地。
- 编码器电源与电机电源分开，避免 PWM 噪声。

### 9.2 参数标定

| 参数 | 含义 | 标定方法 |
|---|---|---|
| `wheel_radius` | 轮半径 | 卷尺量直径/2；或推 1 m 数编码器 |
| `encoder_cpr` | 输出轴每圈计数 | 手动转一圈读计数（含减速比与 4 倍频） |
| `wheelbase_x` / `track_y` | 轴距/轮距 | 卷尺量；麦轮用 (L+W)/2 作为旋转半径 |
| `sign_*` | 电机/编码器方向 | 单轮正转，确认计数增加方向 |
| PID | 轮速闭环 | 先 P 后 I，限制积分饱和；空载/负载分别调 |

### 9.3 验收

- 直行 1 m，里程计误差 < 2%。
- 原地转 360°，yaw 误差 < 3°。
- 麦轮横移 1 m，横向误差 < 5 cm。
- 推车/堵转时看门狗 0.5 s 内回零。

---

## 10. 采购清单（增量）

价格为目标区间，按 2026 年国内电商/淘宝估算。

| 物料 | 建议型号 | 数量 | 价格 | 接口 | 备注 |
|---|---|---|---|---|---|
| 机载计算机 | 树莓派 4B 4GB/8GB 或 Pi 5 8GB | 1 | ¥400–700 | — | 跑 ROS2/SLAM/Nav2；VLA 用 Orin Nano |
| 2D 激光 | LD19/LD06 | 1 | ¥250–400 | USB/UART | 12 m/360°/10 Hz |
| IMU | ICM-20948 模块 | 1 | ¥30–60 | I2C | 3.3V；NRND 风险 |
| 相机 | USB 1080p 广角 | 1 | ¥80–200 | USB | 目标检测 |
| 电机驱动 | TB6612×2 或 DRV8871×4/BTS7960×2 | 1 套 | ¥40–120 | PWM/GPIO | 四轮独立 PWM |
| 主控升级 | STM32F103RCT6 最小系统/转接板 | 1 | ¥25–50 | — | 4 编码器 + 4 PWM |
| 5V 电源 | 5V/5A UBEC | 1 | ¥30–60 | — | Pi/激光/相机 |
| 6V 电源 | 6V/5A BEC | 1 | ¥25–50 | — | 舵机独立 |
| 电量检测 | INA226/INA219 模块 | 1 | ¥15–30 | I2C | 电压/电流 |
| 急停 | 常闭急停按钮 | 1 | ¥10–25 | — | 切电机使能 |
| 悬崖/避障 | 红外或超声波模块 | 2–3 | ¥10–20/个 | GPIO/ADC | 激光盲区 |
| 碰撞开关 | 微动开关 | 2 | ¥5/个 | GPIO | 前/后 |
| 存储 | 64GB 高耐久 SD 或 NVMe | 1 | ¥50–150 | — | 地图/rosbag |
| 散热 | 风扇/散热片 | 1 | ¥15–40 | — | Pi5/Orin |
| 结构 | 激光/相机/IMU 支架 | 1 套 | ¥30–80 | — | 3D 打印/铝板 |
| 可选 | RGB-D 相机（Orbbec/RealSense） | 1 | ¥800–2500 | USB | 稳定目标定位 |
| 可选 | 独立路由器/5GHz AP | 1 | ¥100–200 | — | 远程 edge_server |

**最低可跑 SLAM+导航的增量预算约 ¥900–1500**（Pi 4B + LD19 + ICM-20948 + 驱动 + 电源 + 急停 + 主控升级）。

---

## 11. 从现有车到目标车的改造顺序

1. **先解决计算平台**：Pi 4B/5 装 Ubuntu 22.04 + ROS2 Humble；能跑 `ros2 topic list`。
2. **接激光雷达**：USB 接 Pi，装 LD19 驱动，确认 `/scan` 10 Hz、360°。
3. **接编码器**：升级 MCU 或加计数芯片；实现 4 路编码器 + 轮速 PID；串口回传 `/odom`。
4. **接 ICM-20948**：Pi I2C 或 STM32 转发；确认 `/imu/data` 100 Hz；做静态/六面标定。
5. **跑 EKF**：`robot_localization` 融合 `/odom` + `/imu/data`，确认 `odom→base_link` TF。
6. **跑 SLAM**：`slam_toolbox` 建图，推车/遥控走一圈，保存地图。
7. **跑定位**：AMCL + 地图，设置初始位姿，确认 `map→odom`。
8. **跑导航**：Nav2 或 edge_server 内置直航，先空载 1 m 到点，再加载。
9. **跑目标搜索**：相机 + 激光 + 目标检测，先静态目标，再动态目标。
10. **加安全**：急停、悬崖、电量、看门狗、远程急停。

每一步都用 `edge_server` 的接口做验收，避免“全部接完再调试”。

---

## 12. 主要风险与对策

| 风险 | 表现 | 对策 |
|---|---|---|
| ICM-20948 停产/缺货 | 采购困难 | 备选 ICM-42688-P + MMC5983MA；驱动接口抽象 |
| 磁力计受电机干扰 | yaw 跳变 | 不硬融合；`mag_ok` 门控；远离大电流线；校准 |
| L298N 压降/发热 | 速度不准、驱动保护 | 换 MOSFET 驱动；加散热；电流检测 |
| 编码器噪声 | 里程计跳变 | RC 滤波、屏蔽线、软件限幅/中值 |
| Pi 供电不足 | 重启/掉盘 | 5V/5A UBEC、独立舵机 BEC、保险丝、UPS |
| Wi-Fi 丢包 | 远程导航抖动 | 导航在车上闭环；5GHz/有线；DDS 调参 |
| 激光盲区 | 撞桌腿/台阶 | 低矮障碍传感器、碰撞开关、减速策略 |
| 麦轮打滑 | 里程计漂移 | 激光/AMCL 校正；轮速闭环；地面清理 |
| 地图过大 | 前沿提取慢 | `frontier_max_cells` 下采样；分区建图 |

---

## 13. 与 edge_server 的接口对应

| 硬件 | ROS2 话题/服务 | edge_server 节点 |
|---|---|---|
| 激光 | `/scan` | `bridge_node`、`safety_manager_node`、`navigation_node` |
| 编码器/底盘 | `/odom`、`/joint_states`、`/battery_state`、`/imu/data` | `base_bridge_node` |
| ICM-20948 | `/imu/data` | `base_bridge_node` 或 `imu_node` |
| 地图 | `/map`、`/center/map_info`、`/center/frontiers` | `map_manager_node` |
| 定位 | `/amcl_pose`、TF、`/center/pose`、`/center/localization_status` | `localization_node` |
| 导航 | `/center/navigate_to`、`/nav/cmd_vel` | `navigation_node` |
| 目标 | `/camera/image_raw`、`/center/target_detections` | `target_detector_node` |
| 搜索 | `/center/search_object`、`/center/explore` | `object_search_node` |
| 速度 | `/cmd_vel_raw`、`/nav/cmd_vel`、`/reverse/cmd_vel`、`/safety/cmd_vel_override` → `/cmd_vel` | `velocity_mux_node` |
| 任务 | `/center/execute_task`、`/center/execute_mission` | `task_manager_node` |

详细软件设计见 [`software/edge_server/DESIGN.md`](../software/edge_server/DESIGN.md)。
