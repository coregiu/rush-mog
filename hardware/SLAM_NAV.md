# SLAM 建图 / 自主导航 / 目标搜索：硬件方案与缺口分析

> 本次刷新对应：`hardware/README.md` 新 BOM（Pi 4B + D6 激光 + ICM-20948 + Astra Pro + LM393 悬崖）、
> 编码器改到 PA0~PA7 的 body_control 固件、以及 2026-11-13 刷新后的 KiCad 原理图 `hardware/pcb_design/ago.kicad_sch`。

本文回答两个问题：

1. 以“SLAM 建图 + 自主导航去找物体 + ICM-20948 定位”为目标，当前 RUSH MOG 硬件还缺什么？
2. 推荐的实车硬件拓扑、接口、引脚与标定方案是什么？

## 0. 结论先行

**2D SLAM 建图的三个必要件已经齐了**：D6 激光雷达（Pi UART4）、4 路轮式编码器（STM32 PA0~PA7，
EXTI 四倍频，USART3 上报 Pi）、ICM-20948（Pi I2C1）。理论上接完线、标定完就能跑
`slam_toolbox` 建图。

**但离“能稳定建图 + 导航”还差这些，按优先级：**

| 优先级 | 缺口 | 为什么卡 SLAM |
|---|---|---|
| P0 | **5V/5A UBEC 等电源扩容**（现在 5V/3A） | Pi 4B 峰值 1.5A + 深度相机 ~0.7A + 激光 ~0.6A，3A 会在建图途中掉电重启，地图直接废 |
| P0 | **4 路独立 PWM 电机驱动**（L298N 只有 2 路使能） | 麦轮四轮不能独立调速，轮速闭环和 `holonomic` 运动学做不准，`/odom` 漂移大 |
| P0 | **编码器电平处理**（PA0~PA7 不是 5V 容忍） | JGB37-520 编码器多为 5V 推挽输出，直连会倒灌/计数乱跳，里程计不可信 |
| P0 | **硬件急停 + 保险丝 + 总开关** | 建图时人在车后，Wi-Fi/节点挂了只能靠硬件断电 |
| P1 | **激光/IMU/相机支架** | 原理图里这些器件都还没连线，更没有安装结构；IMU 必须刚性固定在底盘中心 |
| P1 | **INA226 电量检测** | 自主建图/导航途中掉电没有预警 |
| P1 | **悬崖传感器补到 2~3 路并分配 GPIO** | 现在只有 GPIO27 一个信号脚，激光盲区看不到桌沿 |
| P2 | 散热/高耐久存储、碰撞开关、5GHz AP、USB 声卡 | 长时间建图稳定性/体验 |

> ✅ 已解决（2026-11-14）：USART3 波特率 **9600 → 115200**，固件 `BODY_UART_BAUD_RATE` 与
> `edge_server/config/robot.yaml` 的 `base.baud` 已同步。原来 37~48 字符/50 ms 的遥测在
> 9600 下要占 77%~100% 带宽，现在 115200 下一帧只要约 4~5 ms。

**原理图（2026-11-13 刷新版）现状**：器件已经摆齐（Pi 4、STM32、ICM-20948、L298N、4 电机、
4 编码器、2×LM393、喇叭），但**只画通了 STM32→L298N→电机这一条链**；Pi 排针、IMU、4 个编码器、
LM393、喇叭的引脚全部悬空。另外 12V 接在 L298N 的 `Vss(9)`、`Vs(4)` 悬空，按 L298N 数据手册
`Vs` 才是电机电源、`Vss` 是 5V 逻辑电源，**投板前必须核对**。详见 §5。

---

## 1. 目标能力拆解

| 能力 | 需要的输入 | 需要的执行/输出 | 关键硬件 |
|---|---|---|---|
| SLAM 建图 | 2D 激光 + 轮速/IMU 里程计 | 地图（OccupancyGrid）、TF | 激光雷达、编码器、IMU、Linux 计算平台 |
| 自主定位 | 激光 + 地图 + 里程计 + IMU | map→odom→base_link | AMCL、robot_localization、ICM-20948 |
| 自主导航 | 地图、定位、障碍物 | `/cmd_vel`、电机闭环 | Nav2 或内置直航、四轮独立驱动、编码器 |
| 目标搜索 | 相机、激光、地图、定位 | 目标位姿、接近点 | 相机、激光、目标检测、导航 |
| 安全 | 障碍、电量、急停、定位健康 | 急停/回零 | 急停按钮、电压检测、看门狗 |

目标场景：室内 10–30 m²、地面平整、光照可控。室外/多楼层/动态人群要求更高。

---

## 2. 现有硬件盘点

### 2.1 已到位 / 已选型

| 模块 | 现状 | 对 SLAM/导航的价值 |
|---|---|---|
| 树莓派 4B | Edge Server：Wi-Fi AP、网页手柄、MJPEG、STM32 串口、ROS2 Humble | 机载计算平台，跑 EKF / slam_toolbox / Nav2 |
| STM32F103C8T6 | FreeRTOS，电机 + 编码器 + USART3；音频已关闭 | 实时运动控制 + 编码器采集 |
| JGB37-520 电机 ×4 | 自带霍尔编码器，已接 PA0~PA7（EXTI 四倍频） | 轮式里程计 `/odom`（20 Hz 上报） |
| D6 2D 激光 | 已选型，接 Pi UART4 GPIO8/9 | SLAM 建图/定位/避障的主传感器 |
| ICM-20948 | 已选型，接 Pi I2C1 GPIO2/3 | EKF 的角速度/航向，补编码器短时误差 |
| Astra Pro 深度相机 | 已选型，USB 接 Pi | 目标检测/接近；2D 建图非必需 |
| LM393 红外悬崖 ×2 | 已选型，输出接 Pi GPIO27 | 激光盲区安全（桌沿/台阶） |
| USB 全向麦克风 / 蓝牙音箱 | 已选型 | 语音交互，与导航解耦 |
| 12V 3600mAh + 12V→5V 3A / 3.3V 1A | 已选型 | 供电（**3A 偏紧，见 §6**） |
| L298N 四驱板 | 已有 | 能驱动四轮，但只有 2 路使能（**P0 瓶颈**） |
| PCA9685 + 机械臂 | 已有 | 抓取执行，不参与定位；注意机械臂会遮挡激光 |

### 2.2 仍缺 / 不足

| 缺口 | 影响 | 结论 |
|---|---|---|
| 5V 只有 3A | Pi + 相机 + 激光峰值可能超 3A，掉电重启 | **换 5V/5A UBEC**，Pi 单独一路、电机 12V 单独一路 |
| L298N 只有 2 路使能 | 四轮不能独立调速，麦轮全向/轮速 PID 做不准 | **换** TB6612FNG×2 / DRV8871×4 / BTS7960×2 |
| PA0~PA7 非 5V 容忍 | 编码器 5V 输出倒灌，计数乱跳/损坏 IO | 编码器改 3.3V 供电或加分压/电平转换 |
| USART3 9600 已饱和 ✅ | 遥测挤占命令，`/odom` 抖动 | 已改 **115200**（固件 + `robot.yaml` 同步） |
| 无急停/保险丝 | 失控只能拔电 | 常闭急停 + 保险丝 + 总开关 |
| 无电量检测 | 建图中途掉电，地图/参数丢失 | INA226/INA219（I2C） |
| 无安装结构 | 激光/IMU/相机没有刚性基准，TF 标不准 | 3D 打印/铝板支架，IMU 加减震柱 |
| 悬崖只有 1 个 GPIO | 多路传感器无法区分方向 | 2~3 路各占一个 GPIO，或二极管 OR 后接一个 |
| 深度相机驱动待确认 | Astra Pro 是老款结构光，ROS2 支持有限 | 先验证出 `/camera/depth/image_raw`，否则换 Astra Pro Plus / Gemini 2 / D435i |
| PCB 未布线 | `ago.kicad_pcb` 只有空骨架 | 目前只能面包板/杜邦线，投板前先按 §5 修原理图 |
| Flash/RAM 余量 | Flash 17.5KB/64KB 够；RAM 18.0KB/20KB（88%） | 再加功能要换 F103RCT6/F411 或精简缓冲区 |

---

## 3. 实际硬件拓扑

```text
                          Wi-Fi 5GHz / 有线
                                  │
┌─────────────────────────────────▼─────────────────────────────────┐
│ 树莓派 4B（Ubuntu 22.04 + ROS2 Humble + edge_server）              │
│  UART3 GPIO4/5  ── 115200 8N1 ─── STM32 USART3 (PB10/PB11)        │
│  UART4 GPIO8/9  ── TTL UART ───── D6 激光雷达（5V 独立供电）        │
│  I2C1  GPIO2/3  ── 3.3V I2C ───── ICM-20948（0x68）               │
│  GPIO27         ◄── LM393 悬崖输出（3.3V 电平）                    │
│  USB3           ◄── Astra Pro 深度相机                            │
│  USB2           ◄── USB 全向麦克风                                │
│  蓝牙           ──► 蓝牙音箱                                      │
└───────────────┬───────────────────────────────────────────────────┘
                │ UART3（ASCII 命令 + $B 遥测）
┌───────────────▼───────────────────────────────────────────────────┐
│ STM32F103C8T6（body_control，FreeRTOS）                            │
│  PA0~PA7   ◄── 4 路编码器 A/B（EXTI 四倍频）                       │
│  PA8~PA11  ──► L298N IN1~IN4（左前/左后）                          │
│  PB12~PB15 ──► L298N IN5~IN8（右前/右后）                          │
│  PB0/PB1   ──► L298N ENA+EnB / EnC+EnD（TIM3_CH3/CH4，左右两组）    │
│  PC13      ──► 调试 LED                                           │
│  PA13/PA14 ──  SWD 下载调试（不能当 IO）                            │
└───────────────┬───────────────────────────────────────────────────┘
                │
┌───────────────▼───────────────────────────────────────────────────┐
│ 电源：12V 电池 → 保险丝/总开关/急停 → 电机 12V                      │
│                    └→ 5V/5A UBEC → Pi / D6 / 相机 / 麦             │
│                    └→ 3.3V 1A  → IMU / 编码器上拉（若需要）         │
│ 要求：Pi、STM32、激光、IMU、电机驱动 **单点共地**                    │
└───────────────────────────────────────────────────────────────────┘
```

---

## 4. 接口、电平与树莓派配置

### 4.1 接线表（以 `hardware/README.md` 与固件为准）

| 信号 | 树莓派 | 对方 | 电平/方向 | 备注 |
|---|---|---|---|---|
| STM32 串口 | UART3 GPIO4 (TX) / GPIO5 (RX) | STM32 PB11 (RX) / PB10 (TX) | 3.3V TTL，交叉 | 115200 8N1（已配置） |
| 激光雷达 | UART4 GPIO8 (TX) / GPIO9 (RX) | D6 的 RX / TX | 3.3V TTL，交叉 | 5V 独立供电，共地 |
| IMU | I2C1 GPIO2 (SDA) / GPIO3 (SCL) | ICM-20948 SDA/SCL | 3.3V，0x68 | 4.7k 上拉，线尽量短 |
| 悬崖 | GPIO27 | LM393 OUT | 3.3V 输入 | 需要多路时另占 GPIO |
| 深度相机 | USB3 | Astra Pro | USB 2.0 设备 | 不要经 USB Hub，避免带宽/供电 |
| 麦克风 | USB2 | USB 全向麦 | USB | 与相机分开口 |
| 音箱 | 蓝牙 | 蓝牙音箱 | — | 2.4G Wi-Fi 与蓝牙会互相干扰 |

### 4.2 电平与接地方向

- **GPIO4/5/8/9/2/3/27 全部是 3.3V**，不能接 5V 信号；D6 / ICM-20948 若是 5V 供电模块，
  要先确认其 UART/I2C 输出是 3.3V 还是 5V，必要时加电平转换。
- STM32 的 PB10/PB11 是 5V 容忍，接 Pi 3.3V 没问题；但**必须共地**，否则串口随机丢字节。
- 激光雷达、电机、Pi 的地要单点汇合；电机 PWM 大电流地不要穿过 Pi 的地线。
- 串口线建议串 100~330Ω 限流电阻，长线用双绞/屏蔽。

### 4.3 树莓派系统配置

```ini
# /boot/firmware/config.txt（Pi 4）
dtoverlay=uart3          # GPIO4/5，给 STM32
dtoverlay=uart4          # GPIO8/9，给 D6 激光
# enable_uart=1 只在使用 GPIO14/15 主串口时才需要，这里可不加
# 注意：GPIO8=SPI0 CE0、GPIO9=SPI0 MISO，确认没有开 spi；GPIO4 默认是 1-Wire，确认没开 w1-gpio
```

```bash
# 设备名以实际为准（Pi 4 上通常是 ttyAMA2 / ttyAMA3）
ls -l /dev/ttyAMA* /dev/serial*
# 关掉串口控制台（如果用 GPIO14/15 做主 UART 才需要）
sudo systemctl disable --now serial-getty@ttyAMA0.service
```

建议用 udev 固定名字，避免重启后设备号变化：

```bash
# /etc/udev/rules.d/99-rush-mog.rules
KERNEL=="ttyAMA2", SYMLINK+="rush_base"
KERNEL=="ttyAMA3", SYMLINK+="rush_lidar"
```

并把用户加入 `dialout` 组：`sudo usermod -aG dialout $USER`（重新登录生效）。

### 4.4 edge_server 配置要同步改的地方

| 文件/字段 | 现在 | 改成 |
|---|---|---|
| `edge_server/config/robot.yaml` → `base.port` | `/dev/ttyUSB0` | `/dev/rush_base`（UART3） |
| `base.baud` | `9600` | `115200` ✅（固件 `BODY_UART_BAUD_RATE` 已同步） |
| `imu.enabled` | `false` | `true`（IMU 接 Pi I2C1） |
| `imu.i2c_bus` / `address` | `1` / `0x68` | 实物 SA0 接地为 `0x68`，接 3.3V 为 `0x69` |
| `perception.camera.fx/fy/cx/cy` | `0` | 棋盘格标定后填入 |
| `navigation.direct.holonomic` | `true` | 换 4 路独立驱动前先设 `false`（L298N 2 路使能做不了真全向） |

启动：`ros2 launch edge_server robot_bringup.launch.py start_imu:=true`。

---

## 5. 原理图（2026-11-13 刷新版）审查

### 5.1 已画出的连接

| 网络 | 连接 |
|---|---|
| 电机方向 | STM32 PA11/PA10/PA9/PA8 → L298N IN1~IN4；PB15/PB14/PB13/PB12 → IN5~IN8 |
| 电机使能 | STM32 PB0 → ENA+EnB；PB1 → EnC+EnD |
| 电机输出 | L298N OUT1~OUT8 → 4 个直流电机 |
| 电源 | 12V → L298N `Vss(9)`；GND → L298N GND |

### 5.2 还没画的连接（当前全部悬空）

| 器件 | 悬空引脚 | 说明 |
|---|---|---|
| J1 树莓派 4（40 pin） | 全部 | UART3/UART4/I2C1/GPIO27/5V/3V3 都还没接 |
| U1 ICM-20948 | SDA/SCL/INT/VDD/GND/… | 应接 Pi I2C1 GPIO2/3 |
| SW1~SW4 旋转编码器 | A/B/C | 应接 STM32 PA0~PA7（软件四倍频） |
| U2 LM393 ×2 | +/−/OUT | 悬崖传感器，输出接 Pi GPIO（建议 2~3 路） |
| 喇叭 | 1/2 | 与当前“音频已关闭”的固件一致，可删或保留 |

### 5.3 需要修正/确认的点

1. **12V 接到了 L298N `Vss(9)`，`Vs(4)` 悬空**。L298N 数据手册：`Vs`=电机电源（12V），
   `Vss`=逻辑 5V。若这是按“四路 L298N 模块”画的符号，请在符号里把 12V 输入改到 `Vs`，
   并把 `Vss` 接到模块的 5V（或注明由模块内部稳压供电），否则投板会烧逻辑。
2. **编码器没有电平处理**：PA0~PA7 是 TC 类型（非 5V 容忍），原理图里缺少分压电阻/电平转换。
3. **没有激光雷达、深度相机、电源转换、电池、急停、INA226 的符号**：这些目前还是外挂件，
   建议在原理图里补连接器（至少把 Pi 的 UART4/I2C1/GPIO27 引到排针）。
4. **`ago.kicad_pcb` 只有 50 字节的空骨架**：PCB 还没开始布线，当前只能用转接板/杜邦线。
5. **4 个编码器用了 `Device:RotaryEncoder` 符号**（A/B/C 三脚），实际 JGB37-520 是
   VCC/GND/A/B 四线，建议换成 4 pin 连接器符号，避免接线时接错电源。
6. **两个 LM393 符号重叠**：U2 出现了两次，且都在 `(43.18, 109.22)`，参考号也相同。
   删掉一个或改参考号/位置，否则 ERC/网表会出现重复位号。
7. **L298N 的 4 路使能只接了两路**（PB0→ENA+EnB、PB1→EnC+EnD），这与固件一致，
   但意味着左前/左后、右前/右后不能独立调速——和 §10.3 的限制同源，升级驱动时一起改。

---

## 6. 电源与安全

### 6.1 电源树（建议）

```text
12V 锂电池（3S/4S，≥3600mAh，带保护板）
 ├─ 保险丝/总开关
 ├─ 急停（常闭，切断电机驱动使能或 12V）
 ├─ 电机驱动 12V（L298N → 后续换 TB6612/BTS7960）
 ├─ 5V/5A UBEC → 树莓派 4B（单独一路，线径 ≥20AWG）
 │              ├→ D6 激光（5V，启动电流可达 1A 以上）
 │              └→ Astra Pro / USB 麦（经 Pi USB 时注意总电流）
 ├─ 6V/5A BEC → PCA9685 + MG996 舵机（独立，不与 Pi 共地噪声）
 └─ 3.3V 1A → STM32 / ICM-20948 / 编码器上拉（低噪声）
```

### 6.2 功耗估算（决定 5V 电源选型）

| 负载 | 典型 | 峰值 | 备注 |
|---|---|---|---|
| Pi 4B | 3~5 W（0.6~1.0 A） | ~1.5 A | USB 外设另算 |
| Astra Pro | ~2.5 W（0.5 A） | ~0.8 A | USB 供电 |
| D6 激光 | ~2 W（0.4 A） | >1 A 启动 | 不要从 Pi 的 5V 排针取电 |
| USB 麦 | ~0.5 W | 0.2 A | |
| STM32 + 逻辑 | ~0.5 W | 0.1 A | |
| **合计（5V 侧）** | **~8~10 W** | **~3.5 A** | **5V/3A 不够，建议 5V/5A** |

电池：12V 3600mAh ≈ 43Wh，整机平均 12W 左右 → 理论 3h，实际约 1.5~2h（含电机）。

### 6.3 安全件（目前全缺）

- **常闭急停**：直接断电机使能或 12V，同时把状态反馈到 MCU/Pi GPIO。
- **保险丝 + 防反接 + TVS**：电池入口。
- **INA226/INA219**：发布 `/battery_state`，低于阈值自动回充/停机。
- **看门狗**：Pi 侧节点心跳丢失 → STM32 0.5s 内回零（固件已有命令看门狗，需联调）。

---

## 7. ICM-20948 定位方案

### 7.1 接法（方案 A：接 Pi）

| 项 | 值 |
|---|---|
| 总线 | Pi I2C1：GPIO2 (SDA) / GPIO3 (SCL) |
| 地址 | SA0 接地 `0x68`；接 3.3V 为 `0x69` |
| 供电 | 3.3V（不要 5V），与 Pi 共地 |
| 上拉 | 模块自带 4.7k 即可；长线加双绞/屏蔽 |
| 中断 | INT 可选，当前 `imu_node` 轮询 100 Hz，不接也能跑 |

### 7.2 软件配置

```yaml
imu:
  enabled: true
  i2c_bus: 1
  address: "0x68"
  rate_hz: 100.0
  mount_yaw_rad: 0.0      # 安装偏航，实测填入
  mag_declination_rad: 0.0
```

```bash
sudo apt install python3-smbus2
ros2 launch edge_server robot_bringup.launch.py start_imu:=true
ros2 topic hz /imu/data     # 应约 100 Hz
```

### 7.3 安装与标定

- 安装在**底盘几何中心**、刚性固定；加橡胶减震柱可降低电机振动，但不要软到产生相对转动。
- 远离电机、电池、大电流线；磁力计尤其怕电机磁场。
- 静止采集 2~3 分钟做零偏；六面法做加速度计校准；磁力计做硬铁/软铁校准后填
  `imu.calibration.*`。
- 融合链路：编码器 `/odom`（20 Hz）+ IMU（100 Hz）→ `robot_localization` EKF →
  `odom→base_link`；激光 + 地图 → AMCL → `map→odom`。

### 7.4 供应风险

ICM-20948 已进入 NRND/替代周期。新设计可评估 ICM-42688-P + 磁力计（如 MMC5983MA）；
驱动接口已封装在 `edge_server/drivers/icm20948.py`，替换只需改寄存器映射。

参考：
- [TDK ICM-20948 Design Transition Guide](https://www.symmetryelectronics.com/blog/invensense-icm-20948-design-transition-guide/)
- [ICM-20948 NRND/替代说明](https://pcbsync.com/icm-20948/)

---

## 8. 激光雷达（D6）

### 8.1 接口与供电

| 项 | 要求 |
|---|---|
| 接口 | TTL UART（TX/RX/GND），接 Pi UART4 GPIO8/9 |
| 电平 | 3.3V TTL；若模块是 5V TTL 必须加电平转换 |
| 波特率 | 按说明书确认（D6 类产品常见 230400 或 921600），Pi UART4 两种都能跑 |
| 供电 | 5V，独立一路；启动电流可达 1A 以上，**不要从 Pi 5V 排针取电** |
| 数据 | 360°，≥10 Hz，量程按型号（建图建议 ≥12 m） |

### 8.2 驱动与 `/scan`

- 仓库里**没有**激光驱动，需另装 ROS2 驱动（如 `ldlidar_stl_ros2`，或厂家提供的 D6 ROS2 包），
  输出 `/scan`、frame_id `laser_link`。
- 若 D6 是自定义协议，需要写一个解析节点：串口读帧 → `sensor_msgs/LaserScan`。
- `edge_server` 已默认订阅 `/scan`（`sensor_topics.laser`），无需改代码。

### 8.3 安装与 TF

- 安装高度 15~25 cm，360° 无遮挡；注意**机械臂/相机支架不要挡扫描面**。
- 激光平面尽量低，能看到桌腿、墙脚；但太低会被麦轮/底盘挡。
- TF：`base_link → laser_link`（x/y/z + yaw）按实测填写，建议写进 launch 的
  `static_transform_publisher`；标定误差 1 cm/1° 都会让建图重影。

### 8.4 验收

```bash
ros2 topic hz /scan          # ≈10 Hz
ros2 run tf2_ros tf2_echo base_link laser_link
# rviz2 里 LaserScan 应 360° 完整、无固定盲区
```

---

## 9. 深度相机（Astra Pro）

### 9.1 用途定位

- **2D SLAM 建图不需要深度相机**：激光 + 编码器 + IMU 足够。
- 深度相机用于**目标接近/抓取**（`target_detector_node` 的 depth 分支）和低矮障碍补盲。
- 不要用 Pi 4B 跑 RGB-D SLAM（RTAB-Map 全分辨率很吃力）；要用就降到 320×240 / 5 Hz。

### 9.2 供电、带宽与驱动

| 项 | 注意 |
|---|---|
| USB | Astra Pro 是 USB 2.0 设备；插 Pi 4 的 **USB 3.0 口**（兼容 2.0，供电更足），不要经 Hub |
| 带宽 | 深度 640×480@30 + RGB 同开接近 USB 2.0 上限，降帧率/分辨率更稳 |
| 驱动 | Astra Pro 属老款结构光，官方 ROS2 `orbbec_camera` 主要覆盖 Astra 2 / Pro Plus / Gemini 系列；**先确认能出 `/camera/depth/image_raw`**，否则换 Astra Pro Plus / Gemini 2 / RealSense D435i |
| 供电 | 相机峰值 ~0.8A，计入 5V 预算 |

### 9.3 标定

- RGB 内参：棋盘格 → `perception.camera.fx/fy/cx/cy`。
- 深度-彩色外参：用厂家工具或 `camera_calibration`。
- TF：`base_link → camera_link`（含俯仰角），目标定位依赖它。

---

## 10. 编码器与轮速闭环

### 10.1 接线（已实现）

JGB37-520 霍尔编码器：VCC、GND、A、B。

| 轮子 | A | B | EXTI |
|---|---|---|---|
| 左前 FL | PA0 | PA1 | EXTI0 / EXTI1 |
| 右前 FR | PA2 | PA3 | EXTI2 / EXTI3 |
| 左后 RL | PA4 | PA5 | EXTI4 / EXTI9_5(5) |
| 右后 RR | PA6 | PA7 | EXTI9_5(6/7) |

- **PA0~PA7 不是 5V 容忍(FT)引脚**：编码器必须 3.3V 电平（3.3V 供电或分压/电平转换）。
- 编码器 EXTI 抢占优先级为 4（低于 USART3 的 3），避免边沿中断挤掉串口收数。
- 每路串 100~330Ω + 100nF 到地做 RC 滤波；长线用屏蔽线，屏蔽层单端接地。
- 编码器电源与电机电源分开，避免 PWM 噪声。

### 10.2 固件现状

- 软件四倍频（EXTI 双沿），`ENCODER_CPR = 1320`，轮径 96 mm。
- 50 ms 周期通过 USART3（**115200 8N1**，`BODY_UART_BAUD_RATE`）上报
  `$B,<seq>,<uptime>,<fl>,<fr>,<rl>,<rr>,<dist>\n`，一帧约 4~5 ms。
- `edge_server` 解析后发布 `/odom`（20 Hz，`base.odom_rate_hz`）。

### 10.3 标定与验收

| 参数 | 标定方法 |
|---|---|
| `wheel_radius` | 卷尺量直径/2，或推 1 m 数编码器 |
| `encoder_cpr` | 手动转一圈读计数（含减速比与 4 倍频） |
| `wheelbase_x` / `track_y` | 卷尺量；麦轮用 (L+W)/2 作旋转半径 |
| `sign_*` | 单轮正转，确认计数方向 |

验收：直行 1 m 误差 <2%；原地 360° yaw 误差 <3°；堵转/推车 0.5 s 内回零。

> ⚠️ L298N 只有 2 路使能：左前/左后共用 PB0，右前/右后共用 PB1。换 4 路独立驱动前，
> 不要开 `navigation.direct.holonomic`，否则横移指令跟踪会很差。

---

## 11. 采购清单（增量，按当前进度更新）

### 11.1 已购/已有

树莓派 4B、STM32F103C8T6、D6 激光、ICM-20948、Astra Pro、LM393 悬崖、底盘 + JGB37-520×4、
L298N、12V 3600mAh 电池、12V→5V 3A / 3.3V 1A、USB 麦、蓝牙音箱。

### 11.2 还要买（P0 优先）

| 物料 | 建议型号 | 数量 | 价格 | 备注 |
|---|---|---|---|---|
| 5V/5A UBEC | 5V 5A 降压模块（带使能/欠压保护） | 1 | ¥30~60 | Pi + 激光 + 相机 |
| 4 路电机驱动 | TB6612FNG×2 或 DRV8871×4 / BTS7960×2 | 1 套 | ¥40~120 | 4 路独立 PWM |
| 电平转换/分压 | 8 路双向电平转换板或电阻分压 | 1 | ¥10~20 | 编码器 5V→3.3V |
| 急停按钮 | 常闭，10A | 1 | ¥10~25 | 切电机使能/12V |
| 保险丝座 + 保险丝 | 15~20A | 1 套 | ¥10~20 | 电池入口 |
| INA226 模块 | I2C 0.1Ω 分流 | 1 | ¥15~30 | `/battery_state` |
| 支架 | 激光/IMU/相机 3D 打印/铝板 | 1 套 | ¥30~80 | 刚性、可复现 |
| 散热 | Pi 4 风扇/散热片 | 1 | ¥15~40 | 长时间建图 |
| 存储 | 64GB 高耐久 SD 或 SSD | 1 | ¥50~150 | 地图/rosbag |

### 11.3 可选（P2）

碰撞开关 ×2、第二/第三路悬崖、5GHz AP 或有线网卡、USB 声卡（替代蓝牙）、
备用 IMU（ICM-42688-P）、UPS/电量模块。

**最低可跑 SLAM 建图的增量预算：约 ¥200~400**（电源 + 电平 + 急停 + 支架；驱动升级另算 ¥40~120）。

---

## 12. 从现有车到目标车的改造顺序

1. **电源先改**：5V/5A UBEC + 保险丝 + 急停 + 共地；Pi 单独供电，先保证不掉电。
2. **编码器电平/滤波**：分压或 3.3V 供电；示波器/逻辑分析仪看 A/B 波形干净。
3. **串口提速（已完成）**：STM32 `BODY_UART_BAUD_RATE` 与 `robot.yaml` 都已改 115200，确认 `/odom` 20 Hz 不掉帧。
4. **接 IMU**：Pi I2C1 + `start_imu:=true`，`/imu/data` 100 Hz；做零偏/磁校准。
5. **接激光**：Pi UART4 + 驱动，`/scan` 10 Hz；标 `base_link→laser_link`。
6. **跑 EKF**：`/odom` + `/imu/data` → `odom→base_link`；推车 1 m 看轨迹。
7. **建图**：`slam_toolbox` 推车走一圈，保存地图；检查墙体重影/闭环。
8. **定位**：AMCL + 地图，设初始位姿，确认 `map→odom` 稳定。
9. **导航**：Nav2 或内置直航，先空载 1 m 到点，再加载。
10. **安全与升级**：悬崖/碰撞/电量接入；换 4 路独立驱动后再开麦轮全向。
11. **目标搜索**：相机 + 激光 + 目标检测，先静态目标，再动态目标。

每一步都用 `edge_server` 的接口验收，避免“全部接完再调试”。

---

## 13. 主要风险与对策

| 风险 | 表现 | 对策 |
|---|---|---|
| 5V 3A 不够 | Pi 重启/掉盘，地图丢失 | 5V/5A UBEC，Pi 独立一路；加电容 |
| 编码器 5V 直连 PA0~PA7 | 计数乱跳、IO 损坏、串口偶发异常 | 3.3V 供电或分压/电平转换 |
| 9600 带宽饱和（已改 115200） | `/odom` 抖动、命令延迟/丢失 | 固件与 `robot.yaml` 必须同时改；再降回 9600 会复现 |
| UART4 与 SPI0/1-Wire 复用 | 激光打不开/设备号变化 | 关 spi/w1 overlay；udev 固定设备名 |
| Astra Pro 驱动不支持 | 没有 `/camera/depth/image_raw` | 先验证；换 Astra Pro Plus/Gemini 2/D435i |
| 深度相机 USB 带宽/供电 | 掉帧、掉设备 | 插 USB3 口、不经 Hub、降分辨率 |
| L298N 压降/发热/只有 2 路 PWM | 速度不准、麦轮全向做不了 | 换 MOSFET 驱动；先关 holonomic |
| 激光被机械臂遮挡 | 建图出现固定缺口/重影 | 调整安装高度/位置；机械臂收拢 |
| IMU 振动/磁干扰 | yaw 跳变、EKF 发散 | 减震柱、远离电机、磁校准、mag 门控 |
| 麦轮打滑 | 里程计漂移 | 激光/AMCL 校正；轮速闭环；地面清理 |
| 建图时 Wi-Fi 丢包 | 远程看不到地图 | 建图在车上闭环；5GHz/有线；DDS 调参 |
| 地图过大 | 前沿提取慢 | `frontier_max_cells` 下采样；分区建图 |

---

## 14. 与 edge_server 的接口对应

| 硬件 | ROS2 话题/服务 | edge_server 节点 |
|---|---|---|
| D6 激光 | `/scan` | `bridge_node`、`safety_manager_node`、`navigation_node` |
| 编码器/底盘 | `/odom`、`/joint_states`、`/base/status` | `edge_server_node`（`base_bridge_node` 为旧方案） |
| ICM-20948 | `/imu/data` | `imu_node`（Pi I2C） |
| 电量 | `/battery_state` | `edge_server_node`（接 INA226 后） |
| 地图 | `/map`、`/center/map_info`、`/center/frontiers` | `map_manager_node` |
| 定位 | `/amcl_pose`、TF、`/center/pose`、`/center/localization_status` | `localization_node` |
| 导航 | `/center/navigate_to`、`/nav/cmd_vel` | `navigation_node` |
| 目标 | `/camera/image_raw`、`/center/target_detections` | `target_detector_node` |
| 搜索 | `/center/search_object`、`/center/explore` | `object_search_node` |
| 速度 | `/cmd_vel_raw`、`/nav/cmd_vel`、`/reverse/cmd_vel`、`/safety/cmd_vel_override` → `/cmd_vel` | `velocity_mux_node` |
| 任务 | `/center/execute_task`、`/center/execute_mission` | `task_manager_node` |

启动与配置文件：

```bash
ros2 launch edge_server robot_bringup.launch.py start_imu:=true
# 激光驱动、EKF、slam_toolbox、Nav2 另行 launch（见 software/edge_server/launch/）
```

详细软件设计见 [`software/edge_server/DESIGN.md`](../software/edge_server/DESIGN.md)。
