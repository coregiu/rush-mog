# Rush Mog

[中文版](README_cn.md)

Rush Mog is an intelligent omnidirectional-wheel car inspired by the Unimog and controlled by VLA (Vision-Language-Action). Built on a Raspberry Pi + depth camera + STM32F103 + L298N + IMU sensor + LiDAR + robotic arm + VLA inference, it supports three control modes — a phone-based Wi-Fi PS2 emulator, voice control, and VLA intelligent control — together with robotic-arm operation. It also provides action programming, allowing users to define combined action commands.

## Features
#### (1) Control the Car with a Phone-Based PS2 Emulator
Use a phone as a PS2 emulator to drive the car forward, backward, turn left, turn right, strafe left, strafe right, and move diagonally forward-left, forward-right, backward-left, and backward-right.

#### (2) Camera Monitoring and Automatic Object Recognition
Monitor, automatically identify, and classify objects through the onboard camera, and view the live feed in real time on the phone-based PS2 controller interface.

#### (3) SLAM Mapping, Autonomous Navigation, and Automatic Target Seeking
Build a 2D occupancy grid with a 2D LiDAR + wheel encoders + ICM-20948, and localize with AMCL/EKF. Navigate to goals with Nav2 or the built-in direct controller. Search for target objects using camera detection + LiDAR ranging + frontier exploration, return the target pose, and approach it while maintaining a configured standoff distance. Map save/load, mapping missions, exploration missions, and target-search missions are supported.

#### (4) Object Handling
Use the VLA model to control vehicle movement: search for, lock onto, and track target objects. Use the robotic arm to grasp, transfer, and place objects within a designated area, implementing object transport. Transport paths can be planned or learned automatically.

#### (5) Digital Twin Monitoring and Configuration
Open `http://192.168.4.1/dashboard.html` (or tap **RUSH-MOG TWIN** on the controller page) to see an in-vehicle digital twin with EDGE / ROS / CENTER module status, per-device health (camera, LiDAR, IMU, microphone, speaker, arm, base, battery), and runtime parameter editing. ROS 2 and CENTER report to `192.168.4.1`; the page only talks to the Raspberry Pi Edge Server, which proxies module APIs.


## Design Overview
#### Overall System Architecture
![](images/system-arch.png)

**Core modules:**
| Module | Location | Function |
| --- | --- | --- |
| **Edge Server** | Onboard module | Based on a Raspberry Pi 4B: serves the operator web UI, PS2-emulator/voice driving, BodyControl UART bridge, camera MJPEG, vehicle configuration/status, and ROS2 SLAM/navigation/search. Manual control and the web UI remain available even if ROS nodes are not running. |
| **BodyControl** | Onboard module | Based on the STM32F103C8T6, connects the L298N to drive the car's JGB37-520 motors, and reads the motor encoders for EKF pose estimation. |
| **Union Power Supply** | Onboard module | Provides stable 12V / 5V / 3.3V rails; pure hardware, no software module. |
| **Center Server** | Center server module | Provides VLA inference capability and high-level decision-making compute for the vehicle. |
| **Terminal** | User terminal | A phone, tablet, or PC; opens the Edge Server operator web UI in a browser. |


**Module context and APIs:**
| Interface | Protocol | Client | Server | Function |
| --- | --- | --- | --- | --- |
| **Host interface** | http | Terminal | Edge Server | PS2-emulator driving, vehicle speed and odometry status, vehicle configuration and status monitoring. The unified external control entry point for the car. |
| **Vehicle control interface** | uart | Edge Server | BodyControl | Executes car motion commands; reads car speed and odometry status. |
| **Center Server interface** | http | Edge Server / ROS | Center Server | Provides VLA inference execution. |

> ROS 2 runs on the same Raspberry Pi Edge Server; its health gateway `:8090` is proxied by the Edge Server for the dashboard. See [HEALTHCHECK.md](software/edge_server/HEALTHCHECK.md).

#### Hardware Architecture
- DIY car built with 97 mm omnidirectional wheels and JGB37-520 motors (with hall encoders), driven by an L298N driver board (4WD, or two 2WD boards). For navigation, a 4-channel independent PWM driver (TB6612/DRV8871/BTS7960) is recommended.
- Car body control uses an STM32F103C8T6 board; F103RCT6/F411 is recommended for extra encoder/PWM headroom.
- Wi-Fi AP and video are handled by the Raspberry Pi Edge Server (hostapd + ROS camera).
- SLAM/navigation use a 2D LiDAR (LD19/LD06 or RPLIDAR A1) + ICM-20948 9-axis IMU, connected to the onboard Linux computer (the IMU can also connect to the STM32).
- A Raspberry Pi 4B/5 or Jetson Orin Nano must be onboard to run ROS2/slam_toolbox/Nav2.

![](hardware/integrate_archetecture.png)

For details, see: [Hardware Design Notes](hardware/README.md) and [SLAM/Navigation Hardware Plan](hardware/SLAM_NAV.md).

#### Software Architecture
- The software is divided into a vehicle control subsystem and a center inference/navigation subsystem.
- The vehicle control subsystem runs on the STM32 board, driving the car motors, robotic arm, and audio output, and executes commands received from the onboard computer/bridge.
- The onboard Linux subsystem runs ROS2 drivers, `edge_server_node`, ICM-20948, EKF, slam_toolbox, Nav2, target detection, and target search.
- The onboard ROS package `edge_server` (`software/edge_server`) adds SLAM map management, localization health gating, autonomous navigation, target detection/search/approach, velocity arbitration, and the STM32 serial bridge. See [ROS package design](software/edge_server/DESIGN.md).
- `edge_server_node` on the Pi serves the web UI/API (`:80`), MJPEG (`:8080`), the STM32 ASCII UART bridge, and proxies ROS health; `health_monitor_node` exposes ROS and per-device health over HTTP (`:8090/ros/health`); see [healthcheck contract](software/edge_server/HEALTHCHECK.md).
- The independently deployed [Center Server SmolVLA inference service](software/center_server/README.md) provides VLA inference to ROS over HTTP; see the [service design](software/center_server/DESIGN.md).

![](software/architechture.png)

For details, see: [Software Design Notes and Code](software/README.md).

## How to Use the Controller
![](software/edge_server/ps2.png)

After starting the car, connect your phone to the RUSH-MOG Wi-Fi network provided by the Pi Edge Server. Open your phone's browser at 192.168.4.1 to load the PS2 controller page and operate the car.

## Final Product Showcase

![](images/AGO.png)
