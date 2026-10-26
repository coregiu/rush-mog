

# Rush Mog

[中文版](README_cn.md)

Rush Mog is an intelligent omnidirectional wheel car controlled by VLA (Vision-Language-Action), inspired by the Unimog. Based on ESP32 + STM32F103 + L298N + OV2640 + LiDAR, it supports three control modes: a mobile phone Wi-Fi PS2 emulator, voice control, and VLA intelligent control. It also supports robotic arm operations and audio playback. Additionally, it provides action programming, allowing users to customize combined action commands.

## Features
#### (1) Mobile Phone PS2 Emulator for Car Control
Use your phone as a PS2 emulator to control the car to move forward, backward, turn left, turn right, strafe left, strafe right, move diagonally forward-left, forward-right, backward-left, and backward-right.

#### (2) Camera Monitoring & Automatic Object Recognition
Monitor and automatically identify and classify objects via the onboard camera. View the live feed in real-time on the mobile phone PS2 controller interface.

#### (3) Voice Control for Car Movement
Use the voice control button on the mobile PS2 controller to command the car to move forward, backward, turn left, turn right, strafe left, strafe right, and move in diagonal directions.

#### (4) Audio Playback
Play V8 engine sounds through the onboard MP3 player for a thrilling audio experience.

#### (5) Action Programming
A programming interface is provided to script audio effects and car movements, enabling various choreographed sequences.

#### (6) Automatic Target Seeking
Automatically identify and classify objects using the onboard camera and LiDAR. Search for and lock onto target objects, and track moving targets.

#### (7) Object Handling
Use the VLA model to control car movement, enabling searching, locking onto, and tracking target objects. Use the robotic arm to grasp, transfer, and place objects into designated areas. The transport path can be pre-planned or automatically learned.


## Design Overview
#### Overall System Architecture
![](images/system-arch.png)

Design principles:
- 1 ESP32 implements a web server, providing the PS2 control page and voice input.
- 2 ESP32 receives PS2 commands and directly forwards them to STM32 for execution.
- 3 ESP32 receives voice commands, recognizes them, converts them into instructions, and forwards them to STM32.
- 4 ESP32 interfaces with the OV2640 camera and LiDAR, running a lightweight VLA model for inference. The inference results are sent as commands to STM32.
- 5 STM32 acts as the car's control center, translating operation commands into specific GPIO levels or PWM signals to drive the L298N motor driver or the robotic arm.

#### Hardware Architecture
- DIY car using 97mm omnidirectional wheels with JGB37-520 motors, driven by an L298N driver board (4WD, or two 2WD boards).
- Car body control uses an STM32F103C8T6 board.
- Wi-Fi and VLA model inference use an ESP32-S3-CAM + OV2640 camera.

![](hardware/integrate_archetecture.png)

For details, see: [Hardware Design Notes](hardware/README.md)

#### Software Architecture
- The software is divided into the vehicle control subsystem and the remote control center.
- The vehicle control subsystem runs on the STM32F103 board, driving the car motors, robotic arm, and audio output. It receives and executes commands from the remote control subsystem.
- The remote control subsystem runs on the ESP32-S3-CAM, interfacing with the camera and LiDAR, providing a web server, video surveillance streaming to the terminal, and speech recognition for converting voice into commands.

![](software/architechture.png)

For details, see: [Software Design Notes and Code](software/README.md)

## How to Use the Controller
![](software/remote_center/ps2.png)

After starting the car, connect your phone to the RUSH-MOG Wi-Fi network created by the ESP32-S3-CAM. Once connected, open your phone's browser and go to 192.168.4.1 to access the PS2 controller page and operate the car.

## Final Product Showcase

![](images/AGO.png)

## Tips & Cautions
1 Do not place the microphone on the car. The car's motors are loud when running, making voice recognition difficult.
