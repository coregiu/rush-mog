"""硬件驱动适配层（可选）。

- icm20948: 树莓派 I2C 直连 ICM-20948 的寄存器级驱动；
- 若 IMU 接在 STM32 上，则由 base_bridge_node 解析串口 IMU 帧，不需要本驱动。
"""
