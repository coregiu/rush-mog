"""ROS2 定时器时钟工具。

控制/安全循环应使用单调时钟（STEADY_TIME），避免系统墙钟被 NTP 调整或
虚拟机时间同步回拨时导致定时器长时间不触发。
"""

from rclpy.clock import Clock, ClockType


def create_steady_clock() -> Clock:
    return Clock(clock_type=ClockType.STEADY_TIME)
