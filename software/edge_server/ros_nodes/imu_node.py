"""imu_node：树莓派 I2C 直连 ICM-20948，发布 sensor_msgs/Imu。

若 IMU 接在 STM32 上，请不要启动本节点，改用 base_bridge_node 解析串口 IMU 帧。
节点启动时若 IMU 未接/未上电，会按 ``imu.retry_sec`` 周期重试初始化；接好后
无需重启节点即可开始发布 ``/imu/data``。
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

from edge_server.drivers.icm20948 import Calibration, ICM20948
from edge_server.utils.clock import create_steady_clock
from edge_server.utils.config import load_config


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.cfg = load_config(self)
        imu_cfg = self.cfg.get("imu", {})
        self.imu_cfg = dict(imu_cfg)
        self.enabled = bool(imu_cfg.get("enabled", True))
        self.frame_id = str(imu_cfg.get("frame_id", "imu_link"))
        self.rate = max(1.0, float(imu_cfg.get("rate_hz", 100.0)))
        self.retry_sec = max(0.5, float(imu_cfg.get("retry_sec", 2.0)))
        self.error_count = 0
        self.driver = None
        self._last_init_at = 0.0
        self._last_init_log_at = 0.0

        self.pub = self.create_publisher(Imu, "/imu/data", 20)
        if not self.enabled:
            self.get_logger().warn("imu_node disabled by config (imu.enabled=false)")
            return

        self._steady_clock = create_steady_clock()
        self.timer = self.create_timer(
            1.0 / self.rate, self.on_timer, clock=self._steady_clock
        )
        self._try_init()

    def _try_init(self):
        """打开 I2C 并初始化 IMU；失败时保留节点并按 retry_sec 重试。"""
        if self.driver is not None:
            return
        now = time.monotonic()
        if (now - self._last_init_at) < self.retry_sec:
            return
        self._last_init_at = now

        bus = None
        try:
            import smbus2

            bus = smbus2.SMBus(int(self.imu_cfg.get("i2c_bus", 1)))
            driver = ICM20948(
                bus,
                address=int(str(self.imu_cfg.get("address", "0x68")), 0),
                accel_fs=int(self.imu_cfg.get("accel_fs", 1)),
                gyro_fs=int(self.imu_cfg.get("gyro_fs", 2)),
                dlpf=int(self.imu_cfg.get("dlpf", 3)),
                sample_rate_hz=self.rate,
                calibration=Calibration.from_config(
                    self.imu_cfg.get("calibration", {})
                ),
            )
            driver.begin()
        except ImportError:
            if (now - self._last_init_log_at) > 10.0:
                self._last_init_log_at = now
                self.get_logger().error(
                    "smbus2 not installed; install python3-smbus2 or use base_bridge_node"
                )
            if bus is not None:
                try:
                    bus.close()
                except Exception:  # noqa: BLE001
                    pass
            return
        except Exception as exc:  # noqa: BLE001
            if bus is not None:
                try:
                    bus.close()
                except Exception:  # noqa: BLE001
                    pass
            if (now - self._last_init_log_at) > 10.0:
                self._last_init_log_at = now
                self.get_logger().warn(
                    f"ICM-20948 not ready at I2C-{self.imu_cfg.get('i2c_bus', 1)} "
                    f"{self.imu_cfg.get('address', '0x68')}: {exc}; retrying"
                )
            return

        self.driver = driver
        self.get_logger().info(
            f"imu_node started: rate={self.rate:.0f}Hz frame={self.frame_id} "
            f"mag={'ok' if self.driver.mag_available else 'unavailable'}"
        )

    def on_timer(self):
        if self.driver is None:
            self._try_init()
            return
        try:
            sample = self.driver.read_sample()
        except Exception as exc:  # noqa: BLE001
            self.error_count += 1
            if self.error_count % 20 == 1:
                self.get_logger().warn(f"ICM-20948 read error: {exc}")
            self.driver.close()
            self.driver = None
            self._last_init_at = time.monotonic()
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # 不输出姿态估计：由 robot_localization/EKF 融合原始数据
        msg.orientation_covariance[0] = -1.0
        (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ) = (float(v) for v in sample.gyro)
        (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ) = (float(v) for v in sample.accel)
        # Imu.magnetic_field 单位 Tesla；AK09916 原始单位 uT
        (
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ) = (float(v) * 1e-6 for v in sample.mag)
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 0.01
            msg.linear_acceleration_covariance[i] = 0.05
            msg.magnetic_field_covariance[i] = 0.01
        self.pub.publish(msg)

    def destroy_node(self):
        if self.driver is not None:
            self.driver.close()
            self.driver = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
