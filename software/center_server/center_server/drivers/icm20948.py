"""ICM-20948 9 轴 IMU 寄存器级驱动（I2C，AK09916 磁力计 bypass 访问）。

设计要点：
- 总线对象可注入（默认 smbus2.SMBus），便于无硬件单元测试；
- 只用 Bank0 数据寄存器 + Bank2 配置寄存器，避免依赖 DMP 固件；
- 磁力计通过 INT_PIN_CFG.BYPASS_EN 直连 AK09916（地址 0x0C）；
- 输出单位：加速度 m/s^2，角速度 rad/s，磁感应强度 uT，温度 degC。

注意：ICM-20948 已进入 NRND/替代周期，新设计可评估 ICM-42688-P + 磁力计
组合；但本驱动接口保持通用，替换芯片只需改寄存器映射。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

# ---------------------------------------------------------------- register map
REG_BANK_SEL = 0x7F
REG_WHO_AM_I = 0x00
REG_USER_CTRL = 0x03
REG_PWR_MGMT_1 = 0x06
REG_PWR_MGMT_2 = 0x07
REG_INT_PIN_CFG = 0x0D
REG_INT_ENABLE = 0x0E
REG_CONFIG = 0x1A

# Bank 0 数据寄存器
REG_ACCEL_XOUT_H = 0x2D  # 0x2D..0x32 accel, 0x33..0x38 gyro, 0x39/0x3A temp
REG_TEMP_OUT_H = 0x39

# Bank 2 配置寄存器
REG_GYRO_SMPLRT_DIV = 0x00
REG_GYRO_CONFIG_1 = 0x01
REG_GYRO_CONFIG_2 = 0x02
REG_ACCEL_SMPLRT_DIV_1 = 0x10
REG_ACCEL_SMPLRT_DIV_2 = 0x11
REG_ACCEL_CONFIG = 0x14
REG_ACCEL_CONFIG_2 = 0x15

WHO_AM_I_VALUE = 0xEA

# AK09916（磁力计）
AK09916_ADDR = 0x0C
AK_REG_WIA2 = 0x01
AK_REG_CNTL2 = 0x0B
AK_REG_CNTL3 = 0x0C
AK_REG_ST1 = 0x10
AK_WIA2_VALUE = 0x09
AK_MODE_CONT_100HZ = 0x08
AK_UT_PER_LSB = 0.15

ACCEL_FS = {0: 2.0, 1: 4.0, 2: 8.0, 3: 16.0}  # g
ACCEL_LSB_PER_G = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}
GYRO_FS = {0: 250.0, 1: 500.0, 2: 1000.0, 3: 2000.0}  # dps
GYRO_LSB_PER_DPS = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}

GRAVITY = 9.80665
DEG_TO_RAD = math.pi / 180.0


@dataclass
class ImuReading:
    accel: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # m/s^2
    gyro: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # rad/s
    mag: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # uT
    temp_c: float = 0.0
    timestamp: float = 0.0
    mag_valid: bool = False


@dataclass
class Calibration:
    accel_bias: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_bias: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    mag_bias: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    mag_scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)

    @classmethod
    def from_config(cls, cfg: Optional[dict]) -> "Calibration":
        cfg = cfg or {}

        def vec(key, default):
            value = cfg.get(key, default)
            if not isinstance(value, (list, tuple)) or len(value) != 3:
                return tuple(default)
            return tuple(float(v) for v in value)

        return cls(
            accel_bias=vec("accel_bias", (0.0, 0.0, 0.0)),
            gyro_bias=vec("gyro_bias", (0.0, 0.0, 0.0)),
            mag_bias=vec("mag_bias", (0.0, 0.0, 0.0)),
            mag_scale=vec("mag_scale", (1.0, 1.0, 1.0)),
        )


class ICM20948:
    def __init__(
        self,
        bus,
        address: int = 0x68,
        accel_fs: int = 1,
        gyro_fs: int = 2,
        dlpf: int = 3,
        sample_rate_hz: float = 100.0,
        calibration: Optional[Calibration] = None,
    ):
        self.bus = bus
        self.address = int(address)
        self.accel_fs = int(accel_fs) if int(accel_fs) in ACCEL_FS else 1
        self.gyro_fs = int(gyro_fs) if int(gyro_fs) in GYRO_FS else 2
        self.dlpf = max(0, min(7, int(dlpf)))
        self.sample_rate_hz = max(1.0, float(sample_rate_hz))
        self.calibration = calibration or Calibration()
        self._bank = -1
        self._mag_ok = False

    # -------------------------------------------------------------- low level
    def _set_bank(self, bank: int):
        bank = int(bank) & 0x03
        if bank == self._bank:
            return
        self.bus.write_byte_data(self.address, REG_BANK_SEL, bank << 4)
        self._bank = bank

    def _write(self, reg: int, value: int, bank: int = 0):
        self._set_bank(bank)
        self.bus.write_byte_data(self.address, int(reg), int(value) & 0xFF)

    def _read(self, reg: int, length: int, bank: int = 0) -> bytes:
        self._set_bank(bank)
        data = self.bus.read_i2c_block_data(self.address, int(reg), int(length))
        return bytes(bytearray(data))

    def _read_u8(self, reg: int, bank: int = 0) -> int:
        return self._read(reg, 1, bank)[0]

    # AK09916 通过 ICM 的 I2C bypass 出现在同一总线的 0x0C
    def _ak_write(self, reg: int, value: int):
        self._set_bank(0)
        self.bus.write_byte_data(AK09916_ADDR, int(reg), int(value) & 0xFF)

    def _ak_read(self, reg: int, length: int) -> bytes:
        self._set_bank(0)
        data = self.bus.read_i2c_block_data(AK09916_ADDR, int(reg), int(length))
        return bytes(bytearray(data))

    # ------------------------------------------------------------------- setup
    def begin(self) -> bool:
        who = self._read_u8(REG_WHO_AM_I, bank=0)
        if who != WHO_AM_I_VALUE:
            raise RuntimeError(
                f"ICM-20948 WHO_AM_I=0x{who:02X}, expected 0x{WHO_AM_I_VALUE:02X}"
            )
        # 复位并唤醒
        self._write(REG_PWR_MGMT_1, 0x80, bank=0)
        time.sleep(0.1)
        self._write(REG_PWR_MGMT_1, 0x01, bank=0)  # auto clock
        time.sleep(0.05)
        self._write(REG_PWR_MGMT_2, 0x00, bank=0)  # enable accel + gyro
        self._write(REG_USER_CTRL, 0x00, bank=0)   # disable I2C master
        self._write(REG_INT_PIN_CFG, 0x02, bank=0)  # BYPASS_EN -> AK09916 visible
        self._write(REG_INT_ENABLE, 0x00, bank=0)

        # 采样率：gyro 1.125kHz / (1+div)
        div = max(0, int(round(1125.0 / self.sample_rate_hz)) - 1)
        div = min(div, 4095)
        self._write(REG_GYRO_SMPLRT_DIV, div & 0xFF, bank=2)
        self._write(REG_ACCEL_SMPLRT_DIV_1, (div >> 8) & 0x0F, bank=2)
        self._write(REG_ACCEL_SMPLRT_DIV_2, div & 0xFF, bank=2)
        self._write(
            REG_GYRO_CONFIG_1, (self.dlpf << 3) | (self.gyro_fs << 1), bank=2
        )
        self._write(
            REG_ACCEL_CONFIG, (self.dlpf << 3) | (self.accel_fs << 1), bank=2
        )
        self._write(REG_ACCEL_CONFIG_2, 0x00, bank=2)
        self._write(REG_CONFIG, 0x00, bank=0)

        self._init_mag()
        return True

    def _init_mag(self):
        try:
            # bypass 需要 I2C master 关闭；已在 begin 中清 USER_CTRL
            self._ak_write(AK_REG_CNTL3, 0x01)
            time.sleep(0.02)
            wia = self._ak_read(AK_REG_WIA2, 1)[0]
            if wia != AK_WIA2_VALUE:
                self._mag_ok = False
                return
            self._ak_write(AK_REG_CNTL2, AK_MODE_CONT_100HZ)
            time.sleep(0.02)
            self._mag_ok = True
        except Exception:  # noqa: BLE001 - 磁力计缺失不应阻止 IMU 工作
            self._mag_ok = False

    # -------------------------------------------------------------------- read
    def read_sample(self) -> ImuReading:
        data = self._read(REG_ACCEL_XOUT_H, 14, bank=0)
        ax, ay, az = _s16(data[0], data[1]), _s16(data[2], data[3]), _s16(data[4], data[5])
        gx, gy, gz = _s16(data[6], data[7]), _s16(data[8], data[9]), _s16(data[10], data[11])
        temp_raw = _s16(data[12], data[13])

        accel_lsb = ACCEL_LSB_PER_G[self.accel_fs]
        gyro_lsb = GYRO_LSB_PER_DPS[self.gyro_fs]
        bias = self.calibration
        accel = (
            ax / accel_lsb * GRAVITY - bias.accel_bias[0],
            ay / accel_lsb * GRAVITY - bias.accel_bias[1],
            az / accel_lsb * GRAVITY - bias.accel_bias[2],
        )
        gyro = (
            gx / gyro_lsb * DEG_TO_RAD - bias.gyro_bias[0],
            gy / gyro_lsb * DEG_TO_RAD - bias.gyro_bias[1],
            gz / gyro_lsb * DEG_TO_RAD - bias.gyro_bias[2],
        )
        temp_c = temp_raw / 333.87 + 21.0
        mag, mag_valid = self._read_mag()
        return ImuReading(
            accel=accel,
            gyro=gyro,
            mag=mag,
            temp_c=temp_c,
            timestamp=time.monotonic(),
            mag_valid=mag_valid,
        )

    def _read_mag(self):
        if not self._mag_ok:
            return (0.0, 0.0, 0.0), False
        try:
            raw = self._ak_read(AK_REG_ST1, 9)
        except Exception:  # noqa: BLE001
            self._mag_ok = False
            return (0.0, 0.0, 0.0), False
        st1, st2 = raw[0], raw[8]
        if not (st1 & 0x01) or (st2 & 0x08):
            return (0.0, 0.0, 0.0), False
        mx = _s16(raw[1], raw[2]) * AK_UT_PER_LSB
        my = _s16(raw[3], raw[4]) * AK_UT_PER_LSB
        mz = _s16(raw[5], raw[6]) * AK_UT_PER_LSB
        cal = self.calibration
        mag = (
            (mx - cal.mag_bias[0]) * cal.mag_scale[0],
            (my - cal.mag_bias[1]) * cal.mag_scale[1],
            (mz - cal.mag_bias[2]) * cal.mag_scale[2],
        )
        return mag, True

    def close(self):
        try:
            self.bus.close()
        except Exception:  # noqa: BLE001
            pass

    @property
    def mag_available(self) -> bool:
        return self._mag_ok


def _s16(high: int, low: int) -> int:
    value = (int(high) << 8) | int(low)
    return value - 65536 if value & 0x8000 else value
