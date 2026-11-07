"""Raspberry Pi / center_server <-> STM32 底盘串口协议（v1）。

帧格式（小端）：

    magic(2) | version(1) | type(1) | seq(1) | len(1) | payload(len) | crc16(2)

- magic: 0xAA 0x55
- version: 1
- crc16: CRC-16/CCITT-FALSE（poly=0x1021, init=0xFFFF），覆盖 version..payload
- 所有多字节整数小端

主机 -> STM32：
    0x01 CMD_VEL   <hhB>  vx(mm/s), wz(mrad/s), flags
    0x02 MODE      <BB>   mode(0 manual / 1 auto), flags
    0x03 PING      <>     空

STM32 -> 主机：
    0x81 TELEMETRY <I4i4hHhBI> seq, enc[4], wheel[4](mm/s), battery_mv,
                                current_ma, flags, uptime_ms
    0x82 IMU       <I3h3h3hI>  seq, accel(mg)[3], gyro(0.1dps)[3],
                                mag(0.1uT)[3], stamp_us
    0x83 ACK       <B>         ack_type
    0x84 STATUS    <B>         status flags

现有 body_control 固件使用 ASCII 命令（'1'..'9' + PWM 档位），没有编码器/
IMU 回传。要实现 Nav2 闭环，需要按本协议升级固件；协议解析已做向后兼容的
分帧/CRC 校验，坏帧会被丢弃并重新同步。
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

MAGIC = b"\xAA\x55"
VERSION = 1
MAX_PAYLOAD = 64

TYPE_CMD_VEL = 0x01
TYPE_MODE = 0x02
TYPE_PING = 0x03
TYPE_TELEMETRY = 0x81
TYPE_IMU = 0x82
TYPE_ACK = 0x83
TYPE_STATUS = 0x84

MODE_MANUAL = 0
MODE_AUTO = 1
MODE_ESTOP = 2

FLAG_ESTOP = 1 << 0
FLAG_ARMED = 1 << 1
FLAG_IMU_OK = 1 << 2
FLAG_ENCODER_OK = 1 << 3
FLAG_BATTERY_LOW = 1 << 4

_HEADER = struct.Struct("<BBBB")  # version, type, seq, len
_CRC = struct.Struct("<H")
_CMD_VEL = struct.Struct("<hhB")
_MODE = struct.Struct("<BB")
_TELEMETRY = struct.Struct("<I4i4hHhBI")
_IMU = struct.Struct("<I3h3h3hI")


class ProtocolError(ValueError):
    pass


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE。"""
    crc = init & 0xFFFF
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class Frame:
    version: int
    msg_type: int
    seq: int
    payload: bytes


@dataclass
class Telemetry:
    seq: int = 0
    encoders: Tuple[int, int, int, int] = (0, 0, 0, 0)
    wheel_speeds: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    battery_mv: int = 0
    current_ma: int = 0
    flags: int = 0
    uptime_ms: int = 0

    @property
    def estop(self) -> bool:
        return bool(self.flags & FLAG_ESTOP)

    @property
    def armed(self) -> bool:
        return bool(self.flags & FLAG_ARMED)

    @property
    def battery_low(self) -> bool:
        return bool(self.flags & FLAG_BATTERY_LOW)


@dataclass
class ImuSample:
    seq: int = 0
    accel: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # m/s^2
    gyro: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # rad/s
    mag: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # uT
    stamp_us: int = 0


def encode_frame(msg_type: int, payload: bytes = b"", seq: int = 0, version: int = VERSION) -> bytes:
    if not (0 <= int(msg_type) <= 0xFF):
        raise ProtocolError(f"invalid msg_type: {msg_type}")
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload too large: {len(payload)}")
    header = _HEADER.pack(int(version) & 0xFF, int(msg_type) & 0xFF, int(seq) & 0xFF, len(payload))
    body = header + payload
    return MAGIC + body + _CRC.pack(crc16_ccitt(body))


def decode_stream(buffer: bytearray) -> List[Frame]:
    """从可变字节缓冲中取出完整帧，原地删除已消费字节。"""
    frames: List[Frame] = []
    while True:
        idx = buffer.find(MAGIC)
        if idx < 0:
            # 保留可能的半个 magic
            if buffer.endswith(b"\xAA"):
                del buffer[:-1]
            else:
                buffer.clear()
            break
        if idx > 0:
            del buffer[:idx]
        if len(buffer) < 2 + _HEADER.size:
            break
        version, msg_type, seq, length = _HEADER.unpack_from(buffer, 2)
        if version != VERSION or length > MAX_PAYLOAD:
            del buffer[0]
            continue
        total = 2 + _HEADER.size + length + _CRC.size
        if len(buffer) < total:
            break
        payload = bytes(buffer[2 + _HEADER.size : 2 + _HEADER.size + length])
        expected = _CRC.unpack_from(buffer, 2 + _HEADER.size + length)[0]
        actual = crc16_ccitt(bytes(buffer[2 : 2 + _HEADER.size + length]))
        if actual != expected:
            del buffer[0]
            continue
        frames.append(Frame(version, msg_type, seq, payload))
        del buffer[:total]
    return frames


def _clamp_i16(value: float) -> int:
    return int(max(-32768, min(32767, round(float(value)))))


def encode_cmd_vel(vx: float, wz: float, estop: bool = False, seq: int = 0) -> bytes:
    vx_mm_s = _clamp_i16(float(vx) * 1000.0)
    wz_mrad_s = _clamp_i16(float(wz) * 1000.0)
    flags = FLAG_ESTOP if estop else FLAG_ARMED
    return encode_frame(TYPE_CMD_VEL, _CMD_VEL.pack(vx_mm_s, wz_mrad_s, flags), seq=seq)


def encode_mode(mode: int, estop: bool = False, seq: int = 0) -> bytes:
    flags = FLAG_ESTOP if estop else FLAG_ARMED
    return encode_frame(TYPE_MODE, _MODE.pack(int(mode) & 0xFF, flags), seq=seq)


def encode_ping(seq: int = 0) -> bytes:
    return encode_frame(TYPE_PING, b"", seq=seq)


def decode_telemetry(payload: bytes) -> Telemetry:
    if len(payload) != _TELEMETRY.size:
        raise ProtocolError(f"telemetry length {len(payload)} != {_TELEMETRY.size}")
    seq, e0, e1, e2, e3, w0, w1, w2, w3, battery_mv, current_ma, flags, uptime_ms = _TELEMETRY.unpack(payload)
    return Telemetry(
        seq=int(seq),
        encoders=(int(e0), int(e1), int(e2), int(e3)),
        wheel_speeds=(w0 / 1000.0, w1 / 1000.0, w2 / 1000.0, w3 / 1000.0),
        battery_mv=int(battery_mv),
        current_ma=int(current_ma),
        flags=int(flags),
        uptime_ms=int(uptime_ms),
    )


def decode_imu(payload: bytes) -> ImuSample:
    if len(payload) != _IMU.size:
        raise ProtocolError(f"imu length {len(payload)} != {_IMU.size}")
    seq, ax, ay, az, gx, gy, gz, mx, my, mz, stamp_us = _IMU.unpack(payload)
    return ImuSample(
        seq=int(seq),
        accel=(
            ax / 1000.0 * 9.80665,
            ay / 1000.0 * 9.80665,
            az / 1000.0 * 9.80665,
        ),
        gyro=(
            math.radians(gx / 10.0),
            math.radians(gy / 10.0),
            math.radians(gz / 10.0),
        ),
        mag=(mx / 10.0, my / 10.0, mz / 10.0),
        stamp_us=int(stamp_us),
    )


def decode_frame(frame: Frame):
    """按类型解码；未知类型返回 None。"""
    if frame.msg_type == TYPE_TELEMETRY:
        return decode_telemetry(frame.payload)
    if frame.msg_type == TYPE_IMU:
        return decode_imu(frame.payload)
    if frame.msg_type == TYPE_ACK and frame.payload:
        return int(frame.payload[0])
    return None
