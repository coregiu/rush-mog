#!/usr/bin/env python3
"""D6/LD19 激光雷达串口诊断工具（在树莓派上运行）。

用途：
- 在多个候选串口/波特率上读原始字节，判断“没数据 / 波特率不对 / 协议不对”；
- 解析 47 字节 LD19 帧，统计合法包与 CRC 错误；
- `--send` 可发送启动命令，或配合 GPIO8↔GPIO9 短接做 UART 回环自检。

用法（树莓派，/opt/edge_server）：
    sudo python3 tools/lidar_probe.py
    sudo python3 tools/lidar_probe.py --ports /dev/rush_lidar --bauds 230400,921600
    sudo python3 tools/lidar_probe.py --send "a5 40"          # 少数雷达需要启动命令
    sudo python3 tools/lidar_probe.py --send 55aa55aa          # GPIO8/9 短接回环自检

接线（Raspberry Pi 4B，BCM 编号）：
    GPIO8  = 物理 pin 24 = UART4 TXD -> 雷达 RX（可不接）
    GPIO9  = 物理 pin 21 = UART4 RXD -> 雷达 TX（必须接）
    GND    = 物理 pin 6/9/14/20/25/30/34/39 任选，必须共地
    5V     = 物理 pin 2/4（按雷达要求，D6 通常 5V/1A）
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

DEFAULT_PORTS = (
    "/dev/rush_lidar",
    "/dev/ttyAMA2",
    "/dev/ttyUSB0",
    "/dev/ttyACM0",
)
DEFAULT_BAUDS = (230400, 921600, 115200, 460800, 128000)


def _prepare_import() -> None:
    repo = Path(__file__).resolve().parents[2]
    candidates = (
        repo / "software" / "edge_server",
        Path("/opt/edge_server/src/edge_server"),
    )
    for path in candidates:
        text = str(path)
        if path.is_dir() and text not in sys.path:
            sys.path.insert(0, text)


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D6/LD19 lidar UART probe")
    parser.add_argument(
        "--ports",
        default=",".join(DEFAULT_PORTS),
        help="逗号分隔的串口候选（默认 /dev/rush_lidar,/dev/ttyAMA2,/dev/ttyUSB0,/dev/ttyACM0）",
    )
    parser.add_argument(
        "--bauds",
        default=",".join(str(item) for item in DEFAULT_BAUDS),
        help="逗号分隔的波特率候选（默认 230400,921600,115200,460800,128000）",
    )
    parser.add_argument("--seconds", type=float, default=1.5, help="每个组合读取秒数")
    parser.add_argument("--send", default="", help="打开串口后发送的十六进制字节，如 'a5 40'")
    parser.add_argument("--list", action="store_true", help="只列出存在的串口")
    return parser.parse_args()


def _hexdump(data: bytes, limit: int = 32) -> str:
    return " ".join(f"{byte:02x}" for byte in data[:limit])


def _probe(port: str, baud: int, seconds: float, send: bytes) -> dict:
    import serial

    started = time.time()
    with serial.Serial(port, baud, timeout=0.1, write_timeout=0.2) as handle:
        if send:
            handle.write(send)
            handle.flush()
            time.sleep(0.05)
        data = bytearray()
        while time.time() - started < seconds:
            data.extend(handle.read(4096))

    from edge_server.core.lidar import PACKET_SIZE, parse_packet

    frames = 0
    valid = 0
    bad = 0
    index = 0
    while index + 1 < len(data):
        if data[index] == 0x54 and data[index + 1] == 0x2C:
            frames += 1
            packet = parse_packet(bytes(data[index : index + PACKET_SIZE]))
            if packet is not None:
                valid += 1
                index += PACKET_SIZE
                continue
            bad += 1
        index += 1
    return {
        "port": port,
        "baud": baud,
        "bytes": len(data),
        "frames": frames,
        "valid": valid,
        "bad": bad,
        "sample": _hexdump(bytes(data)),
    }


def main() -> int:
    args = _args()
    ports = [item.strip() for item in args.ports.split(",") if item.strip()]
    bauds = [int(item) for item in args.bauds.split(",") if item.strip()]
    existing = [port for port in ports if os.path.exists(port)]

    if args.list:
        print("候选串口：")
        for port in ports:
            print(("  OK   " if port in existing else "  MISS ") + port)
        return 0 if existing else 1

    print("== D6/LD19 lidar probe ==")
    print("候选串口：", ", ".join(existing) if existing else "（都不存在）")
    if not existing:
        print("请检查：dtoverlay=uart4、dtparam=spi=off、udev 规则 /dev/rush_lidar、接线")
        return 1

    _prepare_import()
    try:
        from edge_server.core.lidar import parse_packet  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        print(f"无法导入 edge_server.core.lidar: {exc}")
        print("请在 /opt/edge_server 下运行，或设置 PYTHONPATH=/opt/edge_server/src/edge_server")
        return 2

    send = bytes.fromhex(args.send.replace(" ", "")) if args.send else b""
    if send:
        print(f"发送字节：{send.hex(' ')}")

    success = False
    for port in existing:
        for baud in bauds:
            try:
                result = _probe(port, baud, max(0.3, args.seconds), send)
            except PermissionError:
                print(f"{port}@{baud}: 权限不足，请用 sudo 或加入 dialout 组")
                continue
            except Exception as exc:  # noqa: BLE001
                print(f"{port}@{baud}: ERROR {type(exc).__name__}: {exc}")
                continue
            print(
                f"{port}@{baud}: bytes={result['bytes']} frames={result['frames']} "
                f"valid={result['valid']} bad={result['bad']} sample={result['sample']}"
            )
            if result["valid"] > 0:
                success = True
                print(f"  -> 找到有效 LD19/D6 数据：{port} @ {baud}")
            elif result["bytes"] == 0:
                print("  -> 没有收到任何字节：优先检查供电/共地/雷达 TX→GPIO9/波特率")
            elif result["frames"] == 0:
                print("  -> 有字节但没有 0x54 0x2C 帧头：波特率或协议不匹配")

    if not success:
        print(
            "\n未解析到有效数据。建议：\n"
            "1) 确认雷达电机在转、5V/GND 正常、雷达 TX 接 GPIO9(物理 pin21)、共地；\n"
            "2) 拔掉雷达，短接 GPIO8(pin24) 与 GPIO9(pin21)，执行：\n"
            "   sudo python3 tools/lidar_probe.py --ports /dev/rush_lidar --bauds 115200 --send 55aa55aa\n"
            "   若看到 4 字节回环，说明 Pi UART4 正常，问题在雷达/接线；\n"
            "3) 换波特率（--bauds 230400,921600）或确认是否需要 --send 启动命令。"
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
