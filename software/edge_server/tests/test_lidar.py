"""LD19/D6 协议解析与扫描累积测试（不依赖串口/ROS）。"""

import math

from edge_server.core.lidar import (
    PACKET_SIZE,
    ScanAccumulator,
    crc8,
    parse_packet,
)

SAMPLE_HEX = (
    "54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 "
    "E9 CD 00 E4 CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50"
)


def make_packet(start_deg, end_deg, distances=None, confidence=200):
    distances = list(distances or [1000 + index * 10 for index in range(12)])
    frame = bytearray(PACKET_SIZE)
    frame[0] = 0x54
    frame[1] = 0x2C
    frame[2:4] = (3000).to_bytes(2, "little")
    frame[4:6] = int(round(start_deg * 100)).to_bytes(2, "little")
    for index in range(12):
        offset = 6 + index * 3
        frame[offset : offset + 2] = int(distances[index]).to_bytes(2, "little")
        frame[offset + 2] = int(confidence)
    frame[42:44] = int(round(end_deg * 100)).to_bytes(2, "little")
    frame[44:46] = (1234).to_bytes(2, "little")
    frame[46] = crc8(bytes(frame[:46]))
    return bytes(frame)


def test_parse_sample_packet():
    data = bytes.fromhex(SAMPLE_HEX)
    assert len(data) == PACKET_SIZE
    assert crc8(data[:46]) == data[46]
    packet = parse_packet(data)
    assert packet is not None
    assert packet.speed_dps == 0x0868
    assert abs(packet.start_angle_deg - 324.27) < 0.01
    assert abs(packet.end_angle_deg - 334.70) < 0.01
    assert len(packet.points) == 12
    assert abs(packet.points[0].distance_m - 0.224) < 1e-6
    assert packet.points[0].confidence == 0xE4
    assert abs(packet.points[0].angle_deg - 324.27) < 0.01


def test_parse_rejects_bad_crc_and_header():
    data = bytearray.fromhex(SAMPLE_HEX)
    data[-1] ^= 0x01
    assert parse_packet(bytes(data)) is None
    assert parse_packet(b"") is None
    data = bytearray.fromhex(SAMPLE_HEX)
    data[0] = 0x55
    assert parse_packet(bytes(data)) is None


def test_accumulator_wrap_and_binning():
    accumulator = ScanAccumulator(bins=360, min_points=12, range_max_m=12.0)
    assert accumulator.add_packet(parse_packet(make_packet(0, 30))) is None
    assert accumulator.add_packet(parse_packet(make_packet(30, 60))) is None
    assert accumulator.add_packet(parse_packet(make_packet(330, 359))) is None
    scan = accumulator.add_packet(parse_packet(make_packet(0, 30)))
    assert scan is not None
    assert scan.points == 36
    assert len(scan.ranges) == 360
    assert len(scan.intensities) == 360
    assert math.isclose(scan.angle_increment, math.radians(1.0), rel_tol=1e-6)
    # angle=0 在 -180..180 的 bin 180；距离 1.0m，置信度 200。
    assert math.isclose(scan.ranges[180], 1.0, rel_tol=1e-6)
    assert math.isclose(scan.intensities[180], 200.0 / 255.0, rel_tol=1e-6)


def test_accumulator_filters_out_of_range():
    accumulator = ScanAccumulator(
        bins=360, min_points=12, range_min_m=0.10, range_max_m=5.0
    )
    accumulator.add_packet(parse_packet(make_packet(0, 30, [10] * 12)))
    accumulator.add_packet(parse_packet(make_packet(30, 60, [9000] * 12)))
    accumulator.add_packet(parse_packet(make_packet(330, 359, [0] * 12)))
    scan = accumulator.add_packet(parse_packet(make_packet(0, 30, [0] * 12)))
    assert scan is not None
    assert all(math.isinf(value) for value in scan.ranges)
