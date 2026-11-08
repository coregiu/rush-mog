"""栅格地图表示、前沿提取与 map_server 兼容的 PGM/YAML 存取。

center_server 不直接实现 SLAM，而是消费 slam_toolbox / cartographer / map_server
发布的 nav_msgs/OccupancyGrid。本模块负责：

- 把 OccupancyGrid 转成 numpy，计算覆盖率；
- 提取 frontier（free/unknown 边界）供探索使用；
- 在 slam_toolbox 不可用时，用 PGM+YAML 自行保存/加载地图（与 map_server 兼容）；
- 提供目标点可通行性检查与就近自由点搜索。
"""

from __future__ import annotations

import math
import os
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .pose_utils import Pose2D, wrap_angle


@dataclass
class Frontier:
    """一个前沿聚类。position 在 map 系。"""

    x: float
    y: float
    size: int = 0
    distance: float = 0.0

    @property
    def position(self) -> Tuple[float, float]:
        return float(self.x), float(self.y)


@dataclass
class OccupancyMap:
    width: int = 0
    height: int = 0
    resolution: float = 0.05
    origin: Pose2D = field(default_factory=lambda: Pose2D(0.0, 0.0, 0.0))
    data: Optional[np.ndarray] = None  # int8, shape=(height, width), -1 unknown
    frame_id: str = "map"
    map_id: str = ""
    map_file: str = ""
    _frontiers: List[Frontier] = field(default_factory=list, repr=False)

    # ------------------------------------------------------------------ basics
    @property
    def empty(self) -> bool:
        return self.data is None or self.data.size == 0

    @property
    def total_cells(self) -> int:
        return 0 if self.data is None else int(self.data.size)

    @property
    def known_ratio(self) -> float:
        if self.empty:
            return 0.0
        return float(np.count_nonzero(self.data >= 0) / self.data.size)

    @property
    def free_ratio(self) -> float:
        if self.empty:
            return 0.0
        return float(np.count_nonzero(self.data == 0) / self.data.size)

    @property
    def occupied_ratio(self) -> float:
        if self.empty:
            return 0.0
        return float(np.count_nonzero(self.data >= 50) / self.data.size)

    @property
    def frontier_count(self) -> int:
        return len(self._frontiers)

    @property
    def frontiers(self) -> List[Frontier]:
        return list(self._frontiers)

    def set_data(
        self,
        data,
        width: int,
        height: int,
        resolution: float,
        origin: Pose2D,
        frame_id: str = "map",
        map_id: str = "",
    ):
        arr = np.asarray(data, dtype=np.int8)
        if arr.size != int(width) * int(height):
            raise ValueError(
                f"map data size {arr.size} != {width}*{height}"
            )
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin = origin
        self.data = arr.reshape(self.height, self.width)
        self.frame_id = frame_id or "map"
        if map_id:
            self.map_id = map_id
        self._frontiers = []

    @classmethod
    def from_msg(cls, msg) -> "OccupancyMap":
        info = msg.info
        origin = info.origin
        yaw = math.atan2(
            2.0 * (origin.orientation.w * origin.orientation.z),
            1.0 - 2.0 * (origin.orientation.z * origin.orientation.z),
        )
        obj = cls()
        obj.set_data(
            msg.data,
            info.width,
            info.height,
            info.resolution,
            Pose2D(origin.position.x, origin.position.y, yaw, msg.header.frame_id or "map"),
            frame_id=msg.header.frame_id or "map",
        )
        return obj

    def to_msg(self, stamp=None):
        """构造 nav_msgs/OccupancyGrid。空地图返回 1x1 全 unknown。"""
        from nav_msgs.msg import OccupancyGrid

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id or "map"
        if stamp is not None:
            msg.header.stamp = stamp
        if self.empty:
            msg.info.width = 1
            msg.info.height = 1
            msg.info.resolution = self.resolution or 0.05
            msg.data = [-1]
            return msg
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.resolution = self.resolution
        msg.info.origin.position.x = self.origin.x
        msg.info.origin.position.y = self.origin.y
        msg.info.origin.orientation.z = math.sin(self.origin.yaw / 2.0)
        msg.info.origin.orientation.w = math.cos(self.origin.yaw / 2.0)
        msg.data = self.data.astype(np.int8).reshape(-1).tolist()
        return msg

    # --------------------------------------------------------------- geometry
    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.empty:
            return None
        dx, dy = float(x) - self.origin.x, float(y) - self.origin.y
        c, s = math.cos(self.origin.yaw), math.sin(self.origin.yaw)
        gx = c * dx + s * dy
        gy = -s * dx + c * dy
        col = int(math.floor(gx / self.resolution))
        row = int(math.floor(gy / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        gx = (float(col) + 0.5) * self.resolution
        gy = (float(row) + 0.5) * self.resolution
        c, s = math.cos(self.origin.yaw), math.sin(self.origin.yaw)
        return self.origin.x + c * gx - s * gy, self.origin.y + s * gx + c * gy

    def cell_value(self, x: float, y: float) -> Optional[int]:
        cell = self.world_to_grid(x, y)
        if cell is None:
            return None
        col, row = cell
        return int(self.data[row, col])

    def is_free(self, x: float, y: float, unknown_is_free: bool = False) -> bool:
        value = self.cell_value(x, y)
        if value is None:
            return False
        if value < 0:
            return bool(unknown_is_free)
        return value == 0

    def is_occupied(self, x: float, y: float) -> bool:
        value = self.cell_value(x, y)
        return value is not None and value >= 50

    def is_inside(self, x: float, y: float) -> bool:
        return self.world_to_grid(x, y) is not None

    def find_free_near(
        self,
        x: float,
        y: float,
        radius_m: float = 0.5,
        step_m: Optional[float] = None,
        unknown_is_free: bool = False,
    ) -> Optional[Pose2D]:
        """在 (x,y) 附近螺旋搜索最近的自由栅格，返回其世界坐标。"""
        if self.empty:
            return None
        step = max(float(step_m or self.resolution), self.resolution)
        max_rings = int(max(1.0, radius_m / step))
        for ring in range(max_rings + 1):
            if ring == 0:
                candidates = [(x, y)]
            else:
                candidates = []
                r = ring * step
                n = max(8, ring * 8)
                for i in range(n):
                    a = 2.0 * math.pi * i / n
                    candidates.append((x + r * math.cos(a), y + r * math.sin(a)))
            for cx, cy in candidates:
                if self.is_free(cx, cy, unknown_is_free=unknown_is_free):
                    return Pose2D(cx, cy, 0.0, self.frame_id)
        return None

    # -------------------------------------------------------------- frontiers
    def compute_frontiers(
        self,
        min_size: int = 5,
        max_count: int = 50,
        max_cells: int = 250_000,
        neighbor_unknown_only: bool = True,
    ) -> List[Frontier]:
        """提取 free/unknown 边界并做 8 邻域聚类。

        大图会先按 step 下采样以控制 Python BFS 开销；前沿尺寸按 step^2 还原。
        """
        if self.empty:
            self._frontiers = []
            return []
        data = self.data
        h, w = data.shape
        step = max(1, int(math.ceil(math.sqrt(max(1, h * w) / float(max_cells)))))
        small = data[::step, ::step]
        free = small == 0
        unknown = small < 0
        if not free.any() or not unknown.any():
            self._frontiers = []
            return []

        # 邻域 unknown 检测（8 邻域，边界用 False 填充）
        padded = np.pad(unknown, 1, mode="constant", constant_values=False)
        near_unknown = np.zeros_like(unknown, dtype=bool)
        sh, sw = unknown.shape
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                near_unknown |= padded[1 + dy : 1 + dy + sh, 1 + dx : 1 + dx + sw]
        frontier = free & near_unknown
        # 只保留与自由空间相邻的 unknown 边界；上面已经是 free 与 unknown 的边界
        if not frontier.any():
            self._frontiers = []
            return []

        rows, cols = np.nonzero(frontier)
        frontier_cells = set(zip(rows.tolist(), cols.tolist()))
        visited = set()
        clusters: List[List[Tuple[int, int]]] = []
        for start in frontier_cells:
            if start in visited:
                continue
            queue = deque([start])
            visited.add(start)
            cluster = []
            while queue:
                r, c = queue.popleft()
                cluster.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nb = (r + dr, c + dc)
                        if nb in frontier_cells and nb not in visited:
                            visited.add(nb)
                            queue.append(nb)
            clusters.append(cluster)

        result: List[Frontier] = []
        for cluster in clusters:
            size = len(cluster) * step * step
            if size < int(min_size):
                continue
            mean_r = sum(p[0] for p in cluster) / len(cluster)
            mean_c = sum(p[1] for p in cluster) / len(cluster)
            col = int(min(self.width - 1, max(0, mean_c * step + step // 2)))
            row = int(min(self.height - 1, max(0, mean_r * step + step // 2)))
            wx, wy = self.grid_to_world(col, row)
            result.append(Frontier(wx, wy, size))
        result.sort(key=lambda f: f.size, reverse=True)
        result = result[: int(max_count)]
        self._frontiers = result
        return list(result)

    def select_frontier(
        self,
        pose: Optional[Pose2D],
        frontiers: Optional[Sequence[Frontier]] = None,
        blacklist: Iterable[Tuple[float, float, float]] = (),
        now: float = 0.0,
        min_size: int = 5,
        max_distance: float = 20.0,
        blacklist_radius: float = 0.8,
    ) -> Optional[Frontier]:
        """按 size/(distance+0.5) 选择前沿；blacklist 元素为 (x, y, until)。"""
        items = list(frontiers if frontiers is not None else self._frontiers)
        active_blacklist = [
            (float(bx), float(by))
            for bx, by, until in blacklist
            if float(until) > float(now)
        ]
        best: Optional[Frontier] = None
        best_score = -1.0
        for item in items:
            if item.size < int(min_size):
                continue
            if pose is not None:
                dist = math.hypot(item.x - pose.x, item.y - pose.y)
                item.distance = dist
                if max_distance > 0.0 and dist > float(max_distance):
                    continue
            else:
                dist = 0.0
            if any(
                math.hypot(item.x - bx, item.y - by) <= blacklist_radius
                for bx, by in active_blacklist
            ):
                continue
            score = float(item.size) / (dist + 0.5)
            if score > best_score:
                best_score = score
                best = item
        return best

    # ----------------------------------------------------------- persistence
    def save(self, path: str) -> str:
        """保存为 map_server 兼容的 PGM+YAML，返回 YAML 路径。"""
        if self.empty:
            raise ValueError("cannot save an empty map")
        path = os.path.abspath(path)
        if path.lower().endswith((".yaml", ".yml")):
            yaml_path = path
            stem = os.path.splitext(path)[0]
        else:
            stem = path
            yaml_path = path + ".yaml"
        pgm_path = stem + ".pgm"
        os.makedirs(os.path.dirname(yaml_path) or ".", exist_ok=True)

        # map_server 的像素值：0 占据，205 unknown，254 自由；图像行 0 在顶部
        image = np.full((self.height, self.width), 205, dtype=np.uint8)
        image[self.data == 0] = 254
        image[self.data >= 50] = 0
        image[np.logical_and(self.data > 0, self.data < 50)] = 254
        image = image[::-1, :]

        with open(pgm_path, "wb") as f:
            f.write(b"P5\n# CREATOR: rush-mog center_server\n")
            f.write(f"{self.width} {self.height}\n255\n".encode("ascii"))
            f.write(image.tobytes())

        import yaml

        meta = {
            "image": os.path.basename(pgm_path),
            "resolution": float(self.resolution),
            "origin": [float(self.origin.x), float(self.origin.y), float(self.origin.yaw)],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }
        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)
        self.map_file = yaml_path
        return yaml_path

    @classmethod
    def load(cls, path: str) -> "OccupancyMap":
        """从 YAML 或 PGM 加载地图。"""
        path = os.path.abspath(path)
        if path.lower().endswith((".yaml", ".yml")):
            import yaml

            with open(path, "r", encoding="utf-8") as f:
                meta = yaml.safe_load(f) or {}
            image_path = meta.get("image", "")
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(path), image_path)
            image = _read_pgm(image_path)
            resolution = float(meta.get("resolution", 0.05))
            origin_data = meta.get("origin", [0.0, 0.0, 0.0])
            origin = Pose2D(
                float(origin_data[0]),
                float(origin_data[1]),
                float(origin_data[2]) if len(origin_data) > 2 else 0.0,
                "map",
            )
            map_file = path
        else:
            image = _read_pgm(path)
            resolution = 0.05
            origin = Pose2D(0.0, 0.0, 0.0, "map")
            map_file = path

        if image.ndim != 2:
            raise ValueError(f"unsupported map image shape: {image.shape}")
        # PGM 行 0 在顶部 -> 地图行 0 在底部
        image = image[::-1, :]
        data = np.full(image.shape, -1, dtype=np.int8)
        data[image >= 250] = 0
        data[image <= 5] = 100
        obj = cls()
        obj.set_data(
            data,
            image.shape[1],
            image.shape[0],
            resolution,
            origin,
            frame_id="map",
            map_id=os.path.splitext(os.path.basename(map_file))[0],
        )
        obj.map_file = map_file
        return obj


def _skip_pgm_space_and_comments(buf: bytes, idx: int) -> int:
    n = len(buf)
    while idx < n:
        ch = buf[idx : idx + 1]
        if ch in b" \t\r\n":
            idx += 1
            continue
        if ch == b"#":
            while idx < n and buf[idx : idx + 1] not in b"\r\n":
                idx += 1
            continue
        break
    return idx


def _read_pgm_token(buf: bytes, idx: int) -> Tuple[bytes, int]:
    idx = _skip_pgm_space_and_comments(buf, idx)
    start = idx
    while idx < len(buf) and buf[idx : idx + 1] not in b" \t\r\n":
        idx += 1
    return buf[start:idx], idx


def _read_pgm(path: str) -> np.ndarray:
    """读取 P2/P5 PGM，返回 uint8 (height, width)。"""
    with open(path, "rb") as f:
        buf = f.read()
    magic, idx = _read_pgm_token(buf, 0)
    if magic not in (b"P2", b"P5"):
        raise ValueError(f"not a PGM file: {path}")
    width_tok, idx = _read_pgm_token(buf, idx)
    height_tok, idx = _read_pgm_token(buf, idx)
    maxval_tok, idx = _read_pgm_token(buf, idx)
    width, height, maxval = int(width_tok), int(height_tok), int(maxval_tok)
    if maxval <= 0 or maxval > 255:
        raise ValueError(f"unsupported PGM maxval: {maxval}")
    if magic == b"P5":
        # 头部后必须有一个空白字符
        if idx < len(buf) and buf[idx : idx + 1] in b" \t\r\n":
            idx += 1
        raster = buf[idx : idx + width * height]
        if len(raster) < width * height:
            raise ValueError("truncated PGM raster")
        arr = np.frombuffer(raster, dtype=np.uint8).copy()
    else:
        values = []
        while len(values) < width * height:
            tok, idx = _read_pgm_token(buf, idx)
            if not tok:
                break
            values.append(int(tok))
        if len(values) < width * height:
            raise ValueError("truncated P2 raster")
        arr = np.asarray(values, dtype=np.uint8)
    return arr.reshape(height, width)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    """返回 (z, w)，仅供内部构造 origin 使用。"""
    return math.sin(wrap_angle(yaw) / 2.0), math.cos(wrap_angle(yaw) / 2.0)
