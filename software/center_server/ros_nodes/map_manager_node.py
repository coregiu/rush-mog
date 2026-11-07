"""map_manager_node：SLAM 地图消费、前沿提取、地图存取与建图控制。

- 订阅 /map（slam_toolbox / cartographer / map_server 均可）；
- 周期发布 /center/map_info、/center/frontiers；
- 提供 /center/map_control（start/stop/save/load/reset）；
- slam_toolbox 可用时优先调用其 save/serialize/deserialize/reset 服务；
- 不可用时用内置 OccupancyMap 直接写 map_server 兼容的 PGM+YAML，并可在
  load 后自行发布 /map 供 AMCL/Nav2 使用。
"""

from __future__ import annotations

import math
import os
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from center_server.core.occupancy_map import OccupancyMap
from center_server.utils.clock import create_steady_clock
from center_server.utils.config import load_config
from custom_msgs.msg import Frontier, FrontierList, MapInfo
from custom_msgs.srv import MapControl

try:  # slam_toolbox 是可选的：只影响“优先用 slam_toolbox 存图”
    from slam_toolbox.srv import DeserializeMap, Reset, SaveMap, SerializeMap
except ImportError:  # pragma: no cover - 基础 Humble 环境没有 slam_toolbox
    DeserializeMap = SaveMap = SerializeMap = Reset = None


class MapManagerNode(Node):
    def __init__(self):
        super().__init__("map_manager_node")
        self.cfg = load_config(self)
        cfg = self.cfg.get("map_manager", {})
        self.map_frame = str(cfg.get("map_frame", "map"))
        self.map_dir = os.path.expanduser(
            str(cfg.get("map_dir", "~/.ros/rush_mog/maps"))
        )
        self.publish_rate = max(0.2, float(cfg.get("publish_rate", 1.0)))
        self.frontier_min_size = int(cfg.get("frontier_min_size", 5))
        self.frontier_max_count = int(cfg.get("frontier_max_count", 50))
        self.frontier_max_cells = int(cfg.get("frontier_max_cells", 250_000))
        self.default_map_name = str(cfg.get("default_map_name", "") or "")
        self.prefer_slam_toolbox = bool(cfg.get("prefer_slam_toolbox", True))
        self.map_id = str(cfg.get("map_id", "") or "")

        self.occupancy = OccupancyMap(frame_id=self.map_frame, map_id=self.map_id)
        self.mapping = False
        self.status = "idle"
        self.last_map_time = 0.0
        self.map_file = ""
        self._frontier_dirty = False
        self._last_publish = 0.0
        self._cb_group = ReentrantCallbackGroup()

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # 订阅用 VOLATILE 以兼容 slam_toolbox（VOLATILE）和 map_server
        # （TRANSIENT_LOCAL）两种发布端。
        map_sub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.pub_info = self.create_publisher(MapInfo, "/center/map_info", 10)
        self.pub_frontiers = self.create_publisher(
            FrontierList, "/center/frontiers", 10
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.on_map,
            map_sub_qos,
            callback_group=self._cb_group,
        )
        self.create_service(
            MapControl,
            "/center/map_control",
            self.on_map_control,
            callback_group=self._cb_group,
        )

        self.slam_save = None
        self.slam_serialize = None
        self.slam_deserialize = None
        self.slam_reset = None
        if self.prefer_slam_toolbox and SaveMap is not None:
            self.slam_save = self.create_client(
                SaveMap, "/slam_toolbox/save_map", callback_group=self._cb_group
            )
            self.slam_serialize = self.create_client(
                SerializeMap,
                "/slam_toolbox/serialize_map",
                callback_group=self._cb_group,
            )
            self.slam_deserialize = self.create_client(
                DeserializeMap,
                "/slam_toolbox/deserialize_map",
                callback_group=self._cb_group,
            )
            self.slam_reset = self.create_client(
                Reset, "/slam_toolbox/reset", callback_group=self._cb_group
            )
            self.get_logger().info(
                "slam_toolbox services available; map save/load prefers them"
            )
        else:
            self.get_logger().info(
                "slam_toolbox services unavailable; using built-in PGM/YAML backend"
            )

        self._steady_clock = create_steady_clock()
        self.create_timer(
            1.0 / self.publish_rate,
            self.on_publish_timer,
            clock=self._steady_clock,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"map_manager_node started (map_dir={self.map_dir}, "
            f"prefer_slam_toolbox={self.prefer_slam_toolbox and SaveMap is not None})"
        )

    # ------------------------------------------------------------- callbacks
    def on_map(self, msg: OccupancyGrid):
        try:
            self.occupancy = OccupancyMap.from_msg(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to parse /map: {exc}")
            return
        self.occupancy.frame_id = self.map_frame
        if not self.occupancy.map_id:
            self.occupancy.map_id = self.map_id or self._new_map_id()
        self.map_id = self.occupancy.map_id
        self.last_map_time = time.monotonic()
        self._frontier_dirty = True
        if self.mapping:
            self.status = "mapping"

    def on_map_control(self, request, response):
        command = (request.command or "").strip().lower()
        name = (request.map_name or "").strip()
        try:
            if command == "start":
                self.mapping = True
                self.status = "mapping"
                if request.reset_pose:
                    self._call_slam_reset()
                response.success = True
                response.msg = "mapping started"
            elif command == "stop":
                self.mapping = False
                self.status = "idle"
                response.success = True
                response.msg = "mapping stopped"
            elif command == "save":
                ok, msg, path = self._save_map(name)
                response.success = ok
                response.msg = msg
                response.map_file = path
                self.map_file = path
            elif command == "load":
                ok, msg, path = self._load_map(name)
                response.success = ok
                response.msg = msg
                response.map_file = path
                self.map_file = path
            elif command == "reset":
                self.occupancy = OccupancyMap(frame_id=self.map_frame)
                self.status = "idle"
                self._frontier_dirty = False
                self._call_slam_reset()
                response.success = True
                response.msg = "map cleared"
            else:
                response.success = False
                response.msg = f"unknown command: {command!r}"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"map control {command} failed: {exc}")
            response.success = False
            response.msg = f"{command} failed: {exc}"
        self.publish_map_info()
        return response

    # --------------------------------------------------------------- publish
    def on_publish_timer(self):
        if self._frontier_dirty and not self.occupancy.empty:
            self.occupancy.compute_frontiers(
                min_size=self.frontier_min_size,
                max_count=self.frontier_max_count,
                max_cells=self.frontier_max_cells,
            )
            self._frontier_dirty = False
        self.publish_map_info()
        self.publish_frontiers()

    def publish_map_info(self):
        info = MapInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = self.map_frame
        info.map_id = self.occupancy.map_id or self.map_id
        info.status = self.status
        info.resolution = float(self.occupancy.resolution)
        info.width = int(self.occupancy.width)
        info.height = int(self.occupancy.height)
        origin = self.occupancy.origin
        info.origin.position.x = origin.x
        info.origin.position.y = origin.y
        info.origin.orientation.z = math.sin(origin.yaw / 2.0)
        info.origin.orientation.w = math.cos(origin.yaw / 2.0)
        info.known_ratio = float(self.occupancy.known_ratio)
        info.free_ratio = float(self.occupancy.free_ratio)
        info.frontier_count = int(self.occupancy.frontier_count)
        info.map_file = self.map_file
        self.pub_info.publish(info)

    def publish_frontiers(self):
        msg = FrontierList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.map_id = self.occupancy.map_id or self.map_id
        for frontier in self.occupancy.frontiers:
            item = Frontier()
            item.position.x = float(frontier.x)
            item.position.y = float(frontier.y)
            item.position.z = 0.0
            item.size = float(frontier.size)
            item.distance = float(frontier.distance)
            msg.frontiers.append(item)
        self.pub_frontiers.publish(msg)

    # ------------------------------------------------------------------ save
    def _save_map(self, name: str):
        if self.occupancy.empty:
            return False, "no map data to save", ""
        name = name or self.default_map_name or self._new_map_id()
        path = self._resolve_path(name)
        if self.slam_save is not None and self.prefer_slam_toolbox:
            try:
                req = SaveMap.Request()
                req.name = path
                result = self._call_service(self.slam_save, req, timeout=5.0)
                if result is not None and bool(getattr(result, "result", True)):
                    saved = path + ".yaml" if not path.endswith(".yaml") else path
                    self.status = "saved"
                    self.map_file = saved
                    return True, f"map saved by slam_toolbox: {saved}", saved
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"slam_toolbox save failed ({exc}); falling back to PGM/YAML"
                )
        saved = self.occupancy.save(path)
        self.status = "saved"
        self.map_file = saved
        return True, f"map saved: {saved}", saved

    def _load_map(self, name: str):
        name = name or self.default_map_name
        if not name:
            return False, "map_name is required for load", ""
        path = self._resolve_path(name)
        if not os.path.exists(path):
            # 允许 name 不带扩展名
            for suffix in (".yaml", ".yml"):
                if os.path.exists(path + suffix):
                    path = path + suffix
                    break
        if not os.path.exists(path):
            return False, f"map not found: {path}", ""
        loaded = OccupancyMap.load(path)
        loaded.frame_id = self.map_frame
        loaded.map_id = os.path.splitext(os.path.basename(path))[0]
        self.occupancy = loaded
        self.map_id = loaded.map_id
        self.map_file = path
        self.status = "localized"
        self._frontier_dirty = True
        # 没有 map_server 时，由本节点发布 /map
        self.pub_map.publish(loaded.to_msg(self.get_clock().now().to_msg()))
        return True, f"map loaded: {path}", path

    def _resolve_path(self, name: str) -> str:
        name = str(name).strip()
        if os.path.isabs(name):
            return name
        safe = os.path.normpath(name).lstrip(os.sep)
        if safe.startswith(".."):
            raise ValueError(f"invalid map name: {name!r}")
        return os.path.join(self.map_dir, safe)

    def _new_map_id(self) -> str:
        return f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    # ------------------------------------------------------------ slam calls
    def _call_slam_reset(self):
        if self.slam_reset is None:
            return
        try:
            self._call_service(self.slam_reset, Reset.Request(), timeout=3.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"slam_toolbox reset failed: {exc}")

    def _call_service(self, client, request, timeout: float):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return None
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = MapManagerNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
