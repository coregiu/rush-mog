#!/usr/bin/env python3
"""RUSH-MOG 外部模块上报 agent（CENTER 等）。

ESP32 与树莓派合并后，ROS 2 与 Edge Server 同机运行，ROS 健康由
``edge_server_node`` 直接读取，不再需要本 agent 上报。本脚本保留给
CENTER SERVER 等外部模块：

1. 连接 RUSH-MOG Wi-Fi（Linux + NetworkManager/nmcli，可关闭）；
2. 读取本机健康/参数接口；
3. 通过 ``POST /api/modules/report`` 向 Pi Edge Server 上报模块、状态和精简参数。

服务端只向 Edge Server 上报，不暴露额外端口；dashboard 通过 Edge Server 读取。
配置文件示例见 ``center_link.yaml.example``；``ros_link.yaml.example`` 仅作历史参考。
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import yaml
except Exception:  # pragma: no cover - 运行环境通常有 PyYAML
    yaml = None

LOGGER = logging.getLogger("rush-mog.edge-link")

DEFAULT_SSID = "RUSH-MOG"
DEFAULT_EDGE_URL = "http://192.168.4.1"
DEFAULT_INTERVAL = 5.0
MAX_SUMMARY = 240
MAX_PARAMS_JSON = 2048

# 每个模块上报的精简参数（完整参数由 dashboard 打开抽屉时通过 Edge Server 代理获取）
ROS_PARAM_KEYS = {
    "publish_rate",
    "startup_grace_sec",
    "rate_window_sec",
    "strict_unknown",
    "safety_required",
    "safety_timeout_sec",
    "check_nodes",
}
CENTER_PARAM_KEYS = {
    "action.mode",
    "action.max_linear_x",
    "action.max_linear_y",
    "action.max_angular_z",
    "action.deadband",
    "action.normalized",
    "action.arm_start",
    "action.round_digits",
}


def _run(cmd: list[str], timeout: float = 35.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
    )


def current_ssid() -> str:
    """读取当前 Wi-Fi SSID；没有 nmcli/iwgetid 时返回空字符串。"""
    for command in (["iwgetid", "-r"],):
        try:
            result = _run(command, timeout=4.0)
            if result.returncode == 0:
                return result.stdout.strip()
        except (FileNotFoundError, subprocess.SubprocessError):
            pass

    try:
        result = _run(["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"], timeout=8.0)
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                active, _, ssid = line.partition(":")
                if active.strip().lower() == "yes":
                    return ssid.strip()
    except (FileNotFoundError, subprocess.SubprocessError):
        pass
    return ""


def ensure_wifi(ssid: str, password: str, *, enabled: bool) -> bool:
    if not ssid:
        return False
    current = current_ssid()
    if current == ssid:
        return True
    if not enabled:
        LOGGER.warning("当前 Wi-Fi=%r，目标=%r；--no-wifi 已关闭自动连接", current, ssid)
        return False

    LOGGER.info("正在连接 RUSH-MOG Wi-Fi: %s", ssid)
    # 优先使用已保存的 NetworkManager 连接（密码可能已保存）
    try:
        result = _run(["nmcli", "--wait", "20", "connection", "up", "id", ssid], timeout=30.0)
        if result.returncode == 0:
            time.sleep(1.0)
            if current_ssid() == ssid:
                return True
        elif result.stderr:
            LOGGER.debug("nmcli connection up 失败: %s", result.stderr.strip())
    except (FileNotFoundError, subprocess.SubprocessError) as exc:
        LOGGER.debug("nmcli connection up 异常: %s", exc)

    if not password:
        LOGGER.error(
            "未提供 Wi-Fi 密码，且没有可用的已保存连接；"
            "请设置 RUSH_MOG_WIFI_PASSWORD 或预配置 NetworkManager 连接"
        )
        return False

    try:
        result = _run(
            ["nmcli", "--wait", "20", "device", "wifi", "connect", ssid, "password", password],
            timeout=35.0,
        )
        if result.returncode == 0:
            time.sleep(1.0)
            return current_ssid() == ssid
        LOGGER.error("连接 Wi-Fi 失败: %s", (result.stderr or result.stdout).strip())
    except (FileNotFoundError, subprocess.SubprocessError) as exc:
        LOGGER.error("无法调用 nmcli: %s", exc)
    return False


def local_ip_for(host: str) -> str:
    """获取到边缘网关的本地 IP；失败时退回主机名解析。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect((host, 80))
            return sock.getsockname()[0]
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "127.0.0.1"


def fetch_json(url: str, timeout: float = 3.0) -> Dict[str, Any]:
    request = urllib.request.Request(
        url,
        headers={"Accept": "application/json", "User-Agent": "rush-mog-edge-link/1.0"},
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = response.read().decode("utf-8")
    data = json.loads(payload)
    if not isinstance(data, dict):
        raise ValueError(f"{url} 返回的不是 JSON 对象")
    return data


def compact_params(module: str, params_payload: Optional[Dict[str, Any]], health: Dict[str, Any]) -> Dict[str, Any]:
    values: Dict[str, Any] = {}
    keys = ROS_PARAM_KEYS if module == "ros" else CENTER_PARAM_KEYS
    if params_payload:
        for item in params_payload.get("params") or []:
            if not isinstance(item, dict):
                continue
            key = str(item.get("key") or "")
            if key in keys:
                values[key] = item.get("value")

    if module == "ros":
        devices = health.get("devices") or []
        values["device_count"] = len(devices)
        values["emergency_stop"] = bool(health.get("emergency_stop"))
    else:
        model = health.get("model") or {}
        values["model.loaded"] = bool(model.get("loaded"))
        values["model.backend"] = model.get("backend")

    # 控制上报体积，避免 Edge Server 表单过长。
    raw = json.dumps(values, ensure_ascii=False, separators=(",", ":"))
    if len(raw) > MAX_PARAMS_JSON:
        return {}
    return values


def build_report(config: Dict[str, Any]) -> Dict[str, str]:
    module = str(config.get("module") or "center").strip().lower()
    name = str(config.get("name") or module)
    health_url = str(config.get("health_url") or "")
    params_url = str(config.get("params_url") or "")
    if not health_url:
        raise ValueError("config.health_url 不能为空")

    health: Dict[str, Any] = {}
    health_error = ""
    try:
        health = fetch_json(health_url, float(config.get("health_timeout_sec", 3.0)))
    except (urllib.error.URLError, TimeoutError, ValueError, OSError) as exc:
        health_error = str(exc)

    params_payload: Optional[Dict[str, Any]] = None
    if params_url:
        try:
            params_payload = fetch_json(params_url, float(config.get("params_timeout_sec", 3.0)))
        except (urllib.error.URLError, TimeoutError, ValueError, OSError) as exc:
            LOGGER.debug("读取参数失败: %s", exc)

    if health:
        status = str(health.get("status") or "unknown")
        version = str(health.get("version") or "")
        summary = str(health.get("reason") or f"status={status}")
    else:
        status = "offline"
        version = ""
        summary = f"health unreachable: {health_error}" if health_error else "health unreachable"

    summary = summary[:MAX_SUMMARY]
    params = compact_params(module, params_payload, health) if health else {}
    health_port = int(config.get("public_health_port", 8090))
    params_port = int(config.get("public_params_port", health_port))
    health_path = str(config.get("public_health_path", "/ros/health"))
    params_path = str(config.get("public_params_path", "/ros/params"))

    return {
        "module": module,
        "name": name,
        "version": version,
        "status": status,
        "summary": summary,
        "health_port": str(health_port),
        "health_path": health_path,
        "params_port": str(params_port),
        "params_path": params_path,
        "params_json": json.dumps(params, ensure_ascii=False, separators=(",", ":")),
    }


def post_report(edge_url: str, report: Dict[str, str], timeout: float = 3.0) -> Dict[str, Any]:
    url = edge_url.rstrip("/") + "/api/modules/report"
    body = urllib.parse.urlencode(report).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        method="POST",
        headers={
            "Content-Type": "application/x-www-form-urlencoded;charset=UTF-8",
            "Accept": "application/json",
            "User-Agent": "rush-mog-edge-link/1.0",
        },
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = response.read().decode("utf-8")
    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        return {"ok": False, "raw": payload}


def load_config(path: str) -> Dict[str, Any]:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as handle:
        if yaml is None:
            raise RuntimeError("需要 PyYAML 读取配置；或使用 --config 指向 JSON 文件")
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("配置文件根节点必须是对象")
    return data


def configure_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="[%(asctime)s] [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def run(config: Dict[str, Any], *, once: bool = False, dry_run: bool = False) -> int:
    module = str(config.get("module") or "center")
    if module == "ros":
        LOGGER.info(
            "ROS 2 已并入 Pi Edge Server，健康数据由 edge_server_node 本地读取；"
            "rush-mog-ros-link 服务不再需要"
        )
        return 0
    ssid = str(config.get("ssid") or DEFAULT_SSID)
    password = str(config.get("password") or os.environ.get("RUSH_MOG_WIFI_PASSWORD") or "")
    edge_url = str(config.get("edge_url") or DEFAULT_EDGE_URL)
    interval = max(1.0, float(config.get("interval_sec", DEFAULT_INTERVAL)))
    connect_wifi = bool(config.get("connect_wifi", True))
    edge_host = urllib.parse.urlsplit(edge_url).hostname or "192.168.4.1"

    backoff = 1.0
    while True:
        if connect_wifi:
            try:
                ensure_wifi(ssid, password, enabled=True)
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("Wi-Fi 连接异常: %s", exc)

        try:
            report = build_report(config)
            ip = local_ip_for(edge_host)
            report["_ip"] = ip  # 仅日志/调试；Edge Server 以 remoteIP 为准
            if dry_run:
                print(json.dumps(report, ensure_ascii=False, indent=2))
            else:
                response = post_report(edge_url, {k: v for k, v in report.items() if not k.startswith("_")})
                LOGGER.info(
                    "上报 %s: status=%s ip=%s -> %s",
                    module, report.get("status"), ip,
                    response.get("ok") if isinstance(response, dict) else response,
                )
            backoff = 1.0
        except (urllib.error.URLError, TimeoutError, ValueError, OSError, RuntimeError) as exc:
            LOGGER.warning("上报失败（%.0fs 后重试）: %s", backoff, exc)
            if once:
                return 1
            time.sleep(backoff)
            backoff = min(30.0, backoff * 1.7)
            continue

        if once:
            return 0
        time.sleep(interval)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="RUSH-MOG CENTER -> Pi Edge Server reporter")
    parser.add_argument("--config", default="edge_link.yaml", help="YAML 配置文件")
    parser.add_argument("--once", action="store_true", help="只上报一次后退出")
    parser.add_argument("--dry-run", action="store_true", help="不发送，只打印上报内容")
    parser.add_argument("--no-wifi", action="store_true", help="不主动连接 Wi-Fi")
    parser.add_argument("--edge-url", default="", help="覆盖 Edge Server 地址（默认 http://192.168.4.1）")
    parser.add_argument("--verbose", action="store_true", help="DEBUG 日志")
    args = parser.parse_args(argv)

    configure_logging(args.verbose)
    try:
        config = load_config(args.config)
    except Exception as exc:  # noqa: BLE001
        LOGGER.error("读取配置失败: %s", exc)
        return 2

    if args.edge_url:
        config["edge_url"] = args.edge_url
    if args.no_wifi:
        config["connect_wifi"] = False

    try:
        return run(config, once=args.once, dry_run=args.dry_run)
    except KeyboardInterrupt:
        LOGGER.info("停止")
        return 0


if __name__ == "__main__":
    sys.exit(main())
