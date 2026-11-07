# edge_link：ROS / CENTER 自动接入 RUSH-MOG 并上报

`edge_link.py` 是运行在 ROS（树莓派/Orin）和 CENTER SERVER（PC/GPU 主机）上的轻量 agent：

1. 系统启动后自动连接 **RUSH-MOG** Wi-Fi（Linux + NetworkManager `nmcli`，可关闭）；
2. 读取本机健康/参数接口；
3. 每 5 秒向 `http://192.168.4.1/api/modules/report` 上报模块、状态和精简参数。

```text
┌────────────────────┐        Wi-Fi: RUSH-MOG         ┌────────────────────┐
│ ROS :8090          │ ─────────────────────────────> │ ESP32-S3           │
│ /ros/health        │   POST /api/modules/report     │ 192.168.4.1        │
│ /ros/params        │                                │ /api/modules       │
└────────────────────┘                                │ /api/modules/{id}/ │
┌────────────────────┐        Wi-Fi: RUSH-MOG         │   health/params    │
│ CENTER :8000       │ ─────────────────────────────> │                    │
│ /v1/health         │   POST /api/modules/report     │ dashboard.html     │
│ /v1/params         │                                └─────────┬──────────┘
└────────────────────┘                                          │ HTTP 80
                                                       ┌────────▼──────────┐
                                                       │ 手机 / PC 浏览器   │
                                                       │ 192.168.4.1/      │
                                                       │ dashboard.html    │
                                                       └───────────────────┘
```

- 页面只访问 `192.168.4.1`，由 ESP32 代理模块接口，避免手机跨域/AP 客户端隔离问题；
- 模块上报使用 `application/x-www-form-urlencoded`，字段小，适配 ESP32 WebServer；
- ESP32 用连接来源 `remoteIP` 作为模块 IP，避免上报方伪造地址；
- 完整参数在页面打开模块抽屉时通过 ESP32 代理实时获取，上报里只带精简参数。

---

## 1. 前置条件

- ESP32-S3 固件已升级到包含 `/api/modules*` 的版本（本仓库 `software/edge_server`）；
- AP `RUSH-MOG` 的最大客户端数为 4（`AP_MAX_CLIENTS`），足够手机 + ROS + CENTER + 调试机；
- ROS / CENTER 主机有 Wi-Fi，并安装 NetworkManager（`nmcli`）；没有 `nmcli` 时脚本只会上报，不会主动切网；
- Python 3.8+，并安装 PyYAML（ROS 的 `python3-yaml` / Center 的 requirements 已包含）；
- 模块健康/参数接口可本机访问：
  - ROS：`http://127.0.0.1:8090/ros/health`、`/ros/params`；
  - CENTER：`http://127.0.0.1:8000/v1/health`、`/v1/params`。

## 2. 部署

```bash
# 1) 代码放到约定目录（也可以改 systemd 里的路径）
sudo mkdir -p /opt/rush-mog
sudo cp -r software /opt/rush-mog/

# 2) 配置
sudo mkdir -p /etc/rush-mog
sudo cp /opt/rush-mog/software/tools/ros_link.yaml.example /etc/rush-mog/ros-link.yaml
sudo cp /opt/rush-mog/software/tools/center_link.yaml.example /etc/rush-mog/center-link.yaml

# 3) Wi-Fi 密码放环境文件，不要写进仓库配置
sudo cp /opt/rush-mog/software/tools/systemd/edge-link.env.example /etc/rush-mog/edge-link.env
sudo chmod 600 /etc/rush-mog/edge-link.env
sudo editor /etc/rush-mog/edge-link.env   # 修改 RUSH_MOG_WIFI_PASSWORD

# 4) systemd
sudo cp /opt/rush-mog/software/tools/systemd/rush-mog-ros-link.service /etc/systemd/system/
sudo cp /opt/rush-mog/software/tools/systemd/rush-mog-center-link.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rush-mog-ros-link.service
sudo systemctl enable --now rush-mog-center-link.service
```

ROS 与 CENTER 在同一台机器时，两个 service 可以同时运行；它们共用同一 Wi-Fi 接口，只是模块 ID 不同。

## 3. 验证

```bash
# 只打印，不发送
python3 /opt/rush-mog/software/tools/edge_link.py \
  --config /etc/rush-mog/ros-link.yaml --dry-run --once

# 发送一次
python3 /opt/rush-mog/software/tools/edge_link.py \
  --config /etc/rush-mog/ros-link.yaml --once

# 查看 ESP32 注册表
curl -s http://192.168.4.1/api/modules | python3 -m json.tool

# 看服务日志
journalctl -u rush-mog-ros-link.service -f
journalctl -u rush-mog-center-link.service -f
```

页面入口：手机连 `RUSH-MOG` 后访问 `http://192.168.4.1/dashboard.html`，或在手柄页点击 **RUSH-MOG TWIN** 按钮。

## 4. 配置字段

| 字段 | 默认 | 说明 |
|---|---|---|
| `module` | `ros` | 模块 ID：`ros` / `center`（`edge` 保留） |
| `name` | module | 显示名 |
| `ssid` | `RUSH-MOG` | 目标 Wi-Fi |
| `password` | 空 | 为空时读环境变量 `RUSH_MOG_WIFI_PASSWORD` |
| `connect_wifi` | `true` | 是否主动连接 Wi-Fi |
| `edge_url` | `http://192.168.4.1` | ESP32 地址 |
| `health_url` | 模块默认 | 本机健康接口 |
| `params_url` | 模块默认 | 本机参数接口 |
| `public_health_port` | 8090/8000 | 上报给 ESP32 的端口 |
| `public_health_path` | `/ros/health` 或 `/v1/health` | 路径 |
| `public_params_port` | 同健康端口 | 参数端口 |
| `public_params_path` | `/ros/params` 或 `/v1/params` | 路径 |
| `interval_sec` | 5 | 上报间隔 |
| `health_timeout_sec` | 3 | 健康接口超时 |
| `params_timeout_sec` | 3 | 参数接口超时 |

## 5. 故障排查

| 现象 | 检查 |
|---|---|
| `dashboard` 显示模块未上报 | `curl http://192.168.4.1/api/modules`；看 systemd 日志；确认模块与 ESP32 在同一 `RUSH-MOG` AP |
| 无法连接 Wi-Fi | `nmcli device status`；`nmcli device wifi list`；确认密码；`journalctl` 看 nmcli 输出 |
| `nmcli` 权限不足 | systemd 已用 root；手动测试用 `sudo nmcli ...` |
| 模块 IP 不是 192.168.4.x | DHCP 异常；重新连接 AP；ESP32 最多 4 个客户端 |
| 页面参数抽屉打不开 | 检查 ESP32 是否能访问模块 IP（代理 502）；或点“详情”看缓存 |
| 参数修改返回 401 | 模块配置了 `api_key`；当前 agent/代理未带 token，需要在模块侧关闭或给代理加鉴权 |
| 修改成功但重启后丢失 | ROS 参数当前是运行时生效；重启后回到 YAML 配置。CENTER 的 `action.*` 同理 |

## 6. 安全说明

- `/etc/rush-mog/edge-link.env` 含 Wi-Fi 密码，必须 `chmod 600`，不要提交到仓库；
- ESP32 AP 密码在 `software/edge_server/include/web.h`，量产时应修改；
- 模块参数修改是运行时能力，建议只在可信 Wi-Fi 下开放；ROS/CENTER 的 `api_key` 若启用，需要额外给 ESP32 代理加 token（当前默认空）。
