# center_link：CENTER 自动接入 Pi Edge Server 并上报

ESP32 与树莓派合并后，**ROS 2 与 Edge Server 同机运行**，ROS 健康由
`edge_server_node` 直接读取，不再需要 `ros_link` 上报。本目录保留
`edge_link.py` 给 **CENTER SERVER（PC/GPU 主机）** 等外部模块使用：

1. 系统启动后自动连接 **RUSH-MOG** Wi-Fi（Linux + NetworkManager `nmcli`，可关闭）；
2. 读取本机健康/参数接口；
3. 每 5 秒向 `http://192.168.4.1/api/modules/report` 上报模块、状态和精简参数。

```text
┌──────────────────────────────────────────────┐
│ Pi Edge Server（树莓派，同一台机器）           │
│  :80  web/API      :8080 MJPEG                │
│  edge_server_node  :8090 /ros/health          │
│  ROS 2 导航/感知/健康                          │
└───────────────▲──────────────────────────────┘
                │ Wi-Fi: RUSH-MOG / LAN
                │ POST /api/modules/report
┌───────────────┴──────────────────────────────┐
│ CENTER SERVER :8000                           │
│ /v1/health、/v1/params                        │
└──────────────────────────────────────────────┘
                ▲
                │ HTTP 80（页面只访问 Edge Server）
┌───────────────┴──────────────────────────────┐
│ 手机 / PC 浏览器：http://192.168.4.1/          │
│ dashboard.html / 手柄页                       │
└──────────────────────────────────────────────┘
```

- 页面只访问 `192.168.4.1`，由 Edge Server 同机代理模块接口，避免手机跨域/AP 客户端隔离问题；
- 模块上报使用 `application/x-www-form-urlencoded`，字段小，适配 80 端口 HTTP 服务；
- Edge Server 用连接来源 `remoteIP` 作为模块 IP，避免上报方伪造地址；
- 完整参数在页面打开模块抽屉时通过 Edge Server 代理实时获取，上报里只带精简参数；
- `ros` 是 Edge Server 保留的本地子系统 ID，不允许外部上报。

---

## 1. 前置条件

- Pi 已部署 `software/ros` 的 `edge_server_node`（80/8080）和 ROS 健康网关（`:8090`）；
- AP `RUSH-MOG` 由树莓派 hostapd 提供，或 CENTER 与 Pi 在同一局域网；
- CENTER 主机有 Wi-Fi，并安装 NetworkManager（`nmcli`）；没有 `nmcli` 时脚本只会上报，不会主动切网；
- Python 3.8+，并安装 PyYAML；
- CENTER 健康/参数接口可本机访问：
  - `http://127.0.0.1:8000/v1/health`
  - `http://127.0.0.1:8000/v1/params`

## 2. 部署（只需要 CENTER）

```bash
# 1) 代码放到约定目录（也可以改 systemd 里的路径）
sudo mkdir -p /opt/rush-mog
sudo cp -r software /opt/rush-mog/

# 2) CENTER 配置
sudo mkdir -p /etc/rush-mog
sudo cp /opt/rush-mog/software/tools/center_link.yaml.example /etc/rush-mog/center-link.yaml

# 3) Wi-Fi 密码放环境文件，不要写进仓库配置
sudo cp /opt/rush-mog/software/tools/systemd/edge-link.env.example /etc/rush-mog/edge-link.env
sudo chmod 600 /etc/rush-mog/edge-link.env
sudo editor /etc/rush-mog/edge-link.env   # 修改 RUSH_MOG_WIFI_PASSWORD

# 4) systemd（只需要 center-link）
sudo cp /opt/rush-mog/software/tools/systemd/rush-mog-center-link.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rush-mog-center-link.service
```

> `rush-mog-ros-link.service` 与 `ros_link.yaml.example` 是旧 ESP32 架构的兼容文件，
> 新部署不要再启用。

## 3. 验证

```bash
# 只打印，不发送
python3 /opt/rush-mog/software/tools/edge_link.py \
  --config /etc/rush-mog/center-link.yaml --dry-run --once

# 发送一次
python3 /opt/rush-mog/software/tools/edge_link.py \
  --config /etc/rush-mog/center-link.yaml --once

# 查看 Pi Edge Server 注册表
curl -s http://192.168.4.1/api/modules | python3 -m json.tool

# 看服务日志
journalctl -u rush-mog-center-link.service -f
```

页面入口：手机连 `RUSH-MOG` 后访问 `http://192.168.4.1/dashboard.html`，或在手柄页点击 **RUSH-MOG TWIN** 按钮。

## 4. 配置字段

| 字段 | 默认 | 说明 |
|---|---|---|
| `module` | `center` | 模块 ID（`edge`/`ros` 为 Edge 保留） |
| `name` | module | 显示名 |
| `ssid` | `RUSH-MOG` | 目标 Wi-Fi |
| `password` | 空 | 为空时读环境变量 `RUSH_MOG_WIFI_PASSWORD` |
| `connect_wifi` | `true` | 是否主动连接 Wi-Fi |
| `edge_url` | `http://192.168.4.1` | Pi Edge Server 地址 |
| `health_url` | `http://127.0.0.1:8000/v1/health` | CENTER 本机健康接口 |
| `params_url` | `http://127.0.0.1:8000/v1/params` | CENTER 本机参数接口 |
| `public_health_port` | 8000 | 上报给 Edge Server 的端口 |
| `public_health_path` | `/v1/health` | 路径 |
| `public_params_port` | 同健康端口 | 参数端口 |
| `public_params_path` | `/v1/params` | 路径 |
| `interval_sec` | 5 | 上报间隔 |
| `health_timeout_sec` | 3 | 健康接口超时 |
| `params_timeout_sec` | 3 | 参数接口超时 |

## 5. 故障排查

| 现象 | 检查 |
|---|---|
| `dashboard` 显示 CENTER 未上报 | `curl http://192.168.4.1/api/modules`；看 systemd 日志；确认 CENTER 与 Pi 在同一网络 |
| 无法连接 Wi-Fi | `nmcli device status`；`nmcli device wifi list`；确认密码；`journalctl` 看 nmcli 输出 |
| `nmcli` 权限不足 | systemd 已用 root；手动测试用 `sudo nmcli ...` |
| 页面参数抽屉打不开 | 检查 Edge Server 是否能访问 CENTER IP（代理 502）；或点“详情”看缓存 |
| 参数修改返回 401 | CENTER 配置了 `api_key`；当前代理未带 token，需要在模块侧关闭或给代理加鉴权 |
| 修改成功但重启后丢失 | CENTER 的 `action.*` 参数是运行时生效；重启后回到配置 |

## 6. 安全说明

- `/etc/rush-mog/edge-link.env` 含 Wi-Fi 密码，必须 `chmod 600`，不要提交到仓库；
- `RUSH-MOG` AP 由树莓派 hostapd 配置，密码不在本仓库代码中；
- 模块参数修改是运行时能力，建议只在可信 Wi-Fi 下开放；CENTER 的 `api_key` 若启用，
  需要额外给 Edge Server 代理加 token（当前默认空）。
