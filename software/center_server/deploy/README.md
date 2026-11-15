# Center Server 部署与发布

> 适用版本：0.2.0+（SmolVLA HTTP 推理 + Edge 启动握手）
> 相关文档：[../README.md](../README.md)、[../DESIGN.md](../DESIGN.md)、
> 机载侧 [../../edge_server/README.md](../../edge_server/README.md)

本目录提供三种部署方式，代码与 API 完全一致，只改监听地址/设备：

| 方式 | 适用 | 握手目标 |
|---|---|---|
| A. venv + systemd | GPU 工作站/笔记本长期运行（推荐） | `CENTER_SERVER_EDGE_URL` |
| B. Docker Compose（CPU） | 快速联调、stub 链路、无 GPU | compose `.env` 的 `EDGE_URL` |
| C. Docker Compose + GPU | 已有 NVIDIA Container Toolkit 的服务器 | 同上 |

---

## 1. 启动时序（含握手）

```text
Center Server                          Edge Server（Pi）                 VLA 节点（ROS）
     │                                      │                                │
     │ 1. 加载配置/可选预热模型              │                                │
     │ 2. 绑定 0.0.0.0:8000                 │                                │
     │ 3. POST /api/center/handshake ──────►│                                │
     │    {ip,port,/v1/infer,model,...}     │ 4. 以 TCP 对端 IP 记录 CENTER   │
     │◄──── {"ok":true,"infer_url":...} ────│    地址（内存注册表）           │
     │ 5. 每 10s heartbeat（断线自动重连）  │                                │
     │                                      │◄── GET /api/center ────────────│
     │                                      │─── {base_url,infer_url,...} ──►│
     │◄──────────── POST /v1/infer {图像+state+指令} ────────────────────────│
     │───────────── actions/base_cmd ───────────────────────────────────────►│
```

要点：

- 握手在 HTTP 绑定成功后发送，Edge 记录的地址一定可连接；
- Edge 以握手来源 IP 为准（不信任 payload 里的 `ip`），端口取 `infer_port`；
- 失败按 `retry_sec` 指数退避（上限 `max_retry_sec`），不影响 `/v1/infer`；
- Edge 重启后，下一次 heartbeat 自动重新注册，无需重启 Center。

---

## 2. 模型准备

```bash
cd software/center_server

# 联网机器下载到本地目录（推荐离线部署）
./scripts/download_model.sh lerobot/smolvla_base ./models/smolvla_base

# 或使用本车微调后的 checkpoint（发布到 HF 私有仓库或直接拷贝）
./scripts/download_model.sh my-org/rush-mog-smolvla ./models/rush-mog-smolvla
```

然后修改 `config.yaml`（或环境变量）：

```yaml
model:
  path: "/opt/models/smolvla_base"   # 绝对路径或 HF repo id
  device: "cuda"                     # 无 GPU 用 cpu（延迟高）
  dtype: "float16"
  local_files_only: true
  preload: true                      # 需要 readiness/低首请求延迟时
```

> `lerobot/smolvla_base` 是通用 checkpoint，动作空间取决于训练机器人。
> 控制本车必须用本车数据微调，并把 `action.mode`/`max_*` 调到匹配；
> 先用 `action.mode: raw` 验证链路，再切 `base_twist`。

---

## 3. 方式 A：venv + systemd（推荐）

```bash
# 1) 安装到固定目录
sudo mkdir -p /opt/rush-mog-center /var/lib/rush-mog-center
sudo rsync -a --exclude '.venv' --exclude '__pycache__' \
    software/center_server/ /opt/rush-mog-center/
cd /opt/rush-mog-center

# 2) 创建虚拟环境（GPU 主机先按 pytorch.org 选择 CUDA 版本）
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -r requirements-vla.txt

# 3) 配置
sudo cp deploy/center.env.example /etc/rush-mog/center.env   # 改 EDGE_URL/模型路径
sudo chmod 600 /etc/rush-mog/center.env
sudo cp config.yaml config.yaml.bak                          # 按现场改端口/模型/握手
python -m center_server --config config.yaml --check          # 只校验配置

# 4) 安装服务
sudo useradd -r -m -d /var/lib/rush-mog-center center || true
sudo chown -R center:center /var/lib/rush-mog-center
sudo cp deploy/rush-mog-center.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rush-mog-center
journalctl -u rush-mog-center -f
```

验证：

```bash
curl -s http://127.0.0.1:8000/v1/health | python3 -m json.tool | head -30
curl -s http://127.0.0.1:8000/v1/handshake | python3 -m json.tool
# Edge 侧应看到 CENTER（把 192.168.3.26 换成实际地址）
curl -s http://192.168.3.26/api/center | python3 -m json.tool
```

---

## 4. 方式 B/C：Docker Compose

```bash
cd software/center_server
cp deploy/.env.example .env            # EDGE_URL / CENTER_MODEL_PATH / CENTER_DEVICE

# 先准备模型（容器不联网也能跑离线权重）
./scripts/download_model.sh lerobot/smolvla_base ./models/smolvla_base

# CPU（INSTALL_VLA=0 时镜像小，只能跑 stub/无 torch）
INSTALL_VLA=0 docker compose -f deploy/docker-compose.yml up -d --build

# GPU 主机（构建 torch/lerobot，需要 NVIDIA Container Toolkit）
docker compose -f deploy/docker-compose.yml -f deploy/docker-compose.gpu.yml \
    up -d --build
docker compose -f deploy/docker-compose.yml logs -f center
```

默认挂载：

| 宿主机 | 容器 | 说明 |
|---|---|---|
| `../config.yaml` | `/app/config.yaml` | 可用 `CENTER_CONFIG` 覆盖 |
| `../models` | `/models:ro` | 离线权重 |
| named volume | `/home/app/.cache/huggingface` | HF 缓存（在线模式） |

> 容器内 `127.0.0.1` 指向容器自身，握手地址必须用 `EDGE_URL=http://<Pi IP>`。

---

## 5. 发布清单（上线前逐项确认）

1. **模型**：`config.yaml` 的 `model.path` 指向真实权重，`--warmup` 或
   `preload: true` 能成功；`/v1/ready` 返回 200。
2. **动作空间**：已在本车数据上微调，`action.mode=base_twist` 时
   `max_linear_x/y`、`max_angular_z` 与底盘能力匹配（不是 base checkpoint 直用）。
3. **握手**：`handshake.enabled=true`、`edge_url` 可达，
   `/v1/handshake` 的 `connected=true`；Edge `GET /api/center` 能看到本机。
4. **鉴权**：非可信网络启用 `server.api_key`，Edge 侧 `center.remote.api_key`
   填相同值；不要直接把 8000 暴露公网，TLS 交给反向代理。
5. **防火墙**：仅放行 Edge 网段到 8000/TCP；Edge 的 80 端口只对机载网段开放。
6. **资源**：显存 ≥6GB（float16 约 1GB 权重 + 激活），CPU 模式只演示。
7. **降级**：Edge `vla_engine.backend=auto`，Center 不可用时自动回退规则桩并
   停车；`remote` 模式失败会发 stop（confidence 0）。
8. **日志**：`journalctl -u rush-mog-center` / `docker logs`，不记录图像和密钥。

---

## 6. 握手协议（Center → Edge）

`POST http://<edge>/api/center/handshake`，`Content-Type: application/json`：

```json
{
  "module": "center",
  "name": "SmolVLA Center Server",
  "version": "0.2.0",
  "status": "ok",
  "summary": "ready=True backend=smolvla device=cuda model=lerobot/smolvla_base",
  "ip": "192.168.4.50",
  "port": 8000,
  "scheme": "http",
  "health_port": 8000, "health_path": "/v1/health",
  "params_port": 8000, "params_path": "/v1/params",
  "infer_port": 8000,  "infer_path": "/v1/infer",
  "params_json": "{\"action.mode\":\"base_twist\",\"action.max_linear_x\":0.5}",
  "meta_json": "{\"protocol\":\"http-json-v1\",\"capabilities\":[\"infer\"],\"model\":{\"backend\":\"smolvla\"}}",
  "handshake": "startup",
  "timestamp": "2026-11-15T05:20:00+00:00"
}
```

Edge 响应（字段供 Center 记录，不参与推理）：

```json
{
  "ok": true,
  "module": "center",
  "ip": "192.168.4.50",
  "port": 8000,
  "scheme": "http",
  "health_url": "http://192.168.4.50:8000/v1/health",
  "params_url": "http://192.168.4.50:8000/v1/params",
  "infer_url": "http://192.168.4.50:8000/v1/infer",
  "handshake": "startup",
  "message": "CENTER 握手成功，地址已记录"
}
```

- `handshake`：`startup`（首次）/`heartbeat`（周期）/`manual`（`POST /v1/handshake`）；
- 兼容旧路径：`handshake.path` 可改为 `/api/modules/report`（表单/JSON 均可）；
- Edge 只接受本机/内网地址（`_valid_module_url`），公网 IP 会被 400 拒绝。

---

## 7. 故障排查

| 现象 | 原因/处理 |
|---|---|
| `/v1/handshake` 显示 `connected=false` | 检查 `edge_url`、Pi 是否启动、80 端口/防火墙；看日志里的 `last_error` |
| Edge `GET /api/center` 404 | Center 还没握手成功；确认 Center 已启动且 `handshake.enabled=true` |
| Edge 记录后 VLA 仍走规则桩 | `vla_engine.backend` 不是 auto/remote；Edge 的 `discovery_url` 端口与实际 web 端口不一致 |
| VLA 报 `base_cmd` 缺失 | Center `action.mode` 是 `raw`/`joint`；改 `base_twist` 或 `hybrid` |
| 启动 503 `model_unavailable` | torch/lerobot 未装、权重路径错、显存不足；`/v1/health` 有 `error` |
| Docker 容器握手到 127.0.0.1 | 容器网络隔离；`EDGE_URL` 用 Pi 的局域网 IP |
| GPU 容器看不到显卡 | 未装 NVIDIA Container Toolkit/未加 gpu overlay；先 `nvidia-smi` |

---

## 8. 升级与回滚

```bash
# venv/systemd
sudo rsync -a software/center_server/ /opt/rush-mog-center/ --exclude .venv
sudo systemctl restart rush-mog-center

# Docker
docker compose -f deploy/docker-compose.yml build --pull
docker compose -f deploy/docker-compose.yml up -d
```

Center 与 Edge 只通过 HTTP 解耦，可独立升级；Edge 侧注册表是**内存**数据，
Center 重启后会由 `startup` 握手自动刷新。回滚只需恢复旧镜像/旧代码目录，
动作协议（`/v1/infer` 字段）保持向后兼容。
