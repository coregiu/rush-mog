# Center Server — SmolVLA HTTP 推理服务

> 版本：0.2.0
> 运行环境：Python 3.10+；真实模型建议 NVIDIA GPU（CPU 可跑但延迟高）
> 接口：HTTP/JSON，`POST /v1/infer`、`GET/POST /v1/params`、`GET/POST /v1/handshake`
> **现场部署**：Center `192.168.3.25:8000`（Windows）⇄ Edge `192.168.3.26:80`（树莓派），
> 完整部署/增量构建/发布/运维见 §9。

本目录是 Rush-MOG 架构中的**中心推理模块**。机载 ROS（树莓派 4B/5 或 Jetson
Orin Nano）负责相机/激光/里程计采集与底盘安全控制；中心服务器加载 SmolVLA，
把语言指令 + 多路图像 + 本体状态推理成动作块（action chunk），再通过 HTTP
返回给 ROS。

启动后本服务会主动向 **Edge Server 发送握手**（默认每 10s 心跳），上报本机
IP、推理端口和模型能力；Edge 记录后，机载 `vla_inference_node` 通过
`GET /api/center` 自动发现并调用本服务，不需要手工维护 IP。部署细节见
[`deploy/README.md`](deploy/README.md) 与 §9。

> 本服务**不直接控制 STM32**。动作必须先经过 ROS 的 `velocity_mux_node`、
> `safety_manager_node` 和 `base_bridge_node`，否则绕过急停/避障会有安全风险。

---

## 1. 在系统中的位置

```text
┌──────────────────────────────┐        HTTP/JSON        ┌──────────────────────────────┐
│ 车载 ROS2（Pi 4B/5 / Orin）   │  POST /v1/infer         │ Center Server（GPU/PC）       │
│ camera / scan / odom / arm   │ ──────────────────────► │ SmolVLA policy + processors   │
│ vla_inference_node           │ ◄────────────────────── │ action chunk + base_cmd       │
│ safety / velocity_mux / base │   actions / base_cmd    │ /v1/health /v1/model         │
│       ▲  GET /api/center     │                         │                              │
│  Edge Server（Pi，80 端口）   │ ◄── 启动握手/每 10s ──── │ POST /api/center/handshake   │
└──────────────┬───────────────┘   {ip,port,infer_url}   └──────────────────────────────┘
               │ UART
┌──────────────▼───────────────┐
│ STM32 + L298N + 机械臂        │
└──────────────────────────────┘
```

- 机载 ROS 包（SLAM/导航/搜索/安全）在 [`software/edge_server`](../edge_server/README.md)。
- 本文只描述中心 VLA 推理服务；两者通过 HTTP 解耦，可分别部署/升级。

---

## 2. 目录

```text
software/center_server/
├── center_server/
│   ├── action_mapper.py     # SmolVLA 动作 -> [vx, vy, wz] / 机械臂关节
│   ├── config.py            # YAML 配置加载/校验
│   ├── engine.py            # SmolVLA 加载 + 推理 + stub 后端
│   ├── errors.py            # 统一错误码
│   ├── handshake.py         # 启动/心跳握手 -> Edge Server 注册表
│   ├── http_api.py          # 路由（纯函数，便于单测）
│   ├── images.py            # base64/data URI -> float32 CHW
│   ├── server.py            # ThreadingHTTPServer + CLI
│   ├── service.py           # 校验/推理/响应组装
│   └── clients/http_client.py  # 给 ROS/调试脚本用的轻量客户端
├── deploy/                  # Dockerfile、compose、systemd、部署说明
│   └── windows/             # 192.168.3.25：安装/启动/停止/装 VLA/下模型
├── scripts/                 # download_model.sh / run.sh（Linux）
├── tests/                   # 单元 + HTTP 端到端（不需要 GPU）
├── config.yaml              # 生产配置（smolvla 后端 + Edge 握手）
├── config.stub.yaml         # 联调配置（stub 后端，默认不握手）
├── requirements.txt         # 基础服务依赖
├── requirements-vla.txt     # torch/lerobot 依赖
├── requirements-dev.txt     # pytest 等开发依赖
├── DESIGN.md                # 详细设计
└── README.md
```

---

## 3. 快速开始

### 3.1 基础服务（不加载模型）

```bash
cd software/center_server
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 用固定动作的 stub 验证 HTTP 链路
python -m center_server --config config.stub.yaml
```

另开终端：

```bash
curl -s http://127.0.0.1:8000/v1/health | python3 -m json.tool
curl -s http://127.0.0.1:8000/v1/model  | python3 -m json.tool
```

### 3.2 安装真实 SmolVLA 依赖

```bash
# 先按 https://pytorch.org 选择与本机 CUDA 匹配的 torch
pip install -r requirements-vla.txt
```

`requirements-vla.txt` 包含 `torch`、`torchvision`、`lerobot[smolvla]` 和
Hugging Face 下载依赖。LeRobot/SmolVLA 迭代较快，建议使用 0.4.0 以上版本；
若接口有变化，`engine.py` 会给出明确的 503 错误和安装提示，而不是静默降级。

### 3.3 准备模型

默认配置使用 Hugging Face 上的通用 checkpoint：

```yaml
model:
  path: "lerobot/smolvla_base"
```

首次请求/启动预热时会自动下载到 `~/.cache/huggingface`。离线部署时：

```bash
# 在可联网机器下载
huggingface-cli download lerobot/smolvla_base \
  --local-dir /data/models/smolvla_base

# 中心服务器配置
model:
  path: "/data/models/smolvla_base"
  local_files_only: true
```

> **重要**：`lerobot/smolvla_base` 是跨机器人数据训练的通用 checkpoint，
> 其动作空间由训练数据决定（常见为机械臂关节/末端），不是 Rush-MOG 的
> 底盘速度。直接用 `base_twist` 映射没有物理意义。要控制本车，请先用
> LeRobot 在本车演示数据上微调，并把动作空间设计为 `[vx, vy, wz]`
> （可选再接机械臂关节），再修改 `action` 配置。详见 [DESIGN.md](DESIGN.md)
> 和 LeRobot 文档。

### 3.4 启动真实服务

```bash
python -m center_server --config config.yaml
# 启动即加载模型（首请求更快，但会占显存）：
python -m center_server --config config.yaml --warmup
# 只看配置/状态，不启动 HTTP：
python -m center_server --config config.yaml --check
```

启动日志会打印监听地址和 `/v1/health`。模型是**懒加载**的：默认第一次
`/v1/infer` 才加载；`model.preload: true` 或 `--warmup` 可提前加载。

---

## 4. HTTP API

### 4.1 端点

| 方法 | 路径 | 说明 | 鉴权 |
|---|---|---|---|
| GET | `/` | 服务信息与端点列表 | 否 |
| GET | `/healthz`、`/livez` | 存活探针，恒 200 | 否 |
| GET | `/readyz`、`/v1/ready` | 就绪探针；模型未加载返回 503 | 否 |
| GET | `/v1/health` | 详细健康状态（模型/设备/错误） | 否 |
| GET | `/v1/model` | 模型 schema（相机键、state/action 维度、动作模式） | 是* |
| GET | `/v1/params` | 动作映射/限幅参数描述 | 是* |
| POST | `/v1/params` | 运行时修改动作映射/限幅参数 | 是* |
| POST | `/v1/infer` | 多模态推理 | 是* |
| POST | `/v1/reset` | 清理策略内部动作队列/回合状态 | 是* |
| POST | `/v1/warmup` | 空推理预热（会加载模型） | 是* |
| GET | `/v1/handshake` | 握手状态（目标地址/连接/最近错误） | 是* |
| POST | `/v1/handshake` | 立即向 Edge Server 重发一次握手 | 是* |
| OPTIONS | `*` | CORS 预检 | 否 |

\* 仅当 `server.api_key` 非空时要求鉴权。支持
`Authorization: Bearer <key>` 或 `X-API-Key: <key>`。

`GET /v1/health` 额外返回 `handshake` 段：

```json
{
  "handshake": {
    "enabled": true,
    "target": "http://${edge_server}/api/center/handshake",
    "connected": true,
    "attempts": 3,
    "successes": 3,
    "failures": 0,
    "last_error": "",
    "last_ok_at": "2026-11-15T05:20:10+00:00",
    "last_response": {"ok": true, "ip": "192.168.3.25", "port": 8000,
                      "infer_url": "http://192.168.3.25:8000/v1/infer"}
  }
}
```

### 4.1.0 启动握手：`POST /api/center/handshake`（Center → Edge）

Center Server 在 HTTP 端口绑定成功后，由后台线程向 Edge 发送：

```json
{
  "module": "center",
  "name": "SmolVLA Center Server",
  "version": "0.2.0",
  "status": "ok",
  "summary": "ready=True backend=smolvla device=cuda model=lerobot/smolvla_base",
  "ip": "192.168.3.25",
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

- Edge 以 **TCP 对端 IP** 为准记录地址，payload 里的 `ip` 仅用于展示；
- 成功后每 `handshake.interval_sec` 发送 `heartbeat`，失败按
  `retry_sec → max_retry_sec` 指数退避（不影响 `/v1/infer`）；
- Edge 重启后下一次 heartbeat 自动重新注册；
- 兼容旧版：`handshake.path` 设为 `/api/modules/report` 即可复用旧上报接口。

手工触发一次（调试）：

```bash
curl -s -X POST http://127.0.0.1:8000/v1/handshake | python3 -m json.tool
# Edge 侧确认
curl -s http://${edge_server}/api/center | python3 -m json.tool
```

### 4.1.1 参数：`GET/POST /v1/params`

`GET` 返回可编辑的动作映射/速度限幅参数：

```bash
curl -s http://127.0.0.1:8000/v1/params | python3 -m json.tool
```

```json
{
  "ok": true,
  "module": "center",
  "params": [
    {"key": "action.mode", "label": "动作映射模式", "value": "base_twist",
     "type": "enum", "options": ["raw", "base_twist", "hybrid", "joint"],
     "group": "动作映射", "editable": true},
    {"key": "action.max_linear_x", "label": "最大线速度 X", "value": 0.5,
     "type": "float", "unit": "m/s", "min": 0.0, "max": 5.0,
     "group": "速度限幅", "editable": true}
  ]
}
```

`POST` 使用 `{"updates": {...}}`：

```bash
curl -X POST http://127.0.0.1:8000/v1/params \
  -H 'Content-Type: application/json' \
  -d '{"updates":{"action.mode":"raw","action.max_linear_x":0.7}}'
```

可写参数：`action.mode`（`raw`/`base_twist`/`hybrid`/`joint`）、
`action.normalized`、`action.max_linear_x`、`action.max_linear_y`、
`action.max_angular_z`、`action.deadband`、`action.arm_start`、
`action.round_digits`。修改会重建 `ActionMapper`，下一次推理立即生效；
不重新加载模型。重启服务后回到 `config.yaml`。

### 4.2 请求：`POST /v1/infer`

```json
{
  "instruction": "把红色杯子拿起来",
  "images": {
    "front": "data:image/jpeg;base64,/9j/4AAQSkZJRg...",
    "wrist": "/9j/4AAQSkZJRg..."
  },
  "state": [0.0, 0.0, 0.0, 0.0, 1.57, 0.0, 1.0, 0.5],
  "task_id": "pick-cup-001",
  "session_id": "robot-01",
  "reset": false,
  "options": {
    "chunk_size": 8,
    "return_mapped": false
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `instruction` | string | 是 | 语言指令，≤512 字符；也接受别名 `task`/`language_instruction` |
| `images` | object | 视配置 | `{相机名: base64 或 data URI 或 {"data": "..."}}` |
| `state` | number[] | 否 | 本体状态向量；不足按 `pad_state` 补零，超出截断 |
| `task_id` | string | 否 | 回显；为空时用 `request_id` |
| `session_id` | string | 否 | 回显；当前服务无跨请求会话状态 |
| `reset` | bool | 否 | 为 true 时先调用 `policy.reset()` |
| `options.chunk_size` | int | 否 | 只返回动作块前 N 步，1..1000；默认返回完整 chunk |
| `options.return_mapped` | bool | 否 | 为 true 时额外返回每一步的映射结果 |

> 示例中的 `state` 是 8 维；实际维度以 `/v1/model` 返回的 `state_dim`
> 为准（加载后会被 checkpoint 覆盖）。

相机名：请求里可以用 `front`/`wrist` 等别名，通过
`model.image_aliases` 映射到 checkpoint 的相机键（如 `camera1`）。
缺失相机按 `model.missing_image_policy` 处理：`copy_first`（默认，复制第一路）、
`zeros`（补零）或 `error`（422）。

图像约定：RGB，任意分辨率；服务解成 `float32 CHW [0,1]`，后续缩放/归一化
由 LeRobot 的 SmolVLA processor 完成。单张图默认上限 6 MiB
（`server.max_image_bytes`），请求体默认 16 MiB（`server.max_body_bytes`）。

### 4.3 响应

```json
{
  "ok": true,
  "request_id": "8f0c...",
  "task_id": "pick-cup-001",
  "session_id": "robot-01",
  "instruction": "把红色杯子拿起来",
  "actions": [[0.12, -0.03, 0.05], [0.10, -0.02, 0.04]],
  "action": [0.12, -0.03, 0.05],
  "action_dim": 3,
  "chunk_size": 2,
  "base_cmd": {
    "linear": {"x": 0.06, "y": -0.012, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.05}
  },
  "arm_cmd": null,
  "mapped": {
    "mode": "base_twist",
    "raw": [0.12, -0.03, 0.05],
    "base_cmd": {"linear": {"x": 0.06, "y": -0.012, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.05}},
    "arm_cmd": null,
    "clipped": false
  },
  "model": {"backend": "smolvla", "path": "lerobot/smolvla_base", "device": "cuda", "dtype": "float16"},
  "latency_ms": 42.7,
  "model_latency_ms": 39.1,
  "timestamp": "2026-11-07T11:30:00+00:00",
  "warnings": []
}
```

- `actions`：模型原始动作块（每步一个向量），ROS 可按 `chunk_size` 顺序执行。
- `base_cmd`：**第一步**映射后的底盘速度（仅 `base_twist`/`hybrid` 模式）。
- `arm_cmd`：**第一步**映射后的机械臂关节（仅 `hybrid`/`joint` 模式）。
- `latency_ms`：服务端总耗时；`model_latency_ms`：模型调用耗时。
- `warnings`：state 补齐/图像复制等非致命提示。

### 4.4 错误码

HTTP 层统一返回：

```json
{
  "ok": false,
  "request_id": "...",
  "error": {"code": "model_unavailable", "message": "SmolVLA 模型加载失败: ...", "details": {}}
}
```

| HTTP | code | 场景 |
|---|---|---|
| 400 | `invalid_json`、`invalid_image`、`invalid_length` | JSON/图像/base64 格式错误 |
| 401 | `unauthorized` | 配置了 API Key 但请求缺失/错误 |
| 404 | `not_found` | 未知路径 |
| 405 | `method_not_allowed` | 方法不匹配 |
| 413 | `payload_too_large` | 请求体/图像超限 |
| 415 | `unsupported_media_type` | `Content-Type` 不是 JSON |
| 422 | `instruction_required`、`missing_image`、`unknown_image_key`、`invalid_state` 等 | 请求语义错误 |
| 503 | `model_unavailable` | torch/lerobot/权重不可用，或模型加载失败 |
| 500 | `inference_error`、`internal_error` | 推理/服务内部异常 |

### 4.5 示例：完整调用

```bash
python - <<'PY'
import base64, json, urllib.request
from pathlib import Path

image = base64.b64encode(Path("front.jpg").read_bytes()).decode()
payload = {
    "instruction": "go forward",
    "images": {"front": image},
    "state": [0.0, 0.0, 0.0, 0.0, 1.57, 0.0],
    "options": {"chunk_size": 1},
}
req = urllib.request.Request(
    "http://127.0.0.1:8000/v1/infer",
    data=json.dumps(payload).encode(),
    headers={"Content-Type": "application/json"},
    method="POST",
)
with urllib.request.urlopen(req, timeout=30) as resp:
    print(json.dumps(json.load(resp), ensure_ascii=False, indent=2))
PY
```

或使用自带客户端：

```python
import cv2
from center_server.clients import CenterServerClient

frame = cv2.imread("front.jpg")  # BGR
client = CenterServerClient("http://127.0.0.1:8000", timeout_sec=5)
result = client.infer(
    "把红色杯子拿起来",
    {"front": frame[:, :, ::-1]},       # 客户端按 RGB 编码
    state=[0.0, 0.0, 0.0, 0.0, 1.57, 0.0],
    task_id="pick-cup-001",
)
print(result["base_cmd"], result["arm_cmd"])
```

---

## 5. 动作映射

SmolVLA 输出动作块；服务按 `action.mode` 解释：

| mode | 行为 |
|---|---|
| `base_twist` | 取 `base_indices`（默认 `[0,1,2]`）为 `[vx, vy, wz]`，限幅后输出 `base_cmd` |
| `hybrid` | 前 3 维底盘速度 + `arm_start` 之后的机械臂关节 |
| `joint` | 全部维度按 `joint_names` 输出 `arm_cmd` |
| `raw` | 不做映射，只返回 `actions`/`action` |

`action.normalized`：

- `true`（默认）：模型输出在 `[-1, 1]`，乘以 `max_linear_x`/`max_linear_y`/
  `max_angular_z`；超出部分截断并标记 `clipped`；
- `false`：模型输出已是 m/s、rad/s，直接按 `max_*` 限幅。

其他参数：`action.deadband`（m/s，小于该值归零）、`action.invert`（逐轴取反）、
`action.joint_limits`（关节限位）。

> 对通用 `lerobot/smolvla_base`，输出动作通常对应训练机器人的关节/末端。
> 请先把 `action.mode` 设为 `raw` 或 `joint` 验证链路，再微调出适合本车的
> 动作空间。

---

## 6. 配置

完整默认值见 [`config.yaml`](config.yaml)。

| 配置 | 默认 | 说明 |
|---|---|---|
| `server.host` / `server.port` | `0.0.0.0:8000` | 监听地址 |
| `server.api_key` | 空 | 非空时保护写接口 |
| `server.max_body_bytes` | 16 MiB | 请求体上限 |
| `server.max_image_bytes` | 6 MiB | 单张图像解码前上限 |
| `server.request_timeout_sec` | 30 s | 连接读写超时 |
| `model.backend` | `smolvla` | `smolvla` 或 `stub` |
| `model.path` | `lerobot/smolvla_base` | HF 仓库名或本地目录 |
| `model.device` | `auto` | `cuda`/`cuda:0`/`cpu`/`mps` |
| `model.dtype` | `auto` | 保持 checkpoint dtype；可显式设 float16/bfloat16 |
| `model.image_keys` | `camera1..3` | 加载后被 checkpoint config 覆盖 |
| `model.image_aliases` | `front→camera1` 等 | 请求相机名映射 |
| `model.missing_image_policy` | `copy_first` | 缺失相机处理 |
| `model.require_images` | true | 是否必须有图像 |
| `model.require_state` | false | 是否必须有 state |
| `model.state_dim` | 6 | 加载后被 checkpoint state 维度覆盖 |
| `model.pad_state` | true | state 不足补零 |
| `model.chunk_size` | 50 | 加载后被 checkpoint chunk 覆盖 |
| `model.preload` | false | 启动即加载 |
| `model.apply_postprocessor` | true | 是否执行 LeRobot 后处理（反归一化） |
| `action.mode` | `base_twist` | 动作解释方式 |
| `action.normalized` | true | 模型输出是否归一化 |
| `action.max_linear_x` 等 | 0.5/0.4/1.0 | 速度限幅 |
| `handshake.enabled` | true | 是否向 Edge 握手（`edge_url` 为空时亦不发送） |
| `handshake.edge_url` | 空（生产配置示例 `http://${edge_server}`） | Edge Server 基地址 |
| `handshake.path` | `/api/center/handshake` | 握手指令路径（兼容旧版可改 `/api/modules/report`） |
| `handshake.interval_sec` | 10 s | 成功后的心跳间隔 |
| `handshake.retry_sec` / `max_retry_sec` | 2 s / 30 s | 失败指数退避 |
| `handshake.timeout_sec` | 3 s | 单次握手 HTTP 超时 |
| `handshake.api_key` | 空 | Edge 启用鉴权时使用 |
| `handshake.public_host` | 空 | 空=自动探测到 Edge 的出口 IP（Edge 仍以对端 IP 为准） |
| `handshake.service_name` | `SmolVLA Center Server` | dashboard 展示名 |

环境变量覆盖：`CENTER_SERVER_CONFIG`、`CENTER_SERVER_HOST`、`CENTER_SERVER_PORT`、
`CENTER_SERVER_API_KEY`、`CENTER_SERVER_BACKEND`、`CENTER_SERVER_MODEL_PATH`、
`CENTER_SERVER_DEVICE`、`CENTER_SERVER_DTYPE`、`CENTER_SERVER_EDGE_URL`、
`CENTER_SERVER_HANDSHAKE_ENABLED`、`CENTER_SERVER_HANDSHAKE_PATH`、
`CENTER_SERVER_PUBLIC_HOST`、`CENTER_SERVER_EDGE_API_KEY`。

---

## 7. ROS 接入

推荐在 ROS 侧新增/替换 `vla_inference_node` 的推理后端：

1. **发原始相机帧**，不要发 `MultiModalPreprocessor` 缩放后的 224×224 图；
   SmolVLA 需要较高分辨率。
2. **state 顺序固定**：例如 `[x, y, yaw, j1..j5]`，与微调数据一致；
   服务端只做补零/截断，不猜测语义。
3. **按 8–15 Hz 发请求**，但不要阻塞 ROS 定时器：用后台线程/异步队列，
   只保留最新帧，避免 Wi-Fi/GPU 抖动拖垮控制循环。
4. **只执行 `base_cmd`**，并再次经过 `safety_manager_node` 和
   `velocity_mux_node`；服务端限幅不是安全兜底。
5. **超时/失败降级**：HTTP 超时（建议 1–3 s）或非 200 时回退规则桩/停车，
   不要在无推理结果时继续执行旧动作。
6. **confidence**：SmolVLA 不提供校准置信度，服务端不伪造。ROS 侧以
   HTTP 成功 + 安全检查作为执行条件；可额外用动作块一致性做启发式门控。

```python
# ROS 节点内（示意）
from center_server.clients import CenterServerClient, CenterServerError

client = CenterServerClient("http://192.168.1.50:8000", timeout_sec=2.0)

try:
    result = client.infer(
        instruction=task.language_instruction,
        images={"front": rgb_frame},
        state=[x, y, yaw, *joint_values],
        chunk_size=8,
        task_id=task.task_id,
    )
    cmd = result["base_cmd"]
    # 再发布 /cmd_vel_raw，交给 velocity_mux/safety，而不是直接发 /cmd_vel
except CenterServerError as exc:
    node.get_logger().warn(f"VLA center server unavailable: {exc}")
    # fallback: 规则桩 / 停车
```

> **机载已实现**：`software/edge_server` 的 `VLAEngine` 支持 `backend: auto`，
> 通过 Edge 注册表发现本服务并异步调用 `/v1/infer`；Center 不可用时自动回退
> 规则桩（`remote` 模式下发 stop）。客户端在
> [`edge_server/core/center_client.py`](../edge_server/edge_server/core/center_client.py)，
> 图像默认按 BGR（`cv_bridge bgr8`）编码，state 为 `[x,y,yaw]+关节`。

---

## 8. 性能、容量与安全

- **GPU**：SmolVLA 约 450M 参数；float16 权重约 1 GB，建议 ≥6 GB 显存。
  本现场 GTX 1050 Ti 4GB 实测可跑（float16、256×256、单步 ~1.2s），但显存余量小；
  CPU 可运行，单次推理明显更慢，不适合闭环。
- **并发**：服务用 `ThreadingHTTPServer` 收请求，引擎用 `RLock` 串行推理；
  多客户端会排队。单机器人场景足够，多机器人建议多进程/多卡。
- **延迟**：局域网 HTTP + JPEG 传输通常 <20 ms；模型推理是主要耗时。
  用 `chunk_size` 摊薄推理频率，客户端顺序执行动作块。
- **安全**：服务默认无 TLS；生产环境用 Nginx/Caddy 终止 TLS，只监听内网/
  WireGuard，设置 `server.api_key`，不要把 8000 端口直接暴露公网。
- **资源限制**：`server.max_body_bytes`、单图 6 MiB、图像像素上限 2500 万；
  可按相机分辨率调整。
- **日志**：只记录方法/路径/状态/耗时，不记录图像或 API Key。
- **反向代理示例**：

```nginx
location /v1/ {
    proxy_pass http://127.0.0.1:8000;
    client_max_body_size 16m;
    proxy_read_timeout 60s;
}
```

- **就绪探针**：默认懒加载，`/v1/ready` 在首次推理前返回 503。若用
  Kubernetes/负载均衡的 readiness 探针，请设置 `model.preload: true`，
  或把探针指向 `/healthz`（仅表示进程存活）。

---

## 9. 现场部署与运维手册（192.168.3.25 ⇄ 192.168.3.26）

> 本节记录 Rush-MOG 当前实验室部署的**完整生命周期**：首次部署、日常增量
> 开发、构建、发布、部署、运行、维护与回滚。命令按本机路径写好，换机器时
> 只需替换 IP、盘符和用户名。

### 9.1 拓扑与端口

| 角色 | 地址 | 系统/硬件 | 入口 | 代码/部署目录 |
|---|---|---|---|---|
| **Center Server** | `192.168.3.25:8000` | Windows 10/11，Python 3.10，GTX 1050 Ti 4GB | `http://192.168.3.25:8000` | `F:\CODE\rush-mog\software\center_server` |
| **Edge Server** | `192.168.3.26:80` | Raspberry Pi 4B，Ubuntu 22.04 arm64，ROS 2 Humble | `http://192.168.3.26`（MJPEG `:8080`，健康网关 `127.0.0.1:8090`） | `/opt/edge_server`（colcon 工作区） |
| 开发仓库 | `F:\CODE\rush-mog` | Windows / WSL | — | — |

数据流：

```text
Edge 192.168.3.26:80                        Center 192.168.3.25:8000
  /api/center/handshake  ◄── 启动/每 10s ──  handshake.py
  /api/center（注册表）  ──  GET ─────────►  vla_inference_node（后续启用）
  /v1/infer              ◄── 图像+state ───  center_client.py
```

端口清单：

| 端口 | 所在 | 用途 | 放通策略 |
|---|---|---|---|
| 8000/TCP | Center | `/v1/*` HTTP | 仅 `192.168.3.0/24` |
| 80/TCP | Edge | 网页/API/握手注册表 | 机载局域网 |
| 8080/TCP | Edge | MJPEG 图传 | 机载局域网 |
| 8090/TCP | Edge | ROS 健康网关 | 默认仅 `127.0.0.1` |

### 9.2 前置条件

**Center（Windows，192.168.3.25）**

- Python **3.10**（本机：`C:\Users\regiu\AppData\Local\Programs\Python\Python310\python.exe`）；
- 管理员 PowerShell（注册开机任务、防火墙规则）；
- 固定 IP `192.168.3.25`，能访问 `192.168.3.26:80`；
- 真实模型：NVIDIA 驱动 + CUDA 版 torch（本机 GTX 1050 Ti 4GB，使用
  torch 2.6.0+cu124 / torchvision 0.21.0+cu124）；
- 国内网络建议 `HF_ENDPOINT=https://hf-mirror.com`（脚本已内置）。

**Edge（树莓派，192.168.3.26）**

- Ubuntu 22.04 + ROS 2 Humble，已安装 `cv_bridge`/`colcon`/`python3-opencv`；
- 部署目录 `/opt/edge_server`（`src/`、`build/`、`install/`、`config/`、`deploy/`）；
- systemd 服务 `edge_server`，运行用户 `ubuntu`，绑定 80 端口；
- 配置 `/opt/edge_server/config/deploy.yaml`（**不在仓库内**，需要单独维护）；
- 开发机可免密 `ssh ubuntu@192.168.3.26`、`rsync`。

### 9.3 首次部署

部署顺序：**先 Edge（提供 `/api/center` 握手入口），再 Center（启动即握手）**。

#### 9.3.1 Edge Server

```bash
# 在开发机 F:\CODE\rush-mog\software 下（WSL/Git Bash）
cd /mnt/f/CODE/rush-mog/software

# 1) 备份现有部署（回滚用）
ssh ubuntu@192.168.3.26 'TS=$(date +%Y%m%d-%H%M%S); \
  sudo mkdir -p /opt/edge_server/backups; \
  sudo tar -C /opt/edge_server/src -czf /opt/edge_server/backups/src-$TS.tar.gz edge_server custom_msgs; \
  sudo cp /opt/edge_server/config/deploy.yaml /opt/edge_server/backups/deploy-$TS.yaml; echo $TS'

# 2) 同步源码（改了什么同步什么；全量见下）
rsync -az --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  edge_server/ ubuntu@192.168.3.26:/opt/edge_server/src/edge_server/
rsync -az --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  custom_msgs/ ubuntu@192.168.3.26:/opt/edge_server/src/custom_msgs/

# 3) 构建 + 重启（只改 edge_server 时 --packages-select edge_server 即可）
ssh ubuntu@192.168.3.26 'bash -lc "source /opt/ros/humble/setup.bash && \
  cd /opt/edge_server && \
  colcon build --packages-select custom_msgs edge_server && \
  sudo systemctl restart edge_server"'

# 4) 验证
curl -s -o /dev/null -w '%{http_code}\n' http://192.168.3.26/status       # 200
curl -s -o /dev/null -w '%{http_code}\n' http://192.168.3.26/api/center   # 404 = 尚未握手（正常）
ssh ubuntu@192.168.3.26 'systemctl is-active edge_server; journalctl -u edge_server -n 30 --no-pager'
```

在 `/opt/edge_server/config/deploy.yaml` 末尾追加（缩进 2 空格，与 `edge_server:`
同级；该文件已包含 `lidar/base/imu/perception/edge/health`）：

```yaml
  vla_engine:
    backend: "auto"
    inference_rate: 10
    confidence_threshold: 0.6
    remote:
      enabled: true
      discovery_url: "http://127.0.0.1/api/center"   # 本机 Edge Web
      discovery_interval_sec: 5.0
      discovery_timeout_sec: 1.0
      host: "192.168.3.25"                          # 注册表不可用时的静态回退
      port: 8000
      api_key: ""
      timeout_sec: 3.0     # 实测 1050 Ti 单次约 1.2s，留余量
      max_rate_hz: 2.0
      chunk_size: 1
      image_key: "front"
      color_order: "bgr"
      state_joint_names: []
      confidence: 0.9
      turn_threshold: 0.15
      linear_threshold: 0.02
      retry_after_sec: 5.0
      fallback_to_rules: true
```

> 当前 deploy launch（`software/edge_server/launch/edge_server_deploy.launch.py`）只启动
> `edge_server_node` + `health_monitor_node` 这两个节点；需要真实远程 VLA 闭环时，
> 把 `vla_inference_node` 加进该 launch，并确保 `/center/synced_data` 有数据源。
> 只验证握手/健康时不需要。

#### 9.3.2 Center Server（Windows）

```powershell
# 管理员 PowerShell
cd F:\CODE\rush-mog\software\center_server

# 1) venv + 基础依赖 + 防火墙(192.168.3.0/24 -> 8000) + 开机任务 + 立即启动
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\install_windows.ps1

# 2) 真实推理栈（torch 2.6.0+cu124 / torchvision 0.21.0+cu124 / lerobot[smolvla]）
#    约 2.5GB；脚本用 curl 断点续传到 <repo>\wheels\，再本地安装
#    （pip 直连大 wheel 在国内镜像上容易停滞）
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\install_vla.ps1
# 无 GPU / 显存不足时只装 CPU 版（约 200MB）：
# powershell ... -File deploy\windows\install_vla.ps1 -Cpu

# 3) 下载权重（走 hf-mirror，默认 <repo>\models\smolvla_base）
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\download_model.ps1

# 4) 重启使依赖/模型生效
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\stop_center.ps1
Start-ScheduledTask -TaskName rush-mog-center
```

关键路径：

| 项目 | 路径 |
|---|---|
| 代码/venv | `F:\CODE\rush-mog\software\center_server` / `.venv` |
| 模型 | `F:\CODE\rush-mog\models\smolvla_base` |
| torch wheel 缓存 | `F:\CODE\rush-mog\wheels`（可删除，重新下载即可） |
| HF 缓存 | `C:\ProgramData\rush-mog-center\hf-cache` |
| 应用日志 | `C:\ProgramData\rush-mog-center\stderr.log` |
| 包装日志/PID | `C:\ProgramData\rush-mog-center\center_server.log` / `center.pid` |

如果不用脚本，等价的手工命令（大 wheel 建议先用 `curl -C -` 下载）：

```powershell
py -3.10 -m venv .venv
$env:PYTHONUTF8="1"
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
curl.exe -L -C - -o F:\CODE\rush-mog\wheels\torch-2.6.0+cu124-cp310-cp310-win_amd64.whl `
  "https://mirrors.aliyun.com/pytorch-wheels/cu124/torch-2.6.0%2Bcu124-cp310-cp310-win_amd64.whl"
.\.venv\Scripts\python.exe -m pip install --no-deps `
  F:\CODE\rush-mog\wheels\torch-2.6.0+cu124-cp310-cp310-win_amd64.whl
.\.venv\Scripts\python.exe -m pip install -r requirements-vla.txt
.\.venv\Scripts\python.exe -m center_server --config config.yaml --check
```

Linux/GPU 主机或 Docker 部署见 [`deploy/README.md`](deploy/README.md)；
`systemd` 单元、Dockerfile、compose 均已提供，代码与配置完全相同。

#### 9.3.3 联调验收（必须全过）

```bash
# Center 本机（Windows PowerShell）
Invoke-RestMethod http://127.0.0.1:8000/v1/health | ConvertTo-Json -Depth 5
Invoke-RestMethod http://127.0.0.1:8000/v1/handshake | ConvertTo-Json -Depth 5 -Encoding UTF8
Invoke-RestMethod http://127.0.0.1:8000/v1/ready

# 从 Edge 访问 Center（跨机验证，排除防火墙/路由问题）
ssh ubuntu@192.168.3.26 'curl -s -o /dev/null -w "health=%{http_code}\n" http://192.168.3.25:8000/v1/health'

# Edge 已记录 Center 的 IP/端口/推理地址
curl -s http://192.168.3.26/api/center | python3 -m json.tool
```

验收标准：

1. `/v1/health` 返回 `ok=true`，模型加载后 `ready=true`；未装模型时
   `status=degraded` 属预期；
2. `/v1/handshake` 的 `connected=true`、`last_error=""`、`last_response.infer_url`
   指向 `http://192.168.3.25:8000/v1/infer`；
3. Edge `GET /api/center` 返回 `ip=192.168.3.25`、`port=8000`、`stale=false`；
4. 从 Edge `curl` Center 的 `/v1/health` 为 200（Windows 防火墙放通）；
5. 模型就绪后执行一次真实推理：

```bash
# 预热：首次会下载 HuggingFaceTB/SmolVLM2-500M-Video-Instruct 基座（约 2GB），
# 可能数分钟；之后从 HF_HOME 缓存加载
curl -s -X POST http://192.168.3.25:8000/v1/warmup | python3 -m json.tool

curl -s -X POST http://192.168.3.25:8000/v1/infer \
  -H 'Content-Type: application/json' \
  -d '{"instruction":"前进","images":{},"state":[0,0,0]}' | python3 -m json.tool
```

> 首次加载的下载目标是 `HF_HOME\hub`（本现场
> `C:\ProgramData\rush-mog-center\hf-cache\hub`），走 `HF_ENDPOINT`
> （`start_center.ps1` 默认 `https://hf-mirror.com`）。缓存完成后可设置
> `HF_HUB_OFFLINE=1` 完全离线启动。

本次部署实测（2026-11-15，Center 192.168.3.25 / Edge 192.168.3.26）：

| 项目 | 结果 |
|---|---|
| Edge 构建/重启 | `colcon build` 2 包通过，`edge_server` active，`/status` 200 |
| Center 握手 | `connected=true`，Edge `/api/center` 记录 `192.168.3.25:8000` |
| 模型加载 | `loaded=true`、`device=cuda`、`dtype=float16`、`processors=true` |
| 首次 warmup | 含 `SmolVLM2-500M-Video-Instruct` 基座下载（约 2GB）约 10 分钟 |
| 单次推理 | `/v1/infer` 200，`model_latency_ms` ≈ 1.1–1.3s（GTX 1050 Ti） |
| 跨机访问 | Edge→Center `/v1/health` 200；Edge 重启后 10s 内自动重新握手 |

> 上述动作/`base_cmd` 来自通用 `smolvla_base`，只证明链路与显存/延迟；
> 控制本车仍需换成微调 checkpoint（动作空间 `[vx, vy, wz]`）。

### 9.4 增量开发 → 构建 → 发布 → 部署

统一原则：**改代码 → 本地测试 → 提交/打 tag → 同步到目标机 → 构建（按需）
→ 重启 → 验收**。Center 是纯 Python，无编译步骤；Edge 是 ROS 2 包，必须
`colcon build`。

#### 9.4.1 版本与发布（建议每次上线打 tag）

```bash
cd /mnt/f/CODE/rush-mog
# 1) 改版本号：center_server/__init__.py 的 __version__、pyproject.toml
# 2) 本地测试
cd software/center_server && python3 -m pytest tests -q
cd ../edge_server && python3 -m pytest tests -q
# 3) 提交并打 tag（示例）
git add -A && git commit -m "center: ..."
git tag -a center-v0.2.0 -m "Center Server 0.2.0 (Edge handshake + Windows deploy)"
git push origin master --tags   # 按实际远端/分支
```

发布物 = Git tag + 目标机代码目录（Center 另有 `.venv` 与模型目录）。
`/v1/health` 的 `version` 和握手 payload 的 `version` 都来自 `__version__`，
启动后先确认版本再验收。

#### 9.4.2 Edge 增量构建与部署

| 改动范围 | 命令要点 |
|---|---|
| 只改 `edge_server/**` | `rsync edge_server/` → `colcon build --packages-select edge_server` → restart |
| 改 `custom_msgs/**` | `rsync custom_msgs/` + `edge_server/` → `colcon build --packages-select custom_msgs edge_server` → restart |
| 改 `deploy.yaml` | 直接编辑远端配置 → restart（无需 build） |

```bash
cd /mnt/f/CODE/rush-mog/software
rsync -az --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  edge_server/ ubuntu@192.168.3.26:/opt/edge_server/src/edge_server/
ssh ubuntu@192.168.3.26 'bash -lc "source /opt/ros/humble/setup.bash && \
  cd /opt/edge_server && colcon build --packages-select edge_server && \
  sudo systemctl restart edge_server"'
```

- 单包增量构建通常 5–30s；`custom_msgs` 变更时首次构建可能 3–5min；
- 构建失败不会替换 `install/` 里的旧版本，服务仍在跑旧代码；修复后重跑即可；
- **回滚**：`sudo tar -C /opt/edge_server/src -xzf /opt/edge_server/backups/src-<TS>.tar.gz`
  → 重新 `colcon build` → `sudo systemctl restart edge_server`。

#### 9.4.3 Center 增量构建与部署

Center 无编译步骤，分三种情况：

```powershell
# A. 只改 Python 代码/配置（最常见）
cd F:\CODE\rush-mog\software\center_server
.\.venv\Scripts\python.exe -m pytest tests -q
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\stop_center.ps1
Start-ScheduledTask -TaskName rush-mog-center

# B. 改依赖（requirements*.txt / 新增 import）
.\.venv\Scripts\python.exe -m pip install -r requirements.txt   # 或 requirements-vla.txt
# 然后按 A 重启

# C. 改 Windows 服务/防火墙/IP（start_center.ps1 / config.yaml）
#    非管理员可在仓库目录改 start_center.ps1 的环境变量，然后
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\install_windows.ps1 -SkipVenv
```

- 重启后先看日志：`Get-Content C:\ProgramData\rush-mog-center\stderr.log -Tail 50 -Encoding UTF8`；
- 本地快速验证不占用正式 8000 端口时，可前台起 stub：
  `.\.venv\Scripts\python.exe -m center_server --config config.stub.yaml --port 18000`；
- **回滚**：`git checkout <上一个 tag>` → 重启任务；模型/依赖不需回退时立即生效。

#### 9.4.4 模型更新

```powershell
# 下载到新目录（保留旧版本便于回滚）
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\download_model.ps1 `
  -Repo my-org/rush-mog-smolvla -Dest F:\CODE\rush-mog\models\rush-mog-smolvla

# 修改 start_center.ps1 的 CENTER_SERVER_MODEL_PATH（或 config.yaml 的 model.path）
# 然后重启任务
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\stop_center.ps1
Start-ScheduledTask -TaskName rush-mog-center

# 预热并确认（首次加载会占显存，可能数十秒）
Invoke-RestMethod -Method Post http://127.0.0.1:8000/v1/warmup -TimeoutSec 300
Invoke-RestMethod http://127.0.0.1:8000/v1/ready
```

> 通用 `lerobot/smolvla_base` 只用于打通链路；控制本车必须换成在本车数据上
> 微调、动作空间为 `[vx, vy, wz]` 的 checkpoint，并在 `action` 段配对限幅。

### 9.5 运行

**Center（Windows 计划任务）**

```powershell
Get-ScheduledTask -TaskName rush-mog-center | Select TaskName,State
Get-NetTCPConnection -State Listen -LocalPort 8000
Invoke-RestMethod http://127.0.0.1:8000/v1/health | ConvertTo-Json -Depth 4
Invoke-RestMethod http://127.0.0.1:8000/v1/handshake | ConvertTo-Json -Depth 4
Get-Content C:\ProgramData\rush-mog-center\stderr.log -Tail 30 -Encoding UTF8
```

**Edge（systemd）**

```bash
ssh ubuntu@192.168.3.26 'systemctl status edge_server --no-pager -l | head -20'
ssh ubuntu@192.168.3.26 'journalctl -u edge_server -f'
curl -s http://192.168.3.26/status | python3 -m json.tool | head -30
curl -s http://192.168.3.26/api/center | python3 -m json.tool
```

**健康语义**

| 端点 | 200 含义 | 503/404 含义 |
|---|---|---|
| Center `/healthz` | 进程存活 | 进程未起/端口不通 |
| Center `/v1/ready` | 模型已加载 | 首次推理前（懒加载）或加载失败 |
| Center `/v1/health` | 服务信息；`status=degraded` 时看 `model.error` | — |
| Edge `/api/center` | 已收到 Center 握手 | 404 = 尚未握手（Edge 先启动/网络） |
| Edge `/status.ros_ready` | ROS 健康网关可用 | 看 `ros_status`/`journalctl` |

### 9.6 维护

**日常（建议每天/每次上车前）**

1. `curl -s http://192.168.3.26/api/center` 检查 `stale=false`、`ip/port` 正确；
2. Center `/v1/health` 的 `ready/status`、`handshake.connected`；
3. Edge `/status` 的 `serial_connected/encoder_ok/ros_status`；
4. 磁盘：`Get-PSDrive C,F`、`df -h /opt`（模型/缓存/日志）；
5. 时间同步：Windows `w32tm /resync`，Pi `timedatectl`（日志关联用）。

**日志**

- Center：`stderr.log` 无内置轮转；建议每周归档：

```powershell
$dir = "C:\ProgramData\rush-mog-center"
Move-Item "$dir\stderr.log" "$dir\stderr-$(Get-Date -Format yyyyMMdd).log" -Force
# 任务无需重启，Python 会在下次写日志时重新创建句柄
```

- Edge：`journalctl -u edge_server --since today`；磁盘紧张时
  `sudo journalctl --vacuum-time=7d`；
- 不要记录图像/base64/API Key（服务端已保证）；调试时用 `--log-level DEBUG`。

**重启/停止**

```powershell
# Center
powershell -File deploy\windows\stop_center.ps1
Start-ScheduledTask -TaskName rush-mog-center
# 或仅停止：Stop-ScheduledTask -TaskName rush-mog-center（再配合 stop_center.ps1）
```

```bash
# Edge
ssh ubuntu@192.168.3.26 'sudo systemctl restart edge_server'
ssh ubuntu@192.168.3.26 'sudo systemctl stop edge_server'
```

**升级策略**

- 先升级 **Edge**（向后兼容旧 Center：没有 `/api/center` 时旧版仅不显示）；
- 再升级 **Center**；Center 握手失败只影响注册表，不影响推理；
- 跨版本升级前打 tag、备份 Edge `src`/`deploy.yaml`、Center 保留旧模型目录；
- 若修改了 `/v1/infer` 字段或动作语义，先升级 Edge 的 `center_client`/映射再切 Center。

**安全与备份**

- 防火墙只放行 `192.168.3.0/24` 到 8000；不要把 8000 暴露公网；
- 可选 `server.api_key` + Edge `vla_engine.remote.api_key`（握手不传密钥）；
- 备份：Git tag（代码/配置）+ `models/` 旧目录（权重）+ Edge `/opt/edge_server/backups`；
- 密钥/Token 不写进仓库；`HF_TOKEN` 用环境变量。

### 9.7 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `pip install` 报 `UnicodeDecodeError: 'gbk'` | Windows 默认 GBK 读 UTF-8 requirements | `$env:PYTHONUTF8="1"` 后重试（脚本已内置） |
| 下模型超时/连不上 HF | 国内直连 HF 不通 | 使用 `HF_ENDPOINT=https://hf-mirror.com`（脚本默认） |
| 首次 `/v1/warmup` 很慢（数分钟） | 正在下载基座 `SmolVLM2-500M-Video-Instruct`（约 2GB） | 看 `HF_HOME\hub` 体积增长；缓存后重试即快；也可离线启动 |
| `/v1/ready` 一直 503 | 模型/依赖未装、路径错、显存不足 | `/v1/health.model.error`；`pip list \| findstr torch`；先 `/v1/warmup` 看堆栈 |
| `CUDA out of memory` | 4GB 显存不足/float32 | `CENTER_SERVER_DTYPE=float16`；仍 OOM 改 `CENTER_SERVER_DEVICE=cpu` |
| `TypeError: type variables ... not listed in Generic` | gymnasium 1.4 在 Python 3.10 上的 typing 兼容问题 | `pip install gymnasium==1.1.1`（`requirements-vla.txt` 已固定） |
| Edge `/api/center` 404 | Center 未启动/握手失败 | Center `/v1/handshake` 的 `last_error`；确认 Edge 已升级到含该路由的版本 |
| Edge `/api/center` 的 IP 不对 | 多网卡/抓错出口 | Center 配置 `handshake.public_host=192.168.3.25`；Edge 以 TCP 对端为准 |
| Center 能本机访问，Edge 访问 8000 超时 | Windows 防火墙/网络位置 | 检查 `Rush-MOG Center Server (8000)` 规则、`192.168.3.0/24` 源地址；Edge `curl` 验证 |
| WSL 访问 `192.168.3.25:8000` 失败 | WSL NAT 源地址不在放行网段 | 正常现象；用 Edge 或 Windows `127.0.0.1` 验证 |
| 推理 422 `未知相机名: front` | Center 相机键/别名不匹配 | 看握手 `meta.image_keys`；Center `model.image_aliases` 加 `front: camera1` |
| 动作全 0/不动 | `action.mode=raw` 或未微调策略 | 先确认 `/v1/infer` 的 `base_cmd`；通用 base checkpoint 不能直接控车 |
| Edge 构建失败 | 代码/消息定义错误 | 看 `colcon build` 首个 error；旧 `install/` 仍可用，修复后重建 |
| 计划任务 Running 但 8000 不通 | Python 启动即退出/端口占用 | `stderr.log`；`Get-NetTCPConnection -LocalPort 8000`；`stop_center.ps1` 后重启 |

### 9.8 命令速查

```text
Center 启动/停止/重启（Windows 管理员）
  install_windows.ps1                        首次安装+防火墙+开机任务
  stop_center.ps1                            停止
  Start-ScheduledTask -TaskName rush-mog-center
  Get-Content C:\ProgramData\rush-mog-center\stderr.log -Tail 50 -Encoding UTF8

Center 自检
  Invoke-RestMethod http://127.0.0.1:8000/v1/health
  Invoke-RestMethod http://127.0.0.1:8000/v1/handshake
  Invoke-RestMethod http://127.0.0.1:8000/v1/ready

Edge 构建/部署/重启
  rsync -az --delete --exclude=__pycache__ edge_server/ ubuntu@192.168.3.26:/opt/edge_server/src/edge_server/
  ssh ubuntu@192.168.3.26 'bash -lc "source /opt/ros/humble/setup.bash && cd /opt/edge_server && colcon build --packages-select edge_server && sudo systemctl restart edge_server"'

注册表/链路
  curl -s http://192.168.3.26/api/center | python3 -m json.tool
  ssh ubuntu@192.168.3.26 'curl -s -o /dev/null -w "%{http_code}\n" http://192.168.3.25:8000/v1/health'
```

---

## 10. 测试

不依赖 GPU/torch：

```bash
cd software/center_server
pip install -r requirements-dev.txt
python3 -m pytest tests -q
```

覆盖：配置校验、动作映射/限幅/关节限位、图像编解码、HTTP 路由/鉴权/错误码、
socket 服务与自带客户端、模型缺失时 503、`/v1/params` 参数查询/修改/校验、
握手负载/重试/心跳/`/v1/handshake`（共 57 项）。机载侧另有
`edge_server/tests/test_center_handshake.py`、`test_vla_remote.py` 覆盖
注册表与远程推理降级。

手工联调：

```bash
python -m center_server --config config.stub.yaml
curl -s -X POST http://127.0.0.1:8000/v1/infer \
  -H 'Content-Type: application/json' \
  -d '{"instruction":"前进","images":{},"state":[0,0,0]}' | python3 -m json.tool
```

---

## 11. 当前限制

1. **base checkpoint 不是本车策略**：必须微调动作空间/数据；当前只保证
   推理管线、HTTP 协议和动作映射可运行。
2. **无校准置信度**：SmolVLA 输出动作块，不提供可靠 uncertainty；低置信度
   门控需在 ROS/上层实现。
3. **动作块非闭环**：服务无跨请求会话状态；`session_id` 仅回显。若策略依赖
   历史，需在客户端维护或在服务端扩展 session。
4. **单模型/单进程**：不支持多模型热切换、多租户、批处理。
5. **未做硬件在环**：真实相机/Wi-Fi/GPU 延迟需实车压测。
6. **图像语义**：服务端按 RGB 解码；机载客户端已把 OpenCV BGR 帧编码为
   JPEG 后再上传（`CenterClient.color_order="bgr"`），自定义客户端需自行保证。
7. **握手注册表在内存**：Edge 重启后需等待下一次 heartbeat（默认 ≤10s）才能
   重新发现；不落盘、不支持多 Center 同时注册（同 module 覆盖）。
8. **Edge 最小部署未默认启动 `vla_inference_node`**：当前
   `launch/edge_server_deploy.launch.py` 只跑 Web/健康两个节点（IMU、雷达等驱动
   由 `robot_bringup` 或单独 launch 拉起）；
   启用远程 VLA 闭环需把该节点加入 launch 并提供 `/center/synced_data`。

详细设计（模块、协议、错误处理、扩展点）见 [`DESIGN.md`](DESIGN.md)。
