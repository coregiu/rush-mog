# Center Server — SmolVLA HTTP 推理服务

> 版本：0.1.0  
> 运行环境：Python 3.10+；真实模型建议 NVIDIA GPU（CPU 可跑但延迟高）  
> 接口：HTTP/JSON，`POST /v1/infer`、`GET/POST /v1/params`

本目录是 Rush-MOG 架构中的**中心推理模块**。机载 ROS（树莓派 4B/5 或 Jetson
Orin Nano）负责相机/激光/里程计采集与底盘安全控制；中心服务器加载 SmolVLA，
把语言指令 + 多路图像 + 本体状态推理成动作块（action chunk），再通过 HTTP
返回给 ROS。

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
└──────────────┬───────────────┘                         └──────────────────────────────┘
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
│   ├── http_api.py          # 路由（纯函数，便于单测）
│   ├── images.py            # base64/data URI -> float32 CHW
│   ├── server.py            # ThreadingHTTPServer + CLI
│   ├── service.py           # 校验/推理/响应组装
│   └── clients/http_client.py  # 给 ROS/调试脚本用的轻量客户端
├── tests/                   # 单元 + HTTP 端到端（不需要 GPU）
├── config.yaml              # 生产配置（smolvla 后端）
├── config.stub.yaml         # 联调配置（stub 后端）
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
| OPTIONS | `*` | CORS 预检 | 否 |

\* 仅当 `server.api_key` 非空时要求鉴权。支持
`Authorization: Bearer <key>` 或 `X-API-Key: <key>`。

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

环境变量覆盖：`CENTER_SERVER_CONFIG`、`CENTER_SERVER_HOST`、`CENTER_SERVER_PORT`、
`CENTER_SERVER_API_KEY`、`CENTER_SERVER_BACKEND`、`CENTER_SERVER_MODEL_PATH`、
`CENTER_SERVER_DEVICE`、`CENTER_SERVER_DTYPE`。

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

> 现有 `software/edge_server` 的 `VLAEngine` 仍是规则桩。若要正式接入，建议把 HTTP
> 调用放到独立异步节点，保留规则桩作为降级路径，避免同步网络阻塞控制循环。

---

## 8. 部署与性能

- **GPU**：SmolVLA 约 450M 参数；float16 权重约 1 GB，建议 ≥6 GB 显存。
  CPU 可运行，但单次推理可能数百毫秒到数秒，不适合闭环。
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

## 9. 测试

不依赖 GPU/torch：

```bash
cd software/center_server
pip install -r requirements-dev.txt
python3 -m pytest tests -q
```

覆盖：配置校验、动作映射/限幅/关节限位、图像编解码、HTTP 路由/鉴权/错误码、
socket 服务与自带客户端、模型缺失时 503、`/v1/params` 参数查询/修改/校验。

手工联调：

```bash
python -m center_server --config config.stub.yaml
curl -s -X POST http://127.0.0.1:8000/v1/infer \
  -H 'Content-Type: application/json' \
  -d '{"instruction":"前进","images":{},"state":[0,0,0]}' | python3 -m json.tool
```

---

## 10. 当前限制

1. **base checkpoint 不是本车策略**：必须微调动作空间/数据；当前只保证
   推理管线、HTTP 协议和动作映射可运行。
2. **无校准置信度**：SmolVLA 输出动作块，不提供可靠 uncertainty；低置信度
   门控需在 ROS/上层实现。
3. **动作块非闭环**：服务无跨请求会话状态；`session_id` 仅回显。若策略依赖
   历史，需在客户端维护或在服务端扩展 session。
4. **单模型/单进程**：不支持多模型热切换、多租户、批处理。
5. **未做硬件在环**：真实相机/Wi-Fi/GPU 延迟需实车压测。
6. **图像语义**：默认按 RGB 处理；ROS `cv_bridge` 的 `bgr8` 帧需先转 RGB。

详细设计（模块、协议、错误处理、扩展点）见 [`DESIGN.md`](DESIGN.md)。
