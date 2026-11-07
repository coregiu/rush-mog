# Center Server 详细设计（SmolVLA HTTP 推理）

> 版本：0.1.0（2026-11）  
> 对应上位机文档：[README.md](README.md)  
> 对应机载 ROS 包：[`software/ros`](../ros/README.md)

---

## 0. 范围与状态

| 项目 | 状态 |
|---|---|
| 代码 | `software/center_server`（纯 Python，标准库 HTTP） |
| 推理后端 | SmolVLA（LeRobot）+ `stub` 联调后端 |
| 接口 | HTTP/JSON，`/v1/infer`、`/v1/health`、`/v1/model`、`/v1/params`、`/v1/reset`、`/v1/warmup` |
| 测试 | `pytest tests -q`：38 passed（无 GPU/torch） |
| 真实权重 | 不随仓库分发；默认 `lerobot/smolvla_base` 或本地微调 checkpoint |
| 实车验证 | 未做硬件在环；需联调相机、Wi-Fi、GPU 延迟 |

本文描述的是**中心推理服务**，不包含机载 ROS 的 SLAM/导航/速度仲裁；
那部分在 `software/ros` 的 center_server v2 包中。

---

## 1. 设计目标

1. **职责单一**：只做“多模态输入 → SmolVLA 动作块 → 底盘/机械臂语义映射”，
   不直接发布 `/cmd_vel`、不直接操作串口。
2. **协议稳定**：HTTP/JSON 字段固定，ROS 侧可独立升级；错误码可区分
   “请求错误 / 模型未就绪 / 推理失败”。
3. **可降级**：模型/GPU/torch 不可用时返回 503，绝不伪造动作；规则桩仍由
   机载 ROS 保留，作为网络或模型故障时的降级路径。
4. **可测试**：HTTP 路由与业务层解耦，模型层可替换为 `stub`，CI 不需要 GPU。
5. **可移植**：基础服务只依赖 `numpy`/`Pillow`/`PyYAML`；`torch`/`lerobot`
   仅在真实后端加载时导入。
6. **安全边界清晰**：服务端只做速度限幅；急停、避障、仲裁、看门狗必须在
   机载 ROS/STM32 完成。

非目标：

- 不做 SLAM/建图/导航/目标检测（在 `software/ros`）；
- 不做多机器人调度、模型热切换、训练；
- 不做 TLS 终止（交给反向代理）。

---

## 2. 部署拓扑

```text
┌──────────────────────────────┐
│ 训练/模型准备（可选）          │
│ LeRobot + Rush-MOG 演示数据    │
│ -> checkpoint (HF / 本地)      │
└──────────────┬───────────────┘
               │ copy / download
┌──────────────▼───────────────┐         HTTP          ┌──────────────────────────────┐
│ Center Server（GPU/PC）       │ ◄──────────────────── │ 车载 ROS2（Pi 4B/5 / Orin）   │
│ FastAPI 无关，stdlib HTTP      │                       │ 相机/激光/里程计/关节         │
│ SmolVLA policy + processors   │ ────────────────────► │ vla_inference_node / bridge   │
│ action mapper                 │  actions/base_cmd     │ safety + velocity_mux         │
└──────────────────────────────┘                       └──────────────┬───────────────┘
                                                                      │ UART
                                                               ┌──────▼───────┐
                                                               │ STM32 + 底盘  │
                                                               └──────────────┘
```

部署选项：

| 选项 | 推理位置 | 说明 |
|---|---|---|
| A | 局域网 GPU 主机 | 推荐；ROS 只发图像/状态，收动作块 |
| B | Orin Nano 本机 | 无网络依赖，但 GPU 资源与 ROS 竞争 |
| C | CPU 主机 | 仅演示/低速；延迟可能不可接受 |

服务对 A/B/C 代码相同，只改 `server.host` 与 ROS 的 URL。

---

## 3. 模块划分

| 模块 | 职责 | 关键点 |
|---|---|---|
| `config.py` | YAML 加载、默认值合并、环境变量覆盖、校验 | 路径不存在/端口非法/未知 backend 直接失败 |
| `images.py` | base64/data URI/对象 → RGB float32 CHW | 解压炸弹与像素上限；EXIF 旋转 |
| `action_mapper.py` | 动作向量 → `base_cmd`/`arm_cmd` | `raw`/`base_twist`/`hybrid`/`joint`；限幅/取反/死区/关节限位 |
| `engine.py` | SmolVLA 生命周期与推理 | 懒加载、RLock、processor 兼容、`predict_action_chunk`/`select_action` |
| `service.py` | 请求校验、图像/状态补齐、响应组装 | 先加载模型确定 schema，再校验图像键/state 维度 |
| `http_api.py` | 路由、鉴权、CORS、JSON/大小限制 | 纯函数 `dispatch()`，便于单测 |
| `server.py` | `ThreadingHTTPServer` + CLI/信号 | 每请求独立线程；引擎串行 |
| `clients/http_client.py` | ROS/调试客户端 | 标准库 urllib；图像自动编码 |

依赖方向：

```text
server -> http_api -> service -> engine -> torch/lerobot
                         └──> action_mapper
                         └──> images
config <- server/http_api/service/engine
errors <- 所有层
```

`engine.py` 不导入 `service`/`http_api`；`http_api.py` 不直接导入 torch。

---

## 4. 请求生命周期

```text
HTTP request
  │
  ├─ 1. Content-Length / body 上限
  ├─ 2. JSON 解析、Content-Type 校验
  ├─ 3. API Key 校验（/v1/* 受保护端点）
  │
  ▼
InferenceService.infer
  ├─ 4. instruction 校验（必填、≤512）
  ├─ 5. engine.ensure_loaded()          # 首次加载模型，确定最终 schema
  ├─ 6. 图像解析/别名映射/缺失补齐       # 使用 checkpoint 的 image_keys
  ├─ 7. state 解析 + 补零/截断            # 使用 checkpoint 的 state_dim
  ├─ 8. chunk_size 校验
  ├─ 9. engine.infer()                  # 预处理 -> policy -> 后处理 -> numpy
  ├─ 10. ActionMapper.map() 每步动作     # 第一步写 base_cmd/arm_cmd
  └─ 11. 组装响应（actions/action/...）
```

### 4.1 模型加载状态机

```text
        ┌──────────┐  ensure_loaded  ┌──────────┐  success  ┌────────┐
        │ NOT_LOAD │ ──────────────► │ LOADING  │ ────────► │ LOADED │
        └──────────┘                 └────┬─────┘           └────────┘
                                           │ failure
                                           ▼
                                     ┌──────────┐  reload_interval 后重试
                                     │  ERROR   │ ─────────────────────┐
                                     └──────────┘                      │
                                           └───────────────────────────┘
```

- `ensure_loaded()` 用 `RLock` 串行化，避免并发请求重复加载。
- 加载失败缓存 `error/error_at`；`reload_interval_sec` 内直接 503，避免每个
  请求都触发昂贵的 import/下载/显存分配。
- 预加载 `model.preload=true` 失败**不退出进程**：服务仍可提供 `/health`，
  请求时按间隔重试，便于模型文件晚挂载/网络恢复。
- 默认懒加载时 `/v1/ready` 在首次推理前为 503；使用 readiness 探针的
  编排系统应设 `preload: true`，或改用 `/healthz` 仅做存活检查。

### 4.2 为什么先加载再校验图像

不同 checkpoint 的相机键可能不是配置里的 `camera1..3`。`SmolVLAEngine`
加载后会读取 `policy.config.image_features`/`input_features` 覆盖
`image_keys`，读取 `observation.state` 覆盖 `state_dim`，读取
`chunk_size`/`n_action_steps` 覆盖默认值。因此服务必须在解析请求图像之前
完成加载，否则会按错误 schema 校验。代价是首个非法请求也会触发模型加载；
生产环境可用 `preload: true` 规避。

---

## 5. HTTP 协议

### 5.1 设计约定

- JSON 字段使用 `snake_case`；动作/坐标沿用 ROS 习惯。
- 所有错误返回 `{"ok": false, "error": {"code", "message", "details?"}}`，
  并带 `request_id`；HTTP 状态码表达类别。
- 成功的推理响应同时包含原始动作块和**第一步**映射结果，避免 ROS 侧重复
  实现映射逻辑；后续步骤的映射可按需 `return_mapped`。
- 不使用 HTTP chunked 请求；客户端必须带 `Content-Length`（urllib/requests
  默认满足）。
- 无状态：`session_id` 只回显；策略动作队列由 `policy.reset()` 清理，
  不做多 session 隔离。多机器人应多进程部署。

### 5.2 端点语义

| 端点 | 幂等 | 说明 |
|---|---|---|
| `GET /v1/health` | 是 | 200 返回详细状态；`ready` 表示模型是否可用 |
| `GET /v1/ready` | 是 | 就绪探针；未加载返回 503 |
| `GET /v1/model` | 是 | schema/动作模式/错误信息，不触发模型加载 |
| `GET /v1/params` | 是 | 动作映射/限幅参数描述 |
| `POST /v1/params` | 否 | 运行时修改 `action.*`，重建 ActionMapper，不重载模型 |
| `POST /v1/infer` | 否 | 推理；`reset=true` 先清策略队列 |
| `POST /v1/reset` | 是 | 清策略内部队列/回合状态 |
| `POST /v1/warmup` | 否 | 空推理；首次会加载模型 |

### 5.3 图像传输

- 客户端发送 base64（可带 `data:image/jpeg;base64,` 前缀）；服务端不接收
  文件路径/URL，避免 SSRF 与任意文件读取。
- 解码后统一 RGB、`float32 CHW [0,1]`；缩放/归一化由 LeRobot
  processor 完成，服务端不硬编码 224/512。
- 单图默认 6 MiB（`server.max_image_bytes`）、单请求 16 MiB；像素上限
  2500 万，防止解压炸弹。
- 缺失相机策略：
  - `copy_first`：单目部署时把第一路复制到其余相机（默认）；
  - `zeros`：补黑图；
  - `error`：422，要求客户端补齐。
- 图像语义按 RGB。ROS `cv_bridge` 的 `bgr8` 帧需转 RGB 后再编码；自带客户端
  的 `encode_image_b64` 不做 BGR→RGB 猜测。

### 5.4 状态向量

`state` 是**有序**浮点数组，服务端不猜测语义：

- 长度 < `state_dim`：补 0 并 warning（`pad_state: true`）；
- 长度 > `state_dim`：截断并 warning；
- `require_state: true` 时缺失直接 422；
- 非数字/NaN/Inf/布尔值：422。

推荐在微调数据与 ROS 端使用同一顺序，例如
`[x, y, yaw, j1, j2, j3, j4, j5]`。SmolVLA checkpoint 的 `state_dim`
可能被 pad 到 32，服务端以 checkpoint 的 `observation.state` 特征维度为准。

---

## 6. SmolVLA 适配层

### 6.1 加载

```python
policy = SmolVLAPolicy.from_pretrained(path)   # HF repo id 或本地目录
policy.to(device)                              # cuda/cpu/mps
policy.to(dtype=torch.float16)                 # 可选；auto 时保持 checkpoint dtype
policy.eval()
```

兼容点：

- 导入路径：`lerobot.policies.smolvla.modeling_smolvla` 或
  `lerobot.policies.smolvla`；
- `from_pretrained` 的 `local_files_only`/`trust_remote_code` 参数在新旧版本
  可能不存在，失败时降级重试；
- 设备 `auto`：CUDA > MPS > CPU；配置 CUDA 但不可用时回退 CPU 并告警；
- `dtype: auto` 保持 checkpoint 原始 dtype；显式 `float16` 仅在 CUDA 使用，
  CPU/MPS 回退 `float32`；
- 加载失败统一包装为 503，附带安装提示与错误摘要。

### 6.2 预处理/后处理

SmolVLA 需要：

- 图像：checkpoint 相机键对应张量；
- 语言：`task` 字符串 → tokenizer；
- 状态：按数据集统计归一化；
- 动作：按数据集统计反归一化。

这些由 LeRobot 的 `make_pre_post_processors(policy_cfg, pretrained_path)`
管线完成。`model.require_processors: true` 时缺失该 API 直接 503，不用
“猜”归一化参数。`preprocessor_overrides={"device_processor": {"device": ...}}`
把张量搬到目标设备。

batch 形状（单样本）：

```python
{
  "observation.images.camera1": Tensor[1, 3, H, W],  # float32 [0,1]
  "observation.state":          Tensor[1, state_dim],
  "task":                       str,
}
```

processor 输出通常包含 `observation.language.tokens/attention_mask` 等；
`engine.py` 不假设字段名，直接交给 policy。

### 6.3 推理调用

优先 `policy.predict_action_chunk(batch)`，返回 `(B, T, action_dim)`；
老版本没有该 API 时退回 `policy.select_action(batch)` 循环 `n_action_steps`
并堆叠成动作块。随后：

1. 可选 `postprocessor(actions)`（兼容 dict/tensor 两种返回）；
2. 转 numpy，处理 `(B,T,D)`/`(T,D)`/`(1,D)`；
3. 取第一个 batch；
4. 校验 NaN/Inf、空动作；
5. 按请求 `chunk_size` 截断；
6. 转 Python float 并保留 6 位小数。

`predict_action_chunk` 是**无状态**的（不消耗策略内部队列），适合 HTTP
按需调用；`select_action` 会消耗队列，因此仅作为兼容回退。

### 6.4 动作语义

SmolVLA 只输出向量，不携带“这是速度还是关节”的语义。映射完全由
`action.mode` 决定：

| mode | `base_cmd` | `arm_cmd` |
|---|---|---|
| `raw` | null | null |
| `base_twist` | 前 3 维 → `[vx, vy, wz]` | null |
| `hybrid` | 前 3 维 | `arm_start` 之后 |
| `joint` | null | 全部维度 |

限幅顺序：归一化缩放 → 取反 → 死区 → 四舍五入。`clipped` 标记任一轴
超出归一化范围/物理上限或关节限位。

> `lerobot/smolvla_base` 是通用权重，动作空间取决于训练机器人，不是
> Rush-MOG 底盘策略。生产部署必须用本车数据微调，并在训练时确定动作空间
> （推荐 3 维速度），再把 `action.mode`/`base_indices`/`normalized` 与训练
> 配置对齐。

---

## 7. 配置与错误处理

### 7.1 配置优先级

1. `--config`；
2. `CENTER_SERVER_CONFIG`；
3. 仓库 `config.yaml`（CLI 默认探测）；
4. 内置默认值。

环境变量可覆盖 host/port/api_key/backend/path/device/dtype。配置校验在
启动时完成，非法值直接退出（exit 2），不会带病运行。

### 7.2 错误分类

| 层 | 异常 | HTTP |
|---|---|---|
| 配置 | `ConfigError` | 启动失败/exit 2 |
| HTTP | 长度/JSON/Content-Type/鉴权/路径/方法 | 400/401/404/405/413/415 |
| 服务 | instruction/图像/状态/chunk/相机名 | 422 |
| 引擎 | torch/lerobot/checkpoint/processor 不可用 | 503 |
| 引擎 | 推理异常/NaN/空动作 | 500 `inference_error` |
| 兜底 | 未捕获异常 | 500 `internal_error`（不泄漏堆栈） |

500 日志记录完整堆栈；响应只给异常类型/摘要，避免泄漏路径/环境信息。

### 7.3 降级

- 模型加载失败：503；不返回固定动作。
- 推理超时：客户端超时；服务端无法中断 CUDA kernel，需客户端丢弃结果并
  停车/回退规则桩。
- `stub` 后端只用于联调，生产配置不得使用。

---

## 8. 并发、性能与资源

- HTTP：`ThreadingHTTPServer`，每请求一个线程；适合低频（5–15 Hz）推理。
- 引擎：`RLock` 串行化加载与推理；多请求排队，避免 GPU 并发导致 OOM。
- 动作块：`chunk_size` 越大，推理频率越低，但客户端开环执行越久；
  建议 1–8 步，并保证底层速度看门狗/超时。
- 显存：SmolVLA ~450M 参数，float16 权重约 1 GB；建议 ≥6 GB 显存。
- CPU：可运行但延迟高；仅用于联调/演示。
- 请求体：base64 比二进制大约 33%；高分辨率多相机时优先 JPEG + 限流。
- 日志：每请求一行 method/path/status/latency；不记录图像/Key。

---

## 9. 安全设计

1. **网络**：默认监听 `0.0.0.0` 便于局域网调试；生产建议绑定内网地址，
   由 Nginx/Caddy 终止 TLS，公网不直接暴露。
2. **鉴权**：可选 Bearer/X-API-Key；`/v1/health`、`/v1/ready` 保持开放供
   探针使用。
3. **输入限制**：body/图像大小、像素数、图像数量、instruction 长度、
   state 数值有限性、chunk 上限。
4. **不接收 URL/文件路径**：只接受 base64，避免 SSRF/任意文件读取。
5. **模型代码**：`trust_remote_code` 默认 false；只加载可信 checkpoint。
6. **控制安全**：服务端限幅不是急停；ROS 必须保留 `safety_manager_node`、
   速度看门狗、硬件急停。网络失败时底盘应自动回零。

---

## 10. 测试策略

| 层级 | 测试 | 是否需要 GPU |
|---|---|---|
| 配置 | 默认值/合并/非法端口/未知 backend | 否 |
| 图像 | PNG/JPEG、data URI、dict、损坏、超限、round-trip | 否 |
| 动作 | 归一化/物理单位/限幅/取反/死区/hybrid/joint/raw/NaN | 否 |
| 服务 | 图像补齐策略、state 补零/截断、未知相机、chunk 校验 | 否 |
| HTTP | 路由/鉴权/CORS/405/413/422/503、socket 端到端 | 否 |
| 客户端 | health/model/infer/reset、错误码解析 | 否 |
| 真实模型 | 加载/推理/动作 shape/延迟 | 是（手工/ nightly） |

运行：

```bash
cd software/center_server
pip install -r requirements-dev.txt
python3 -m pytest tests -q
```

---

## 11. 与 ROS 的接口契约

ROS 侧应把本服务当作**无状态动作块生成器**：

```text
request:  instruction + images + state
response: actions[T][D] + base_cmd(第 1 步) + arm_cmd(第 1 步)
```

约定：

1. 发原始相机帧（RGB），不要发 224×224 预处理图；
2. `state` 顺序与训练数据一致；
3. 动作块由 ROS 按时间顺序执行，执行期间可并发请求下一块；
4. `base_cmd` 先写 `/cmd_vel_raw`，经 `velocity_mux`/`safety` 后再到
   `/cmd_vel`，不得绕过；
5. HTTP 失败/超时 → 丢弃动作块、停车、回退规则桩；
6. `confidence` 不由本服务伪造；执行门控由 ROS 基于安全检查/动作一致性实现。

现有 `software/ros/center_server/core/vla_engine.py` 仍是规则桩；正式接入
建议新增异步 `smolvla_client_node`，保留规则桩作为降级路径，避免在 ROS
定时器里同步阻塞网络请求。

---

## 12. 后续工作

1. **微调 Rush-MOG 策略**：动作空间 `[vx, vy, wz]`（可选 + 机械臂），
   采集遥操作/规则控制演示数据，输出可用的 SmolVLA checkpoint。
2. **动作块执行器**：ROS 侧按时间戳/速度积分执行 chunk，支持随时抢占/急停。
3. **会话管理**：若策略依赖历史，增加 `session_id` → 状态/队列隔离。
4. **异步推理**：服务端队列 + 最新帧丢弃，避免旧帧浪费 GPU。
5. **可观测性**：Prometheus 指标（请求数/延迟/队列/显存）、结构化日志。
6. **多模型/多租户**：模型注册表、按 `model_id` 路由、GPU 配额。
7. **安全增强**：mTLS、请求签名、审计日志、模型文件校验和。
8. **实车压测**：Wi-Fi 抖动、相机断流、GPU 降频、急停优先级。
