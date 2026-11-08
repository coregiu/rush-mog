# ROS 健康检查接口（healthcheck）

> 版本：1.1（对应 Pi Edge Server 合并后的 `edge_server` 0.2.0）
> 调用方：Edge Server（HTTP）；ROS 内部客户端也可用 ROS 2 服务/话题
> 实现：`ros_nodes/health_monitor_node.py` + `edge_server/core/health_monitor.py`
> 统一接口归档：[`software/api.yaml`](../api.yaml) 的 `Edge Server` 分组

---

## 1. 目标与边界

Edge Server 需要用一个接口回答三个问题：

1. **ROS 本身是否健康**：核心节点是否都在线，话题是否有发布者；
2. **所有连接设备是否健康**：摄像头、雷达、IMU、麦克风、音箱、机械臂、底盘、电池、里程计；
3. **当前是否适合继续执行任务**：是否有必需设备失效、是否急停、是否还在启动宽限期。

本接口只做**只读观测**，不提供恢复/重启/配置写入。恢复动作由 Edge Server 或运维流程决定。

设计取舍：

- **HTTP 优先**：Web/控制面不在 ROS 图内，用 HTTP/JSON 最直接；
- **ROS 原生同时保留**：ROS 节点可用 `/center/health` 话题和 `/center/healthcheck` 服务；
- **设备注册表驱动**：新增设备只改 YAML，不改代码；
- **诚实报告未知**：没有反馈通道的设备（当前是音箱、机械臂）报 `unknown`，不伪造 `ok`；
- **健康检查不能拖垮系统**：HTTP 线程只读预计算快照，端口占用/客户端断开不影响 ROS 节点。

---

## 2. 拓扑与传输

```text
┌──────────────┐   HTTP GET /ros/health     ┌─────────────────────────────┐
│  Edge Server │ ─────────────────────────> │ health_monitor_node (ROS2)  │
│  (管理/编排) │ <───────────────────────── │  :8090  HTTP 网关           │
└──────────────┘   200/503 + JSON           │  /center/health  话题       │
                                            │  /center/healthcheck 服务   │
                                            └──────────────┬──────────────┘
                                                           │ ROS graph / 订阅
                              ┌────────────────────────────┼────────────────────────────┐
                              ▼                            ▼                            ▼
                        /camera/image_raw            /scan, /imu/data            /base/status
                        /edge/video                  /odom                       /battery_state
                        /audio/raw                   /arm/joint_states           /body/audio_status
```

- 代码与 `robot.yaml` 默认监听 `127.0.0.1:8090`；Edge Server 与 ROS 同机，直接走回环代理。
- 跨机监控时才把 `host` 改为 `0.0.0.0`，并设置 `api_key` + 防火墙限制来源。
- Edge Server 的 `dashboard.html` 通过 `GET /api/modules/ros/health`、`/api/modules/ros/params`
  代理本接口；页面只访问 Edge Server 的 80 端口。
- HTTP 快照由节点按 `publish_rate`（默认 1 Hz）预计算；HTTP 线程不调用 rclpy。
- 因此 HTTP 返回的 `age_sec`/`rate_hz` 最多滞后一个发布周期（默认 ≤1 s）。

---

## 3. HTTP 接口

### 3.1 端点

规范路径以 `/ros/*` 为准；同时保留 `/v1/*`、`/health*` 兼容别名，便于和 `edge_server` 的探针习惯保持一致。

| 方法 | 路径 | 说明 | 成功码 |
|---|---|---|---|
| GET | `/ros/health` | 详细健康快照（主接口） | 200 |
| GET | `/v1/health`、`/health`、`/healthcheck` | 详细健康快照（别名） | 200 |
| GET | `/ros/health/devices/{device_id}` | 单设备健康 | 200 |
| GET | `/v1/health/{device_id}`、`/healthcheck/{device_id}` | 单设备健康（别名） | 200 |
| GET | `/ros/params`、`/params` | 健康监控参数描述 | 200 |
| POST | `/ros/params`、`/params` | 修改健康监控参数（运行时） | 200 |
| GET | `/ros/healthz` | 存活探针，进程活着即 200 | 200 |
| GET | `/healthz`、`/livez` | 存活探针（别名） | 200 |
| GET | `/ros/readyz` | 就绪探针；未就绪返回 503 | 200 / 503 |
| GET | `/readyz` | 就绪探针（别名） | 200 / 503 |
| GET | `/` | 服务名、版本、端点列表 | 200 |
| OPTIONS | 任意 | CORS 预检 | 204 |

查询参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `include_metrics=false` | `true` | 不返回设备 `metrics` 字段，减小响应体 |

请求头：

| 头 | 说明 |
|---|---|
| `X-Request-ID` | 可选；不传则服务端生成 `uuid4`，响应头和响应体都会回显 |
| `Authorization: Bearer <key>` | `api_key` 非空时使用 |
| `X-API-Key: <key>` | `api_key` 非空时使用，二选一 |

响应头：`Content-Type: application/json; charset=utf-8`、`Cache-Control: no-store`、`X-Request-ID`、CORS 头。

### 3.2 状态码

| 码 | 场景 |
|---|---|
| 200 | 请求成功；**整体 `status=error` 时 `/ros/health` 仍返回 200**，由 body 判断 |
| 204 | `OPTIONS` |
| 400 | 参数请求体非法/参数校验失败 |
| 401 | 配置了 `api_key` 但缺失/错误 |
| 404 | 未知路径，或 `device_id` 不存在 |
| 405 | 该方法不支持（`/ros/params` 支持 GET/POST） |
| 413 | 参数请求体超过 65536 字节 |
| 503 | `/ros/readyz` 未就绪；或健康/参数快照尚未生成 |

> 关键约定：**`/ros/health` 的 HTTP 200 只代表“健康数据可读”**，不代表机器人健康。
> 机器健康看 body 的 `status` / `healthy` / `ready`；探针看 `/ros/healthz` 和 `/ros/readyz`。

### 3.3 详细健康：`GET /ros/health`

响应顶层字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `ok` | bool | HTTP 请求成功，恒为 `true`（错误响应另有结构） |
| `status` | string | `ok` / `starting` / `degraded` / `error` / `disabled` |
| `healthy` | bool | `status == ok` |
| `ready` | bool | `status in {ok, degraded}` 且未急停；可接新任务 |
| `service` | string | `rush-mog-ros-health` |
| `version` | string | 接口/实现版本，当前 `0.2.0` |
| `uptime_sec` | number | 健康节点启动至今秒数 |
| `timestamp` | string | 快照 UTC 时间，ISO8601，`...Z` |
| `reason` | string | 非 ok 的摘要；ok 时为 `ok`，最长 512 字符 |
| `summary` | object | 设备计数：`total`/`enabled`/`ok`/`degraded`/`error`/`unknown`/`disabled` |
| `emergency_stop` | bool | 安全模块当前是否急停 |
| `safety` | object | 安全状态详情，见下表 |
| `ros` | object | 节点发现/缺失节点信息，见下表 |
| `http` | object | HTTP 网关绑定信息，见下表 |
| `devices` | array | 设备健康列表，顺序与配置一致 |
| `request_id` | string | 请求 ID |

`safety`：

| 字段 | 类型 | 说明 |
|---|---|---|
| `available` | bool | 最近 `safety_timeout_sec`（默认 3 s）内收到 `/center/safety_state` |
| `age_sec` | number | 最后一条安全状态年龄；从未收到为 -1 |
| `emergency_stop` | bool | 急停 |
| `reason` | string | 安全模块原因，如 `manual_emergency_stop,obstacle_too_close(0.20m)` |
| `data_ok` | bool/null | 传感器数据是否在安全模块超时窗口内 |
| `localization_ok` | bool/null | 定位是否满足导航门限 |
| `battery_ok` | bool/null | 电量是否高于低电量阈值 |
| `battery_percent` | number | 电量百分比；未知 -1 |
| `min_obstacle_distance` | number | 最近障碍距离；未知 -1 |

`ros`：

| 字段 | 类型 | 说明 |
|---|---|---|
| `ok` | bool | `missing_nodes` 为空 |
| `checked` | bool | 是否启用了节点发现（`check_nodes`） |
| `node_count` | int | 发现的节点数；未检查为 -1 |
| `expected_nodes` | array | 配置的期望节点 |
| `missing_nodes` | array | 缺失的期望节点 |
| `use_sim_time` | bool | 是否使用 `/clock` |
| `error` | string | 节点发现 API 异常时的原因（可选） |

`http`：

| 字段 | 类型 | 说明 |
|---|---|---|
| `enabled` | bool | 配置是否启用 HTTP |
| `host` | string | 绑定地址 |
| `port` | int | 实际端口（`port: 0` 时为系统分配） |
| `configured_port` | int | 配置端口 |
| `bound` | bool | 是否成功绑定 |
| `error` | string | 绑定失败原因 |
| `auth` | bool | 是否启用 API Key |

设备对象：

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | string | 设备 ID，如 `camera` |
| `type` | string | `camera`/`lidar`/`imu`/`microphone`/`speaker`/`arm`/`base`/`battery`/`odometry`/`other` |
| `status` | string | `ok`/`degraded`/`error`/`unknown`/`disabled` |
| `healthy` | bool | `status == ok` |
| `ready` | bool | `status in {ok, degraded}` |
| `enabled` | bool | 配置是否启用 |
| `required` | bool | 是否必需设备 |
| `connected` | bool | ROS graph 有发布者，或曾收到数据 |
| `data_fresh` | bool | 最后一帧在 `timeout_sec` 内 |
| `topic` | string | 反馈话题 |
| `message_type` | string | ROS 消息类型 |
| `publisher_count` | int | 当前发布者数；查询失败时按 0 处理 |
| `age_sec` | number | 最后一帧年龄；从未收到为 -1 |
| `rate_hz` | number | 最近 `rate_window_sec` 实测频率；样本不足为 -1 |
| `expected_rate_hz` | number | 配置的期望频率 |
| `detail` | string | 人类可读原因；机器判断用 `status` |
| `metrics` | object | 设备指标，如分辨率、电压、关节名；`include_metrics=false` 时省略 |

### 3.4 单设备：`GET /ros/health/devices/{device_id}`

```json
{
  "ok": true,
  "status": "ok",
  "device": { "...": "DeviceHealth 对象" },
  "request_id": "b7..."
}
```

- `status` 是**整体**状态，便于调用方在查单设备时仍知道系统级状态；
- `device_id` 不存在返回 404：

```json
{
  "ok": false,
  "error": {"code": "unknown_device", "message": "未知设备: nope"},
  "request_id": "..."
}
```

### 3.5 存活/就绪

`GET /ros/healthz`（`/healthz`、`/livez`）：

```json
{"ok": true, "status": "ok", "service": "rush-mog-ros-health", "version": "0.2.0",
 "uptime_sec": 12.5, "timestamp": "2026-11-07T15:00:00.000Z", "request_id": "..."}
```

- 只要 HTTP 线程能响应就 200，**不检查设备**；
- 适合 systemd/Docker/livenessProbe。

`GET /ros/readyz`（`/readyz`）：

```json
{"ok": true, "status": "ready", "reason": "ok", "status_detail": "ok", "request_id": "..."}
```

- `ready=true` 返回 200；否则 503：

```json
{"ok": false, "status": "not_ready",
 "reason": "camera: stale 2.4s > 2.0s", "status_detail": "error", "request_id": "..."}
```

- `ready` 的定义：整体 `ok`/`degraded` 且未急停；`starting`/`error`/`disabled` 都不是就绪。

### 3.6 错误响应

```json
{
  "ok": false,
  "error": {"code": "unauthorized", "message": "无效或缺失的 API Key"},
  "request_id": "..."
}
```

错误码：`unauthorized`、`unknown_device`、`not_found`、`method_not_allowed`、`not_ready`。

### 3.7 完整示例（degraded）

```json
{
  "ok": true,
  "status": "degraded",
  "healthy": false,
  "ready": true,
  "service": "rush-mog-ros-health",
  "version": "0.2.0",
  "uptime_sec": 128.4,
  "timestamp": "2026-11-07T15:02:08.123Z",
  "reason": "battery: battery low: 12.0% < 15.0%; emergency_stop: battery_low(12.0%)",
  "summary": {"total": 9, "enabled": 9, "ok": 6, "degraded": 1, "error": 0, "unknown": 2, "disabled": 0},
  "emergency_stop": true,
  "safety": {
    "available": true, "age_sec": 0.08, "emergency_stop": true,
    "reason": "battery_low(12.0%)", "data_ok": true, "localization_ok": true,
    "battery_ok": false, "battery_percent": 12.0, "min_obstacle_distance": 1.2
  },
  "ros": {
    "ok": true, "checked": true, "node_count": 15,
    "expected_nodes": ["bridge_node", "vla_inference_node", "health_monitor_node"],
    "missing_nodes": [], "use_sim_time": false
  },
  "http": {
    "enabled": true, "host": "127.0.0.1", "port": 8090,
    "configured_port": 8090, "bound": true, "error": "", "auth": false
  },
  "devices": [
    {
      "id": "camera", "type": "camera", "status": "ok", "healthy": true,
      "ready": true, "enabled": true, "required": true, "connected": true,
      "data_fresh": true, "topic": "/camera/image_raw",
      "message_type": "sensor_msgs/msg/Image", "publisher_count": 1,
      "age_sec": 0.05, "rate_hz": 14.9, "expected_rate_hz": 15.0,
      "detail": "ok", "metrics": {"width": 640, "height": 480, "encoding": "bgr8", "bytes": 921600}
    },
    {
      "id": "speaker", "type": "speaker", "status": "unknown", "healthy": false,
      "ready": false, "enabled": true, "required": false, "connected": false,
      "data_fresh": false, "topic": "/body/audio_status",
      "message_type": "std_msgs/msg/String", "publisher_count": 0,
      "age_sec": -1.0, "rate_hz": -1.0, "expected_rate_hz": 0.0,
      "detail": "optional device not present", "metrics": {}
    }
  ],
  "request_id": "..."
}
```

### 3.8 参数：`GET/POST /ros/params`

`GET` 返回参数描述列表，dashboard 直接按 `type/group/min/max/step` 渲染表单：

```json
{
  "ok": true,
  "module": "ros",
  "name": "ROS 2 edge_server",
  "version": "0.2.0",
  "editable": true,
  "requires_restart": false,
  "params": [
    {
      "key": "publish_rate",
      "label": "健康发布频率",
      "value": 1.0,
      "type": "float",
      "group": "健康检查",
      "editable": true,
      "requires_restart": false,
      "unit": "Hz",
      "min": 0.2,
      "max": 10.0,
      "step": 0.1
    }
  ],
  "request_id": "..."
}
```

`POST` 支持 JSON 或表单：

```bash
curl -X POST http://127.0.0.1:8090/ros/params \
  -H 'Content-Type: application/json' \
  -d '{"updates":{"publish_rate":2.0,"device.lidar.timeout_sec":2.0}}'
```

响应：

```json
{
  "ok": true,
  "module": "ros",
  "updated": {"publish_rate": 2.0, "device.lidar.timeout_sec": 2.0},
  "requires_restart": false,
  "message": "参数已更新",
  "request_id": "..."
}
```

可写参数：

| 参数 | 类型/范围 | 说明 |
|---|---|---|
| `publish_rate` | float 0.2–10 | 健康话题/HTTP 快照频率；下一周期应用 |
| `startup_grace_sec` | float 0–120 | 启动宽限期 |
| `rate_window_sec` | float 0.5–60 | 频率统计窗口 |
| `strict_unknown` | bool | 可选设备 unknown 是否降级整体 |
| `safety_required` | bool | 安全状态超时是否算故障 |
| `safety_timeout_sec` | float 0.5–60 | 安全状态超时 |
| `check_nodes` | bool | 是否执行节点发现 |
| `device.<id>.expected_rate_hz` | float 0–1000 | 设备期望频率 |
| `device.<id>.timeout_sec` | float 0–60 | 设备数据超时 |
| `device.<id>.min_rate_ratio` | float 0–1 | 设备最低频率比例 |
| `device.<id>.required` | bool | 设备是否必需 |

只读项：`device.<id>.topic`、`device.<id>.message_type`。修改 `topic`/`message_type`
需要改 YAML 并重启节点（涉及重新订阅）。

> 参数是**运行时生效**：重启 `health_monitor_node` 后回到 `config.yaml`/`robot.yaml` 的值。
> 参数修改接口在配置 `api_key` 时与 `/ros/health` 一样需要鉴权。

---

## 4. ROS 2 原生接口

### 4.1 话题 `/center/health`

| 项 | 值 |
|---|---|
| 类型 | `custom_msgs/msg/HealthStatus` |
| 方向 | 发布 |
| 频率 | `health.publish_rate`，默认 1 Hz |
| QoS | RELIABLE + TRANSIENT_LOCAL + depth 1（晚加入也能拿到最后一帧） |

```bash
ros2 topic echo /center/health --once
ros2 topic hz /center/health
```

### 4.2 服务 `/center/healthcheck`

| 项 | 值 |
|---|---|
| 类型 | `custom_msgs/srv/HealthCheck` |
| 请求 | `string[] device_ids`、`bool include_metrics=true` |
| 响应 | `bool success`、`HealthStatus status`、`string message` |

```bash
# 全部设备
ros2 service call /center/healthcheck custom_msgs/srv/HealthCheck "{}"

# 只看相机/雷达，不返回 metrics
ros2 service call /center/healthcheck custom_msgs/srv/HealthCheck \
  "{device_ids: [camera, lidar], include_metrics: false}"
```

约定：

- `success=true` 只表示请求被正常处理；整体健康看 `status.status`；
- `device_ids` 为空 = 全部；未知设备返回 `success=false`、`message="未知设备: ..."`；
- `include_metrics=false` 时设备 `metrics_json` 为空字符串；
- 过滤设备时，`status.status` 仍是**全局**状态，避免“只看相机 ok 就误判整车 ok”。

### 4.3 消息字段

`HealthStatus`（`custom_msgs/msg/HealthStatus.msg`）：

```text
std_msgs/Header header
string status              # ok / starting / degraded / error / disabled
bool healthy
bool ready
float64 uptime_sec
string service
string version
string reason
bool emergency_stop
string safety_reason
bool safety_available
uint32 device_count / ok_count / degraded_count / error_count / unknown_count / disabled_count
DeviceHealth[] devices
string ros_json
```

`DeviceHealth`（`custom_msgs/msg/DeviceHealth.msg`）：

```text
std_msgs/Header header
string id / type / status
bool healthy / ready / enabled / required / connected / data_fresh
string topic / message_type
uint32 publisher_count
float64 age_sec / rate_hz / expected_rate_hz
string detail
string metrics_json
```

---

## 5. 状态语义

### 5.1 设备状态判定

按以下顺序判定（`T` 为设备 `timeout_sec`）：

| 条件 | 设备 `status` |
|---|---|
| `enabled=false` | `disabled` |
| 配置错误（topic/message_type/订阅失败） | `error` |
| 从未收到数据，且仍在启动宽限期 | `unknown`（整体 `starting`） |
| 从未收到数据，超出宽限期，`required=true` | `error` |
| 从未收到数据，超出宽限期，`required=false` | `unknown` |
| 最后数据年龄 > `T`，`required=true` | `error` |
| 最后数据年龄 > `T`，`required=false` | `degraded` |
| 数据非法（空图像/空雷达/臂关节缺失/电池低等） | `required` ? `error` : `degraded` |
| 实测频率 < `expected_rate_hz * min_rate_ratio` | `degraded` |
| 其他 | `ok` |

补充：

- 频率至少需要 2 帧才能计算，样本不足时 `rate_hz=-1`，不因频率扣分；
- `connected` = ROS graph 有发布者 **或** 曾收到数据；它只作诊断，不作为唯一健康依据；
- `data_fresh` 是年龄判定结果；`age_sec=-1` 表示从未收到；
- 设备 `detail` 为英文机器可读摘要，如 `stale 2.4s > 2.0s`、`low rate 2.0Hz < 5.0Hz`。

### 5.2 整体状态判定

| 条件（按优先级） | 整体 `status` |
|---|---|
| 任一 `required` 设备 `error`；或超出宽限期后仍有期望节点缺失；或 `safety_required=true` 且安全状态超时 | `error` |
| 启动宽限期内，仍有必需设备/期望节点/安全状态未就绪 | `starting` |
| 任一设备 `degraded`；或可选设备 `error`；或急停；或 `strict_unknown=true` 且存在 `unknown` | `degraded` |
| 其他 | `ok` |
| 健康监控被配置关闭 | `disabled` |

- `healthy = (status == ok)`；
- `ready = status in {ok, degraded} 且 emergency_stop == false`；
- `reason` 汇总所有 `error`/`degraded` 设备、缺失节点、急停原因，最长 512 字符；
- 过滤设备（`device_ids`）不改变整体状态。

### 5.3 启动宽限期

- `health.startup_grace_sec` 默认 5 s（实车配置 8 s）；
- 宽限期内，缺失的必需设备/节点只让整体进入 `starting`，不立即报 `error`；
- 宽限期后仍缺失才报 `error`；
- 宽限期内如果设备已就绪，整体可直接 `ok`/`degraded`。

### 5.4 安全与急停

- 订阅 `/center/safety_state`（TRANSIENT_LOCAL），超时 `safety_timeout_sec` 默认 3 s；
- `safety_required=true` 且安全状态超时：宽限期内 `starting`，之后 `error`；
- `emergency_stop=true`：整体 `degraded`，`ready=false`，`reason` 带 `emergency_stop: <原因>`；
- 急停是“安全状态”，不是设备故障；设备本身仍可 `ok`。

### 5.5 `strict_unknown`

- 默认 `false`：可选设备无反馈（`unknown`）不影响整体 `ok`，适合当前音箱/机械臂尚未接反馈的阶段；
- 设为 `true`：任何 `unknown` 都让整体 `degraded`，适合“所有设备必须有反馈”的验收环境。

---

## 6. 设备清单与数据源

### 6.1 本车默认设备（`robot.yaml`）

| id | type | 反馈话题 | 必需 | 期望频率 | 说明 |
|---|---|---|---|---|---|
| `camera` | camera | `/camera/image_raw` | 是 | 15 Hz | 空图/宽高为 0 判非法 |
| `lidar` | lidar | `/scan` | 是 | 10 Hz | 空 ranges 判非法 |
| `imu` | imu | `/imu/data` | 是 | 100 Hz | 全零/非有限值判非法 |
| `microphone` | microphone | `/audio/raw` | 否 | 10 Hz | 空音频帧判非法 |
| `speaker` | speaker | `/body/audio_status` | 否 | 0 | **当前无反馈，unknown** |
| `arm` | arm | `/arm/joint_states` | 否 | 10 Hz | **当前无反馈，unknown**；接入后校验 j1..j5 |
| `base` | base | `/base/status` | 是 | 1 Hz | JSON；`encoder_ok=false` 判非法 |
| `battery` | battery | `/battery_state` | 否 | 1 Hz | 低于 `battery_low_percent` 判非法 |
| `odom` | odometry | `/odom` | 是 | 20 Hz | 非有限位姿判非法 |

仿真默认（`config.yaml`）使用 `/edge/video`、`/edge/radar`、`/edge/audio`、`/robot/odometry`、`/robot/joint_states`，`base`/`battery` 关闭。详见配置文件注释。

### 6.2 反馈缺口与建议协议

#### 音箱 `/body/audio_status`（待接入）

建议由底盘桥/音频控制节点以 1 Hz 发布 `std_msgs/msg/String`：

```json
{"online": true, "busy": false, "volume": 20, "track": 3, "error": ""}
```

- `online=false`：健康检查判设备 `degraded`（可选设备）；
- 无发布者：`unknown`，不影响整体 `ok`；
- 若后续有 JQ6500 状态回读，把 `online` 映射为真实状态即可。

#### 机械臂 `/arm/joint_states`（待接入）

建议机械臂驱动/回读节点发布 `sensor_msgs/msg/JointState`，`name` 至少包含配置里的 `joint_names`：

```yaml
arm:
  topic: "/arm/joint_states"
  message_type: "sensor_msgs/msg/JointState"
  joint_names: ["j1", "j2", "j3", "j4", "j5"]
```

- 缺少任一关节：`degraded`（可选设备）；
- 无发布者：`unknown`；
- 若驱动只发布 `/joint_states`，可把配置 topic 改为 `/joint_states`，但要注意轮子关节也会混入，`joint_names` 校验能区分。

#### 底盘 `/base/status`（已实现）

`edge_server_node` 在收到 STM32 `$B,...` 遥测后发布 JSON：

```json
{"source": "pi", "seq": 12, "age_ms": 80, "encoder_ok": true,
 "estop": false, "distance_mm": 1520, "wheel_mm_s": [100, -200, 300, -400],
 "uptime_ms": 123456, "serial_connected": true}
```

健康检查会：

- `encoder_ok=false` → 设备非法（`required` 时 `error`）；
- `estop`、`wheel_mm_s`、`distance_mm` 等作为 metrics 暴露，不重复判故障；
- 电池低阈值来自 `safety.battery_low_percent`（默认 15%）。

### 6.3 新增设备

只改配置，无需改节点代码：

```yaml
health:
  devices:
    temperature:
      type: other
      topic: "/sensor/temperature"
      message_type: "sensor_msgs/msg/Temperature"
      expected_rate_hz: 1.0
      timeout_sec: 5.0
      required: false
```

- `message_type` 支持任意已安装的 ROS 消息类型（`rosidl_runtime_py.get_message` 动态加载）；
- 未知消息类型只做“有数据/频率/超时”判定，`metrics` 里给 `message_class`；
- 需要设备专属校验（如关节名、JSON 字段）时，在 `health_monitor_node._extract_metrics` 增加分支。

---

## 7. Edge Server 接入建议

### 7.1 轮询策略

| 用途 | 端点 | 建议周期 |
|---|---|---|
| 存活 | `/ros/healthz` | 5–10 s |
| 就绪门控 | `/ros/readyz` | 2–5 s（启动/任务前） |
| 设备健康看板 | `/ros/health` | 1–5 s |
| 故障定位 | `/ros/health/devices/{id}` | 告警时按需 |

- 请求超时建议 2 s；连续失败按指数退避（1→2→4→8 s，上限 30 s）；
- 用 `X-Request-ID` 关联日志；
- 不要每秒打全量日志，只记录状态变化或 `error`。

### 7.2 状态映射建议

| 健康状态 | Edge Server 动作 |
|---|---|
| `ok` | 正常接单/下发任务 |
| `starting` | 等待，不判定故障；启动宽限期后再升级 |
| `degraded` | 允许降级任务；记录告警；按设备禁用能力 |
| `error` | 阻止新任务；正在执行的任务按安全策略暂停/取消 |
| `disabled` | 健康监控未启用，按“不可观测”处理，不要当作健康 |
| HTTP 连不上 | ROS/主机可能宕机；按通信故障处理，触发底盘安全策略 |

设备级建议：

- `camera` error → 禁用目标搜索/VLA 视觉，保留导航；
- `lidar` error → 禁止自主导航/避障，立即降速/停车；
- `imu` error → 定位降级，禁止依赖航向的长任务；
- `microphone` error → 禁用语音入口；
- `speaker` unknown → 不承诺语音播报，改用页面提示；
- `arm` unknown/error → 禁用抓取/放置；
- `battery` degraded → 回充/结束任务；
- `base`/`odom` error → 停止运动，检查串口/STM32；
- `ros.missing_nodes` 非空 → 核心链路不完整，按 `error` 处理。

### 7.3 Python 轮询示例

```python
import json
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8090"
API_KEY = ""  # 与 health.http.api_key 一致；为空表示不鉴权


def fetch(path: str, timeout: float = 2.0) -> tuple[int, dict]:
    headers = {"X-Request-ID": f"edge-center-{int(time.time() * 1000)}"}
    if API_KEY:
        headers["Authorization"] = f"Bearer {API_KEY}"
    request = urllib.request.Request(BASE + path, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())
    except (urllib.error.URLError, TimeoutError) as exc:
        return 0, {"ok": False, "error": {"code": "unreachable", "message": str(exc)}}


def poll_loop() -> None:
    backoff = 1.0
    while True:
        status_code, health = fetch("/ros/health")
        if status_code == 0:
            # ROS/主机不可达：按通信故障处理
            print("health unreachable:", health)
            time.sleep(backoff)
            backoff = min(30.0, backoff * 2)
            continue
        backoff = 1.0
        state = health.get("status")
        if state == "error":
            print("ROS unhealthy:", health.get("reason"))
            for device in health.get("devices", []):
                if device["status"] in {"error", "degraded"}:
                    print("  ", device["id"], device["status"], device["detail"])
        elif state == "degraded":
            print("ROS degraded:", health.get("reason"))
        elif state == "starting":
            print("ROS starting: wait before dispatching tasks")
        time.sleep(2.0)
```

### 7.4 curl 示例

```bash
curl -s http://127.0.0.1:8090/ros/health | python3 -m json.tool
curl -s http://127.0.0.1:8090/ros/healthz
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8090/ros/readyz
curl -s http://127.0.0.1:8090/ros/health/devices/lidar | python3 -m json.tool
curl -s 'http://127.0.0.1:8090/ros/health?include_metrics=false' | python3 -m json.tool
# 启用 api_key 后：
curl -s -H 'Authorization: Bearer <key>' http://127.0.0.1:8090/ros/health
```

---

## 8. 配置参考

```yaml
edge_server:
  health:
    enabled: true                 # false: 节点不订阅设备，整体报 disabled
    publish_rate: 1.0             # 话题/HTTP 快照频率 Hz
    startup_grace_sec: 8.0        # 启动宽限期
    rate_window_sec: 5.0          # 频率统计窗口
    min_rate_ratio: 0.5           # 低于 expected * ratio 判 degraded
    strict_unknown: false         # true: unknown 也降级整体
    safety_required: true         # 安全状态超时是否算故障
    safety_timeout_sec: 3.0
    check_nodes: true             # 是否做节点发现
    expected_nodes: [...]         # 缺失即 error（宽限期后）
    topic: "/center/health"
    service: "/center/healthcheck"
    http:
      enabled: true
      host: "127.0.0.1"           # 同机 Edge Server 代理；跨机监控再改 0.0.0.0 + api_key
      port: 8090
      api_key: ""                 # 非空启用 Bearer/X-API-Key
      cors_allow_origin: "*"
    device_defaults:              # 可选：所有设备的默认值
      timeout_sec: 2.0
      min_rate_ratio: 0.5
    devices:
      <id>:
        type: camera|lidar|imu|microphone|speaker|arm|base|battery|odometry|other
        enabled: true
        required: true
        topic: "/..."
        message_type: "pkg/msg/Type"
        expected_rate_hz: 10.0
        timeout_sec: 2.0
        min_rate_ratio: 0.5
        qos: sensor|reliable|transient_local
        joint_names: ["j1", "j2"]  # 仅 arm 等需要关节校验时
        description: "..."          # 可选；设备从未上报时作为 unknown 的说明
```

说明：

- `expected_rate_hz: 0` 表示不检查频率，只检查超时；
- `timeout_sec: 0` 表示不检查超时（不建议）；
- `qos: sensor` 用 `SensorDataQoS`（best effort），兼容真实驱动；`reliable` 用 depth 10；`transient_local` 用 depth 1 + RELIABLE + TRANSIENT_LOCAL；
- 设备配置顺序就是 HTTP 返回 `devices` 的顺序。

---

## 9. 安全

- 代码与 `robot.yaml` 默认只监听 `127.0.0.1`，由同机 Edge Server 代理，无需鉴权；
- 跨机部署必须：
  1. `host: "0.0.0.0"`；
  2. 设置 `api_key`（`Authorization: Bearer` 或 `X-API-Key`）；
  3. 防火墙限制来源网段；
  4. 不要把 8090 直接暴露到公网。
- 本接口只读，不提供写入/重启；但仍会暴露节点名、设备拓扑、电量等信息，生产环境应保护；
- 未实现 TLS；需要时用 Nginx/Caddy/VPN 反向代理；
- `/ros/healthz`、`/ros/readyz` 在启用 `api_key` 时仍保持匿名，便于探针；敏感信息在 `/ros/health`。

---

## 10. 测试与验收

### 10.1 单元测试

```bash
cd software/edge_server
PYTHONPATH=.:$PYTHONPATH python3 -m pytest tests/test_health_monitor.py tests/test_health_http.py -q
# 29 passed
PYTHONPATH=.:$PYTHONPATH python3 -m pytest tests -q
# 94 passed（含原有 69 项）
```

覆盖：

- 正常/超时/低频率/非法数据/配置错误/禁用设备；
- 启动宽限期；
- 缺失期望节点、安全状态超时；
- 急停、`strict_unknown`、设备过滤、JSON/metrics 开关；
- HTTP 存活/就绪/鉴权/404/405/单设备/metrics 开关/无快照。

### 10.2 ROS 联调

```bash
# 终端 A
ros2 launch edge_server edge_server.launch.py

# 终端 B
python3 tests/fake_edge_publisher.py

# 终端 C
curl -s http://127.0.0.1:8090/ros/health | python3 -m json.tool
ros2 topic echo /center/health --once
ros2 service call /center/healthcheck custom_msgs/srv/HealthCheck "{}"
```

实车验收：

1. 拔掉相机，5 s 内 `camera.status=error`，整体 `error`；
2. 拔掉雷达，`lidar.status=error`，导航应被上层禁用；
3. 拔掉 IMU，`imu.status=error`；
4. 停掉 `vla_inference_node`，`ros.missing_nodes` 包含它，整体 `error`；
5. 触发急停，`emergency_stop=true`、`ready=false`、`status=degraded`；
6. 恢复设备后，下一周期自动回到 `ok`（无需重启健康节点）；
7. Edge Server 断网/重启后，能重新拿到 `TRANSIENT_LOCAL` 最后一帧和 HTTP 快照。

---

## 11. 版本与兼容

- 当前接口版本 `0.2.0`，与 `edge_server` 包版本一致；
- 新增字段保持向后兼容；客户端应忽略未知字段；
- `status` 枚举稳定：`ok`/`starting`/`degraded`/`error`/`unknown`（设备）/`disabled`；
- 破坏性变更（改字段含义/删字段/改路径）升 minor 或 major，并在 `README.md`/`api.yaml` 同步；
- `custom_msgs` 增加 `DeviceHealth`、`HealthStatus`、`HealthCheck` 属于新增，不改变现有消息。

---

## 12. FAQ

**Q：为什么 `ok=true` 但 `healthy=false`？**
`ok` 是 HTTP 请求成功；`healthy` 是机器人健康。请用 `status`/`healthy`/`ready`。

**Q：`/ros/healthz` 200、`/ros/readyz` 503，正常吗？**
正常。前者只表示进程活着；后者表示系统还没就绪（`starting`/`error`/急停）。

**Q：音箱/机械臂为什么是 `unknown`？**
当前 ROS 里没有它们的反馈话题（JQ6500 经 STM32 串口控制、机械臂无回读）。健康接口不伪造 `ok`；接入 `/body/audio_status`、`/arm/joint_states` 后自动变成 `ok`/`degraded`/`error`。

**Q：端口 8090 被占用怎么办？**
节点会记录 `health HTTP bind failed`，ROS 话题/服务仍可用，`http.bound=false`；改 `health.http.port` 或释放端口后重启。

**Q：为什么设备 `connected=false` 但 `status=ok`？**
`connected` 是 ROS graph 发布者数量（可能查询失败/时序问题）；只要 `data_fresh=true`，说明数据确实在流，仍判 `ok`。

**Q：可以只监控一部分设备吗？**
可以。`health.devices` 只列出需要监控的设备；未列出的设备不会出现在 `devices`，也不参与整体判定。`device_ids` 只过滤返回列表，整体状态仍是全局。

**Q：健康节点自己挂了怎么办？**
Edge Server 的 HTTP 请求会失败；同时 Edge Server 应把“探针连续失败”当作 ROS/主机故障。健康节点是核心节点之一，启动脚本应守护它。
