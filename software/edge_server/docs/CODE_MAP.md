# 代码结构与控制链路导读

> 面向二次开发：想知道“点一下网页按钮，代码到底走了哪些地方”。
> 配套：[OPERATIONS.md](OPERATIONS.md)（启停/排障）、[DESIGN.md](DESIGN.md)（设计细节）、
> [HEALTHCHECK.md](HEALTHCHECK.md)（健康检查契约）。
> 代码基线：2026-11-14，`software/edge_server` + `software/body_control`。

---

## 1. 一张图看懂

```text
浏览器 / 手机
  │  POST /cmd  "button:up:down"
  ▼
edge_server_node（Pi，:80）
  ├─ edge_web.py         HTTP 路由 / 静态页 / 模块代理
  ├─ edge_server_node.py 节点主体：串口、/cmd_vel、/odom、/status
  └─ edge_control.py     纯逻辑：按钮映射、运动状态、遥测解析
        │  write("1\n")
        ▼
SerialLink（线程）── /dev/ttyAMA1 @115200 ── GPIO4(TXD3)
                                             │
                                             ▼
                                   STM32 USART3 PB11(RX)
                                     USART3_IRQHandler
                                             │  '\n' 分帧
                                             ▼
                                     command_queue（FreeRTOS）
                                             │
                                     exe_task_from_queue
                                             │
                                     execute_command
                                       ├─ LED = !LED（PC13）
                                       ├─ 命令 → 模块
                                       └─ motor_direct_executor / motor_pwm_executor

反方向（遥测）：
STM32 编码器 → encoder_telemetry_task（50ms）
  → "$B,seq,uptime,fl,fr,rl,rr,distance\n" → USART3 PB10(TX)
  → Pi GPIO5(RXD3) /dev/ttyAMA1
  → SerialLink._loop → parse_body_line → /odom、/joint_states、/base/status
```

> 本机接线：Pi GPIO4 → STM32 PB11，Pi GPIO5 → STM32 PB10，**必须共地**。
> Pi 侧设备是 `/dev/ttyAMA1`（`dtoverlay=uart3`），不是 `/dev/ttyUSB0`。

---

## 2. 目录结构

```text
software/
├── edge_server/                 ROS2 包：Pi 端全部应用
│   ├── edge_server/             Python 库（不依赖 ROS 的纯逻辑 + HTTP）
│   │   ├── edge_control.py      命令映射/MotionState/遥测解析（重点）
│   │   ├── edge_web.py          HTTP 服务器/路由/静态文件/模块代理
│   │   ├── http_health.py       健康接口数据组装
│   │   ├── core/                业务核心：body_drive、odometry、velocity_mux、
│   │   │                        safety_checker、vla_engine、navigation、object_search…
│   │   ├── drivers/             硬件驱动（ICM-20948）
│   │   ├── utils/               配置、时钟、日志
│   │   └── config/              robot.yaml（实车）、config.yaml（仿真）
│   ├── ros_nodes/               ROS2 节点入口（setup.py 的 console_scripts）
│   │   ├── edge_server_node.py  核心：网页+串口+ROS 桥（重点）
│   │   ├── health_monitor_node.py 健康检查 + 8090 HTTP
│   │   ├── base_bridge_node.py  旧串口桥（legacy）
│   │   ├── esp32_bridge_node.py 旧 ESP32 HTTP 桥（legacy）
│   │   └── vla_/navigation_/object_search_…  全栈节点
│   ├── launch/                  edge_server / robot_bringup / slam / nav2
│   ├── web/                     index.html（手柄）、dashboard、JS/CSS
│   ├── tests/                   pytest / 端到端脚本
│   └── setup.py                 entry_points + launch/config/web 安装清单
├── custom_msgs/                 ROS2 自定义消息/服务
├── body_control/                STM32F103 固件（FreeRTOS）
│   └── src/
│       ├── src/main.c           入口：时钟→协议→模块→调度器
│       ├── src/controller.c     命令表、execute_command、初始化顺序
│       ├── src/ps2_receiver.c   USART3 初始化 + 接收中断（重点）
│       ├── src/task_manager.c   命令队列消费任务
│       ├── src/encoder_sensor.c 编码器 EXTI + $B 遥测任务
│       ├── src/motor_direct_executor.c  8 路方向/使能
│       ├── src/motor_pwm_executor.c     PWM 档位/微调
│       ├── src/cliff_sensor.c   悬崖保护（PB3/PB4）
│       └── inc/command.h        协议常量、波特率、命令枚举
├── center_server/               PC 侧上位机（与 Pi 独立）
└── tools/                       联调脚本
```

---

## 3. 进程与节点

### 3.1 当前部署（systemd）

`edge_server.service` 只跑两个节点：

| 进程 | 入口 | 作用 | 端口 |
|---|---|---|---|
| `edge_server_node` | `ros_nodes/edge_server_node.py:main` | 网页/API、串口、`/cmd_vel`→STM32、STM32→`/odom` | 80、8080 |
| `health_monitor_node` | `ros_nodes/health_monitor_node.py:main` | 话题/设备健康检查 | 127.0.0.1:8090 |

### 3.2 全栈（可选）

`robot_bringup.launch.py` 会再拉起 `edge_server.launch.py` 里的 VLA/感知/导航/定位/
速度仲裁/目标搜索等节点。**它会自己再起一个 `edge_server_node`**，所以和 systemd
服务二选一，不能同时跑（80 端口冲突）。

| launch | 内容 |
|---|---|
| `edge_server.launch.py` | 全量节点，参数裁剪用 `start_*=true/false` |
| `robot_bringup.launch.py` | 实车：全量 + `base_transport:=edge_serial` |
| `slam_stack.launch.py` | 只包 `slam_toolbox`，不占 80 端口 |
| `nav2_stack.launch.py` | 只包 Nav2 navigation + localization |

---

## 4. 一次按钮点击的完整调用链（核心）

以按住“↑”为例，消息是 `button:up:down`：

| 步骤 | 位置 | 做什么 |
|---|---|---|
| 1 | `web/index.html` `pressButton()` → `sendButtonCommand()` → `sendRawCmd()` | `fetch('/cmd', {method:'POST', body:'button:up:down'})`，纯文本 |
| 2 | `edge_server/edge_web.py` `do_POST()`（约 513 行） | 识别 `/cmd`，`app.control(text)` |
| 3 | `EdgeHttpApp.control()`（约 355 行） | 长度校验，调 `runtime.handle_command(text)` |
| 4 | `ros_nodes/edge_server_node.py` `handle_command()`（约 909 行） | 急停检查，`dispatch_control_message()` |
| 5 | `edge_server/edge_control.py` `dispatch_control_message()`（308 行） | 拆 `button:key:event` |
| 6 | `edge_control.button_command()`（147 行） | `up → "1"`；松开 `up → "0"` |
| 7 | `MotionState.apply_manual()`（约 229 行） | 记录持续运动状态；松开时清状态并发 `"0"` |
| 8 | `EdgeServerNode._send_command()`（487 行） | 0.15s 内相同命令去重；`force=True` 时跳过 |
| 9 | `SerialLink.write()`（136 行） | `serial.write("1\n")`；端口未打开直接返回 False |
| 10 | STM32 `ps2_receiver.c` `USART3_IRQHandler()`（104 行） | 收字节，遇 `\n`/`\r` 组帧，`send_to_queue_isr()` |
| 11 | `task_manager.c` `exe_task_from_queue()` | 从队列取命令 |
| 12 | `controller.c` `execute_command()`（86 行） | **`LED = !LED`**，查命令表，调对应执行器 |

关键点：

- 命令是 **ASCII + 换行**，不是二进制帧；`SerialLink.write()` 负责补 `\n`；
- 网页按钮/摇杆**按下时只发一次**，松开/停止时再发一次；STM32 会锁存运动状态，
  按住期间不需要重发。`edge.motion.resend_manual=true` 可恢复旧的周期性重发；
- 松开方向键发 `"0"`；`square/circle` 松开发 `"I"`（直行）；页面隐藏用
  `sendBeacon('/cmd', 'button:stop:down')` 兜底；
- `handle_command()` 只在急停时返回空；**串口没打开也会返回命令**（HTTP 仍回 `ok`），
  所以“接口 200”不等于“STM32 收到了”；
- 每条命令、串口连接/断开、急停、租约超时都会写入节点内的 120 条环形缓冲
  （`_record_event`），dashboard 通过 `/api/events` 展示；服务重启后清空。

### 4.1 按钮 → 命令字

| 网页 id | 按下 | 松开 | 说明 |
|---|---|---|---|
| `up` | `1` | `0` | 前进 |
| `down` | `2` | `0` | 后退 |
| `left` | `3` | `0` | 左横移 |
| `right` | `4` | `0` | 右横移 |
| `l1` / `r1` | `9` / `A` | `0` | 左/右原地转 |
| `l2` / `r2` | `B` / `C` | `0` | 左/右大转弯 |
| `triangle` | `E` | 不发 | 加速档 |
| `cross` | `F` | 不发 | 减速档 |
| `square` | `G` | `I` | 左微调，松开回直行 |
| `circle` | `H` | `I` | 右微调，松开回直行 |
| `start` | `D` | 不发 | 测试序列 |
| `select` | `J` | 不发 | 复位 |
| `stop` | `0` | — | 停车 |
| 左摇杆 | `1..8` + 档位数字 | `0` | 如 `stick:left-analog:uple:5` → `"55"` |

---

## 5. 遥测 / 里程计链

| 步骤 | 位置 | 说明 |
|---|---|---|
| 1 | `encoder_sensor.c` `encoder_telemetry_task()` | 每 50ms（20Hz）组一帧 |
| 2 | `encoder_uart_write()` | USART3 PB10 阻塞发送，115200 约 4~5ms |
| 3 | `SerialLink._loop()` | `readline()` 收行，UTF-8 解码 |
| 4 | `_on_serial_line()` | `parse_body_line()` 校验 |
| 5 | `_update_odom()` | 四轮 mm/s → `WheelOdometry` |
| 6 | `on_odom_timer()` / `on_status_timer()` | 发布 `/odom`、`/joint_states`、`/base/status` |

帧格式（8 段，固定）：

```text
$B,<seq>,<uptime_ms>,<fl_mm_s>,<fr_mm_s>,<rl_mm_s>,<rr_mm_s>,<distance_mm>\n
```

解析失败（段数不对、数字非法、速度越界 ±5000）会整帧丢弃；`/status` 里
`encoder_ok=true` 表示最近 2 秒收到过合法帧。

里程计几何在 `edge_server/core/odometry.py`，参数来自 `robot.yaml`：

- `wheel_radius`、`wheelbase_x`、`track_y`、`encoder_cpr`；
- `sign_fl/fr/rl/rr` 修正轮子安装方向；
- `publish_tf` 默认 false（由 EKF 发 `odom→base_link`）。

---

## 6. ROS 接口

| 方向 | 话题/服务 | 说明 |
|---|---|---|
| 订阅 | `/cmd_vel`（Twist） | `body_drive.twist_to_body_command()` 量化成方向+PWM |
| 订阅 | `/center/safety_state` | 安全状态，急停时丢弃运动命令 |
| 发布 | `/odom`、`/joint_states`、`/base/status` | 来自 STM32 遥测 |
| 发布 | `/center/health` | 健康状态（health_monitor_node） |
| 服务 | `/center/healthcheck` | 同上，请求/响应式 |

`/cmd_vel` 与网页共用同一个串口出口，**网页手动控制优先**：手动命令未松开前，
ROS 的 `raw:` 命令不会抢占（`MotionState.apply_raw(respect_manual=True)`）。

Twist 量化规则（`core/body_drive.py`）：

- 平移按方向分 8 个扇区（1/5/3/7/2/8/4/6），按速度大小取 1~9 档；
- 原地转向 `wz>0 → 9`，`wz<0 → A`；
- 同时有平移和转向：默认 `priority` 取归一化后更大的一项；
  `time_share` 按 `arc_period_sec` 交替（近似弧线，抖动更大）。

---

## 7. HTTP / Web 层

| 方法 | 路径 | 作用 |
|---|---|---|
| POST | `/cmd` | 纯文本 `button:...` / `stick:...` / `raw:...` |
| POST | `/cmd-button` | 表单 `key`/`type`（兼容旧页面） |
| POST | `/cmd-stick` | 表单 `stickId`/`direct`/`stepValue` |
| GET/POST | `/stop` | 无条件停车 |
| GET | `/status` | 运行状态（含 `motion` 当前指令） |
| GET | `/api/body` | 遥测详情 + `motion` 当前指令/来源/持续时长 |
| GET | `/api/events?limit=N` | 最近 N 条运行事件（命令/串口/急停/超时） |
| GET | `/api/modules` | 模块注册表 |
| GET | `/api/modules/edge` | Edge 模块详情 |
| GET | `/api/modules/edge/params` | 可调参数 |
| POST | `/api/modules/edge/params` | 改参数（keepalive/fps） |
| GET | `/api/modules/ros/health` | 代理 8090 健康接口 |
| GET | `/stream` | 302 跳到 `:8080/stream` |
| GET | 其他 | `web/` 静态文件 |

前端文件：

| 文件 | 作用 |
|---|---|
| `web/index.html` | 手柄页；按钮/摇杆事件、`/cmd` 发送、stop beacon |
| `web/ps2.js` / `ps2.css` | 手柄视觉与交互 |
| `web/dashboard.html` / `dashboard.js` | 数字孪生：EDGE + ROS 2 合并节点、hardware/README BOM 的 11 个硬件模块矩阵 |
| `web/audio.js` | 音频控制（与运动链路独立） |
| `web/yolo.js` | 视觉检测展示 |

注意：`/cmd` 只接受**纯文本**，不要发 JSON；页面改动后要 `colcon build` +
`systemctl restart`，浏览器再 Ctrl-F5。

---

## 8. STM32 固件结构

### 8.1 启动顺序（`main.c` → `controller.c`）

```text
clock_init / delay_init
init_protocols()       → LED(PC13) 初始化
init_modules()
  ├─ audio_receiver.init()       USART2（JQ6500，音频已停用）
  ├─ ps2_receiver.init()         USART3 115200 + RXNE 中断   ← Pi 链路
  ├─ motor_direct_executor.init()
  ├─ init_freertos()             命令队列 + 消费任务
  ├─ motor_pwm_executor.init()   TIM3 PWM
  ├─ encoder_sensor_init()       PA0~PA7 EXTI + 遥测任务
  └─ cliff_sensor_init()         PB3/PB4，关闭 JTAG 保留 SWD
LED = !LED
vTaskStartScheduler()
```

### 8.2 任务/中断

| 名称 | 位置 | 优先级 | 作用 |
|---|---|---|---|
| `USART3_IRQHandler` | `ps2_receiver.c` | 3 | 收 Pi 命令，按 `\n` 分帧入队 |
| `CONSUMER_TASK` | `task_manager.c` | 2 | 出队并 `execute_command()` |
| `ENC_TLM` | `encoder_sensor.c` | 1 | 50ms 发 `$B` 遥测 |
| 悬崖任务 | `cliff_sensor.c` | — | 10ms 采样，触发即封运动 |

### 8.3 命令表（`inc/command.h`）

| 命令 | 含义 | 模块 |
|---|---|---|
| `0` | 停 | MOTOR_DIRECT |
| `1`~`8` | 前/后/左/右/四斜向 | MOTOR_DIRECT |
| `9` / `A` | 左转 / 右转 | MOTOR_DIRECT |
| `B` / `C` | 左/右大转弯 | MOTOR_DIRECT |
| `D` | 测试序列 | MOTOR_DIRECT |
| `E` / `F` | 加速 / 减速 | MOTOR_PWM |
| `G` / `H` | 左/右微调 | MOTOR_PWM |
| `I` | 直行（微调回正） | MOTOR_PWM |
| `J` | 复位（清队列、停电机） | RESET |
| `K`~`P` | 音频（当前固件未启用） | AUDIO |

第二位可选 `0`~`9` 表示 PWM 档位，例如 `15` = 前进 5 档。
**命令必须以 `\n` 或 `\r\n` 结尾**，否则会一直留在接收缓冲里。

### 8.4 LED 的含义

`execute_command()` 第一行就是 `LED = !LED`（PC13）：

- 每成功出队一条命令，LED 翻转一次；
- 上电初始化后 LED 先亮再灭，之后只由命令驱动；
- **LED 不闪 = 命令没到 `execute_command()`**，问题在 USART3 接收/队列，不在电机。

> 验证技巧：网页“按一下”会发两条命令（按下 `1`、松开 `0`），LED 翻转两次。
> 一条命令对应一次翻转；按住期间不再重发，所以不会连闪（旧版本才会 5Hz 连闪）。
> 要连续观察就发奇数条停车命令：

```bash
for i in $(seq 1 9); do
  curl -s -X POST -H 'Content-Type: text/plain' \
    --data 'button:stop:down' http://192.168.3.26/cmd >/dev/null
  sleep 0.5
done
```

> 9 条命令应在 4~5 秒内看到 LED 翻转 9 次。

固件里留了三个可直接用 ST-Link watch 的计数器（`ps2_receiver.c`）：

| 变量 | 含义 |
|---|---|
| `g_ps2_rx_bytes` | 收到的字节数；不涨 = 接线/波特率/引脚复用问题 |
| `g_ps2_rx_frames` | 完整帧数；bytes 涨而它不涨 = 没收到换行 |
| `g_ps2_rx_last_byte` | 最后一个字节，正常应是 `0x0A`/`0x0D` 或命令字符 |

---

## 9. 配置系统

优先级（`edge_server/utils/config.py`）：

```text
ROS 参数 config_file  >  环境变量 EDGE_SERVER_CONFIG  >  install/.../config/config.yaml
```

本机 deployment：

```text
/etc/systemd/system/edge_server.service
  Environment=EDGE_SERVER_CONFIG=/opt/edge_server/config/deploy.yaml
  ExecStart=bash -lc '... ros2 launch /opt/edge_server/deploy/edge_server.launch.py'
```

| 配置 | 位置 | 本机值 |
|---|---|---|
| 运行配置 | `/opt/edge_server/config/deploy.yaml` | 远程专用，不随 rsync 更新 |
| 仓库实车配置 | `edge_server/config/robot.yaml` | 版本管理，默认值 |
| 串口 | `base.port` / `base.baud` | `/dev/ttyAMA1` / `115200` |
| 手动控制重发 | `edge.motion.resend_manual` | `false`（按下只发一次；`true`=旧行为） |
| 手动/ raw 刷新周期 | `edge.motion.keepalive_ms` | `200` |
| Web / MJPEG | `edge.web` | `0.0.0.0:80` / `8080` |
| 遥测状态文件 | `edge.state_file` | `/var/lib/edge_server/edge_params.json` |
| 健康网关 | `health.http` | `127.0.0.1:8090` |

改 `deploy.yaml` 后必须 `systemctl restart edge_server`；改仓库 `robot.yaml`
不会自动影响这台机器。

---

## 10. 分层排查地图

按“从浏览器到 LED”的顺序查，命中哪层就停在哪层：

| 层 | 怎么查 | 正常表现 |
|---|---|---|
| 浏览器 | F12 → Network，点按钮看 `/cmd` | 200，响应 `ok` |
| HTTP | `curl -X POST -d 'button:stop:down' http://IP/cmd` | `ok` |
| 节点 | `journalctl -u edge_server` | `serial connected: /dev/ttyAMA1@115200` |
| 状态 | `curl http://IP/status` | `serial_connected:true`、`encoder_ok:true` |
| 串口写 | `strace -f -e write -p <node_pid>` | `write(19, "0\n", 2) = 2` |
| Pi 引脚 | `ls /dev/ttyAMA1`、`dtoverlay=uart3` | 设备存在 |
| STM32 收 | ST-Link watch `g_ps2_rx_bytes/frames` | 字节/帧持续增长 |
| STM32 执行 | 看 PC13 LED | 每收一条命令翻转一次 |
| 电机 | 看 8 路 IN 电平 / 电机供电 | 按命令表变化 |

### 本次故障的根因（记录）

1. **GPIO4/5 默认不是 UART**：`/boot/firmware/config.txt` 只有 `enable_uart=1`
   （那是 GPIO14/15 的 UART0），没有 `dtoverlay=uart3`，所以 Pi 上只有
   `/dev/ttyAMA0`，没有 GPIO4/5 对应的设备；
2. **配置指向了不存在的设备**：`deploy.yaml` 里是 `/dev/ttyUSB0`（USB 串口），
   实际接线是 GPIO UART；
3. **波特率不一致**：配置 9600，固件 `BODY_UART_BAUD_RATE=115200`。

修复后：`dtoverlay=uart3` → `/dev/ttyAMA1`，`base.port=/dev/ttyAMA1`、
`base.baud=115200`，遥测 `$B,...` 正常，`serial_connected=true`。

> 注意：HTTP `/cmd` 返回 `ok` 只代表命令被接受，**不代表串口写成功**；
> 真正的下行确认要么看 `strace` 的 `write`，要么看 STM32 的 LED/计数器。

---

## 11. 最小验证命令

```bash
# 1) 看串口设备与权限
ls -l /dev/ttyAMA1          # 应存在，root:dialout
groups | grep dialout       # ubuntu 应在 dialout 组

# 2) 看服务是否连上
curl -s http://192.168.3.26/status | python3 -m json.tool | grep -E "serial|encoder"

# 3) 发一条安全停车命令（LED 应翻转一次）
curl -s -X POST -H 'Content-Type: text/plain' \
  --data 'button:stop:down' http://192.168.3.26/cmd; echo

# 4) 直接读遥测（先停服务，避免抢占串口）
sudo systemctl stop edge_server
python3 - <<'PY'
import serial
s = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
for _ in range(3):
    print(s.readline())
s.close()
PY
sudo systemctl start edge_server
```

如果第 3 步 LED 不翻转，但第 4 步能读到 `$B,...`：

- 检查 Pi GPIO4 → STM32 PB11 这根线（TX 方向）；
- 检查两块板是否共地；
- 用 ST-Link 看 `g_ps2_rx_bytes` 是否增长；
- 确认烧录的固件与 `software/body_control` 当前源码一致（115200、LED 翻转逻辑）。
