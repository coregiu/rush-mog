# Edge Server 部署与发布说明

> 目标机：`ubuntu@192.168.3.26`（Raspberry Pi 4B，Ubuntu 22.04 arm64）
> 部署目录：`/opt/edge_server`
> 本文只描述当前这台机器的部署方式，不包含密码/密钥。
>
> **日常启停、日志、整机开关机、故障排查见 [OPERATIONS.md](OPERATIONS.md)。**

---

## 1. 安装内容

### 1.1 系统与 ROS 环境

- 修复 arm64 apt 源：
  - 原配置使用 `mirrors.aliyun.com/ubuntu/`，arm64 包实际在 `ubuntu-ports/`，已改为
    `mirrors.aliyun.com/ubuntu-ports/`；
  - 备份：`/etc/apt/sources.list.bak.rushmog`；
  - 注释了 `deb-src` 源，减少索引下载量。
- 添加 ROS 2 apt 源：
  - keyring：`/usr/share/keyrings/ros-archive-keyring.gpg`；
  - source：`/etc/apt/sources.list.d/ros2.list`；
  - 使用 USTC 镜像：`https://mirrors.ustc.edu.cn/ros2/ubuntu`。
- 安装：
  - `ros-humble-ros-base`、`ros-humble-cv-bridge`、`ros-humble-tf2-ros`、
    `ros-humble-message-filters`、`ros-humble-rosidl-default-generators`、
    `ros-humble-rosidl-default-runtime`、`ros-humble-launch-ros`；
  - `python3-colcon-common-extensions`、`python3-pytest`；
  - `python3-opencv`、`python3-numpy`、`python3-yaml`、`python3-serial`；
  - `build-essential`、`cmake`。
- 启用 GPIO4/5 的 UART3（2026-11-14）：
  - `/boot/firmware/config.txt` 末尾追加 `dtoverlay=uart3`；
  - 重启后生成 `/dev/ttyAMA1`，供 `edge_server_node` 直连 STM32 USART3；
  - `enable_uart=1` 只启用 GPIO14/15 的 UART0，不能代替 `uart3`。
- 蓝牙音箱/语音播报（2026-11-14）：
  - 安装 `pipewire`、`pipewire-pulse`、`wireplumber`、
    `libspa-0.2-bluetooth`、`pulseaudio-utils`、`alsa-utils`、`espeak-ng`；
  - `loginctl enable-linger ubuntu`，启用用户级 `pipewire`、
    `pipewire-pulse`、`wireplumber`；
  - BlueZ 已配对/信任 `newmine`（`73:DB:39:F2:15:4B`）；
  - 首次配对：`bluetoothctl pair/trust/connect`，之后由 `edge_server_node`
    按 `edge.speaker.connect_interval_sec` 自动重连；
  - 麦克风走 HFP：`pulseaudio-utils` 提供 `parec`，录音前切
    `headset-head-unit`，停止后恢复 `a2dp-sink`；部署配置启用
    `edge.speaker.microphone`，录音目录 `/var/lib/edge_server/audio/mic`。

### 1.2 代码与构建

- 从本地 `software/edge_server` 复制到 `/opt/edge_server/src/edge_server`；
- 从本地 `software/custom_msgs` 复制到 `/opt/edge_server/src/custom_msgs`；
- 在 `/opt/edge_server` 下执行：
  ```bash
  source /opt/ros/humble/setup.bash
  colcon build --packages-select custom_msgs edge_server
  ```
- 生成 `/opt/edge_server/build|install|log`；运行使用
  `/opt/edge_server/install/setup.bash`。

### 1.3 部署配置

- `/opt/edge_server/config/deploy.yaml`（远程专用，不在仓库里）：
  - Web：`0.0.0.0:80`
  - MJPEG：`0.0.0.0:8080`
  - ROS 健康网关：`127.0.0.1:8090`
  - STM32 串口：`/dev/ttyAMA1`（GPIO4/5 → UART3），115200 8N1
    - 2026-11-14 从 USB 串口（`/dev/ttyUSB0@9600`）改为 GPIO UART；
      `dtoverlay=uart3` 已写入 `/boot/firmware/config.txt`；
    - 接线：Pi GPIO4(TXD3)→STM32 PB11(RX)，GPIO5(RXD3)→PB10(TX)，必须共地
  - 状态文件：`/var/lib/edge_server/edge_params.json`
  - 蓝牙音箱：`edge.speaker` 启用，`device_name=newmine`、
    `device_mac=73:DB:39:F2:15:4B`，音频目录 `/var/lib/edge_server/audio`，
    A2DP 输出走本机 PipeWire（sink 名 `bluez_output.73_DB_39_F2_15_4B.a2dp-sink`）
  - 树莓派电源监控：`edge.power` 启用，读取 `vcgencmd get_throttled` 与
    `rpi_volt`，发布 `/body/power_status`；dashboard“电源及转换板”模块使用该状态
  - 健康检查：监控 camera/lidar/imu/base/odom/speaker/microphone/power 8 个设备，
    `check_nodes=false`、`safety_required=false`；音箱/麦克风/电源 `online=false`
    或电源异常时对应设备显示 `degraded`
- `/opt/edge_server/deploy/edge_server.launch.py`：只启动
  `edge_server_node` + `health_monitor_node`，不启动 SLAM/Nav2/感知全栈。

### 1.4 systemd 服务

- 单元文件：`/etc/systemd/system/edge_server.service`
- 运行用户：`ubuntu`
- 通过 `AmbientCapabilities=CAP_NET_BIND_SERVICE` 绑定 80 端口
- `After=network-online.target bluetooth.service user@1000.service`，
  `Wants=... bluetooth.service`
- 音频会话环境（让播放器连到用户级 PipeWire/Pulse）：
  ```ini
  Environment=XDG_RUNTIME_DIR=/run/user/1000
  Environment=PULSE_SERVER=unix:/run/user/1000/pulse/native
  Environment=DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
  ```
- 已执行：
  ```bash
  sudo systemctl daemon-reload
  sudo systemctl enable edge_server
  sudo systemctl start edge_server
  ```
- 运行目录：
  - ROS home：`/var/lib/edge_server/ros`
  - ROS 日志：`/var/log/edge_server/ros`
  - 应用日志：`journalctl -u edge_server`

### 1.5 当前状态

- `edge_server.service`：`active`，已开机自启
- 端口：
  - `0.0.0.0:80`
  - `0.0.0.0:8080`
  - `127.0.0.1:8090`
- STM32：`serial_connected=true`（`/dev/ttyAMA1@115200`）
- 未启动 ROS 相机驱动：`camera=0`，`/stream` 返回 503
- 蓝牙耳机：`newmine` 已连接，A2DP sink
  `bluez_output.73_DB_39_F2_15_4B.a2dp-sink`，`online=true`；
  手动断开后 20s 内自动重连成功，`/api/speaker/say` 实测可播报
  （中文 `espeak-ng`），health monitor 的 speaker 为 `ok`
- 蓝牙麦克风：HFP profile 可用，`/api/microphone/start|stop` 录音实测成功
  （8 kHz mono WAV，`/edge/audio` 10 Hz AudioData），停止后 A2DP 自动恢复，
  health monitor 的 microphone 为 `ok`
- 电源监控：`/api/power` 返回 `status=ok`，`throttled_raw=0x50000`
  （历史欠压/降频已恢复，`warning=true`）；当前 `in0_lcrit_alarm=0`，
  health monitor 的 power 为 `ok`
- ROS 健康整体为 `degraded`（缺相机/雷达/IMU/底盘数据），当前属预期

---

## 2. 本地开发目录和部署目录的关系

```text
本地开发机（源码，Git 管理）
F:\CODE\rush-mog\software\edge_server
（WSL 路径：/mnt/f/CODE/rush-mog/software/edge_server）
│
│  rsync / tar 复制（手动发布，不是自动同步）
▼
192.168.3.26:/opt/edge_server/src/edge_server
（源码副本，不是 git 工作区，也不是软链接）
│
│  colcon build
▼
/opt/edge_server/install/edge_server
（实际运行产物：Python 包、launch、config、web）
│
│  systemd 启动
▼
edge_server_node + health_monitor_node
```

关键点：

1. **本地是唯一的开发源**；远端 `src/edge_server` 只是发布快照，改本地不会自动影响远端。
2. **不是软链接**：远端不能通过改本地文件“热更新”。
3. `/opt/edge_server/install/` 是构建产物，不要手改，重新 `colcon build` 会覆盖。
4. `/opt/edge_server/config/deploy.yaml` 是**远程专用覆盖配置**，仓库里没有；
   服务通过 `EDGE_SERVER_CONFIG` 指向它。改仓库的
   `edge_server/config/robot.yaml` 不会自动改变这台机器的运行配置。
5. `/opt/edge_server/deploy/edge_server.launch.py` 和
   `/etc/systemd/system/edge_server.service` 也是远程部署文件；
   仓库里的 `launch/edge_server.launch.py` 是全量 ROS 启动文件，服务没有用它。
6. 本地 Git 提交不会自动传到 Pi；发布 = 同步源码 + 远端重新构建 + 重启服务。

---

## 3. 以后新开发怎么发布

### 3.1 推荐：配置 SSH 免密（只需一次）

```bash
ssh-keygen -t ed25519 -C "rush-mog-edge"   # 已有可跳过
ssh-copy-id ubuntu@192.168.3.26
```

之后 rsync/ssh 不再反复输入密码。

### 3.2 本地先跑测试

```bash
cd /mnt/f/CODE/rush-mog/software/edge_server
PYTHONPATH=.:$PYTHONPATH python3 -m pytest -q tests
```

当前基线：`126 passed`。

### 3.3 同步源码到 Pi

在 WSL 下执行（`/mnt/f/CODE/rush-mog`）：

```bash
cd /mnt/f/CODE/rush-mog

rsync -avz --delete \
  --exclude='__pycache__' --exclude='.pytest_cache' \
  software/edge_server/ \
  ubuntu@192.168.3.26:/opt/edge_server/src/edge_server/

rsync -avz --delete \
  --exclude='__pycache__' --exclude='.pytest_cache' \
  software/custom_msgs/ \
  ubuntu@192.168.3.26:/opt/edge_server/src/custom_msgs/
```

说明：

- `--delete` 只清理远端这两个源码副本中多余的文件，不会碰
  `/opt/edge_server/config`、`deploy`、`install`。
- 如果改了 `custom_msgs`，第二条 rsync 必须执行；只改 Python/网页可以只同步
  `edge_server`。

### 3.4 远端重新构建并重启

```bash
# 注意：远端登录 shell 是 zsh，必须显式用 bash -lc 包一层
ssh ubuntu@192.168.3.26 'bash -lc "
  source /opt/ros/humble/setup.bash &&
  cd /opt/edge_server &&
  colcon build --packages-select custom_msgs edge_server &&
  sudo systemctl restart edge_server
"'
```

只改了 `edge_server` 的 Python/网页、没有改消息时，可以只构建 `edge_server`：

```bash
ssh ubuntu@192.168.3.26 'bash -lc "
  source /opt/ros/humble/setup.bash &&
  cd /opt/edge_server &&
  colcon build --packages-select edge_server &&
  sudo systemctl restart edge_server
"'
```

构建耗时参考：

- 首次 `custom_msgs`：约 7 分钟（Pi 4B）；
- 之后只构建 `edge_server`：约 10 秒。

### 3.5 验证

```bash
ssh ubuntu@192.168.3.26 'systemctl is-active edge_server'
curl -s http://192.168.3.26/status
curl -s -o /dev/null -w '%{http_code}\n' http://192.168.3.26/
curl -s -o /dev/null -w '%{http_code}\n' http://192.168.3.26/dashboard.html
curl -s -o /dev/null -w '%{http_code}\n' http://192.168.3.26:8080/stream
curl -s http://192.168.3.26/api/speaker | python3 -m json.tool
curl -s http://192.168.3.26/api/modules/edge/params | python3 -m json.tool | grep -A2 speaker
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -n 50 --no-pager'
```

---

## 4. 不同改动类型的发布方式

| 改动 | 需要同步 | 需要构建 | 需要重启服务 | 备注 |
|---|---|---|---|---|
| 节点 Python 逻辑 | `edge_server` | `edge_server` | 是 | 最常见 |
| `web/` 页面/JS/CSS | `edge_server` | `edge_server` | 建议重启 | 构建会把 web 装到 install；浏览器强刷 |
| `launch/`、`config/robot.yaml` | `edge_server` | `edge_server` | 是 | 但运行配置仍以远程 `deploy.yaml` 为准 |
| `package.xml` / `setup.py` / 入口点 | `edge_server` | `custom_msgs edge_server` 或清理后全量 | 是 | 入口点变化必须重装 |
| `custom_msgs` | `custom_msgs` + `edge_server` | 两者都构建 | 是 | 所有使用该消息的进程都需重启 |
| 只改远程 `config/deploy.yaml` | 不需要 rsync | 不需要 | 是 | 直接编辑远端配置 |
| 改 `deploy/edge_server.launch.py` | 单独复制 | 不需要 | 是 | 文件在 `/opt/edge_server/deploy/` |
| 改 `/etc/systemd/system/edge_server.service` | 单独复制 | 不需要 | `daemon-reload` + restart | 注意文件在 `/etc` |

### 4.1 只改远程运行配置

```bash
ssh ubuntu@192.168.3.26
sudoedit /opt/edge_server/config/deploy.yaml
sudo systemctl restart edge_server
```

### 4.2 改了部署 launch

```bash
scp /path/to/edge_server.launch.py \
  ubuntu@192.168.3.26:/opt/edge_server/deploy/edge_server.launch.py
ssh ubuntu@192.168.3.26 'sudo systemctl restart edge_server'
```

### 4.3 改了 systemd 单元

```bash
scp /path/to/edge_server.service \
  ubuntu@192.168.3.26:/tmp/edge_server.service
ssh ubuntu@192.168.3.26 '
  sudo cp /tmp/edge_server.service /etc/systemd/system/edge_server.service &&
  sudo systemctl daemon-reload &&
  sudo systemctl restart edge_server
'
```

### 4.4 回滚

发布前建议保留上一版源码（放在 colcon 工作区之外，否则会被当成重名包）：

```bash
ssh ubuntu@192.168.3.26 '
  mkdir -p /opt/edge_server/backup &&
  rm -rf /opt/edge_server/backup/edge_server.bak &&
  cp -a /opt/edge_server/src/edge_server /opt/edge_server/backup/edge_server.bak
'
```

回滚时用 `rsync -a --delete` 把备份写回 `src/edge_server`，重新 build 并 restart
即可（见 OPERATIONS.md §6.5）。

---

## 5. 当前访问地址

- 手柄页：`http://192.168.3.26/`
- 数字孪生：`http://192.168.3.26/dashboard.html`
- 状态：`http://192.168.3.26/status`
- 模块：`http://192.168.3.26/api/modules`
- 图传：`http://192.168.3.26:8080/stream`
- ROS 健康代理：`http://192.168.3.26/api/modules/ros/health`
- 音箱状态/播报：`http://192.168.3.26/api/speaker`

---

## 6. 注意事项

- STM32 已通过 GPIO4/5（UART3，`/dev/ttyAMA1`）接入，115200 遥测正常；
  接线/排障见 [OPERATIONS.md](OPERATIONS.md) §8。
- 当前没有启动 ROS 相机驱动，`/stream` 返回 503；相机驱动发布
  `/camera/image_raw` 后会自动出图。
- 服务只启动 Edge + 健康网关，不启动 SLAM/Nav2/感知全栈；
  需要全栈时另行启动对应 launch/节点。
- 不要直接修改 `/opt/edge_server/install/`，重新构建会覆盖。
- 本地 Git 提交 ≠ 已发布；每次发布按“同步 → 构建 → 重启 → 验证”执行。
- 蓝牙/音频栈是系统级配置（apt 包、`loginctl enable-linger`、用户级
  PipeWire/WirePlumber、systemd 的 `XDG_RUNTIME_DIR`），不在仓库里；
  重装系统后需按 §1.1/§1.4 重建，再重启 `edge_server`。
- 麦克风录音会临时把输出切到 HFP（8 kHz mono），停止录音后自动恢复 A2DP；
  录音文件在 `/var/lib/edge_server/audio/mic`，只允许通过 API 下载。

---

## 7. 蓝牙音箱 / 语音播报

### 7.1 自动连接

1. 音箱开机，且不要被手机/其他主机占用；
2. `edge_server_node` 启动后立即尝试连接，之后每 20s 重试；
3. 连接成功后 PipeWire 创建 `bluez_output.<MAC>.a2dp-sink` sink，节点将其设为默认
   输出并应用 `edge.speaker.volume`；
4. 状态：
   ```bash
   curl -s http://192.168.3.26/api/speaker | python3 -m json.tool
   ```
   - `state=connected, audio_ready=true`：可播报；
   - `state=error`：看 `last_error`（常见 `br-connection-page-timeout`）；
   - `state=scanning/connecting`：正在重试。

手动强制重连：

```bash
curl -X POST http://192.168.3.26/api/speaker/connect
```

### 7.2 语音播报

```bash
curl -X POST http://192.168.3.26/api/speaker/say \
  -H 'Content-Type: application/json' -d '{"text":"机器人已启动"}'

# 也可以直接用 ROS 话题
ros2 topic pub --once /body/say std_msgs/msg/String "{data: 请注意避让}"
```

- TTS 使用 `espeak-ng -v cmn`，离线可用；音色/语速在
  `edge.speaker.tts.voice/speed` 调整；
- 每个 TTS 默认前置 450ms 静音（`tts.pre_roll_ms`），避免 A2DP 从
  SUSPENDED 唤醒时吞掉第一个字；
- `playback_latency_ms`（默认 250）通过 `PULSE_LATENCY_MSEC` 请求更大
  播放缓冲，降低蓝牙/WiFi 抖动造成的卡顿；
- 播放队列串行，最多 20 条；`POST /api/speaker/stop` 清空并停止；
- 播放本地文件：先放到 `/var/lib/edge_server/audio/`，再
  `POST /api/speaker/play -d '{"file":"prompt.wav"}'`；只允许该目录
  （防任意路径读取），支持 wav/mp3/ogg/flac/m4a/aac。

### 7.3 常见问题

- **`br-connection-page-timeout`**：音箱未响应经典蓝牙寻呼。多数是：
  音箱被手机连着、处于休眠/关机、超出距离，或需要按一下音箱上的配对键。
  在手机端断开后，节点会在下一个 20s 周期自动连上。
- **已连接但 `audio_ready=false`**：PipeWire/WirePlumber 没起来。检查：
  ```bash
  systemctl --user is-active pipewire pipewire-pulse wireplumber
  pactl list short sinks | grep bluez_output
  loginctl show-user ubuntu -p Linger
  ```
- **能连但没声音**：确认 `pactl get-default-sink` 是 bluez sink，
  `wpctl status` 中该 sink 未静音；必要时 `pactl set-sink-mute <name> 0`。
- **服务里 `pactl` 报连接失败**：systemd 单元缺少 §1.4 的
  `XDG_RUNTIME_DIR`/`PULSE_SERVER`，`daemon-reload` 后重启。

### 7.4 麦克风录音（HFP）

`newmine` 的麦克风只在 HFP/HSP profile 下可用；录音模块会自动：

1. 停掉当前 A2DP 播放；
2. `pactl set-card-profile bluez_card.<MAC> headset-head-unit`；
3. 等待 `bluez_input.<MAC>.headset-head-unit` 输入源；
4. `parec` 采集 8 kHz mono s16le，写 WAV 并按 100ms 发布 `/edge/audio`；
5. 停止后恢复原 A2DP profile 和播放 sink。

```bash
# 开始 10 秒录音
curl -X POST http://192.168.3.26/api/microphone/start \
  -H 'Content-Type: application/json' -d '{"duration":10}'

# 查看状态 / 停止
curl -s http://192.168.3.26/api/microphone | python3 -m json.tool
curl -X POST http://192.168.3.26/api/microphone/stop

# 列表 / 下载 / 回放
curl -s http://192.168.3.26/api/microphone/recordings
curl -OJ http://192.168.3.26/api/microphone/recordings/<file>.wav
curl -X POST http://192.168.3.26/api/microphone/play \
  -H 'Content-Type: application/json' -d '{"file":"<file>.wav"}'
```

录音期间：
- `GET /api/microphone` 的 `recording=true`，`source=bluez_input...`；
- 默认 `gain=3.0` 会在写 WAV/发 ROS 前做软件增益（约 +9.5dB）；
  可在 `/api/microphone/gain` 或 dashboard 参数里调整 1.0..8.0；
- `ros2 topic hz /edge/audio` 约 10 Hz；
- 输出 sink 变为 `...headset-head-unit`，属正常；
- 停止后 `GET /api/speaker` 的 sink 恢复 `...a2dp-sink`。

排查：
- `supported=false`：card 没有 `headset-head-unit` profile，检查
  `pactl list cards`；确认不是只支持 A2DP 的音箱。
- `未找到蓝牙麦克风输入源`：HFP profile 切换失败/被手机占用；
  先 `bluetoothctl disconnect/connect`，再重试。
- 录音文件很小或 RMS 接近 0：麦克风静音/距离远；检查
  `pactl list sources` 的 `Mute`/`Volume`，用 `/api/microphone/volume` 调大。

### 7.5 电源监控（电源及转换板）

树莓派 5V 输入没有独立的 ADC，使用 SoC 固件与 `rpi_volt` hwmon 判定：

```bash
vcgencmd get_throttled          # throttled=0x50000
cat /sys/class/hwmon/hwmon1/in0_lcrit_alarm   # 0=正常，1=当前欠压
vcgencmd measure_volts core
vcgencmd measure_temp
curl -s http://192.168.3.26/api/power | python3 -m json.tool
```

位定义与状态映射：

- `0x1` 当前欠压 → `error`；
- `0x2` ARM 限频、`0x4` 降频/限流、`0x8` 软温度限制 → `degraded`；
- `0x10000` 历史欠压、`0x40000` 历史降频、`0x80000` 历史温度限制
  → 当前 `ok`，`warning=true`，dashboard 显示“历史欠压已恢复”；
- `/sys/class/hwmon/hwmon*/in0_lcrit_alarm=1` → `error`。

当前部署实测 `throttled=0x50000`，是此前一次瞬时欠压留下的历史位，
`in0_lcrit_alarm=0`，所以电源模块显示 `ok`。若需要清零历史位，
重启树莓派或重新插拔电源后再观察；持续出现 `0x10000` 才说明供电/线缆
/电源转换板需要检查。

dashboard 的「电源及转换板」卡片直接使用 `/api/power`；health monitor 的
`power` 设备使用 `/body/power_status`，电源异常时整体健康会降级。

### 7.6 语音卡顿/少第一个字与麦克风音量优化

已部署的优化（2026-11-14）：

1. **TTS 预滚 450ms**：`edge.speaker.tts.pre_roll_ms`，在 espeak 合成后
   给 WAV 前补静音，覆盖 A2DP 从挂起唤醒/功放启动的时间，避免第一个字被吞。
2. **播放缓冲 250ms**：`edge.speaker.playback_latency_ms`，通过
   `PULSE_LATENCY_MSEC` 让 `paplay` 请求更大缓冲，降低蓝牙链路抖动卡顿。
3. **A2DP sink 常驻**：`/home/ubuntu/.config/wireplumber/bluetooth.lua.d/51-rush-mog.lua`
   给 `bluez_output.*` 设置 `session.suspend-timeout-seconds=0`，播放结束后
   sink 保持 IDLE 而不是 SUSPENDED，减少下次唤醒丢字。
4. **PipeWire 实时调度**：`/etc/polkit-1/localauthority/50-local.d/50-rush-mog-rtkit.pkla`
   允许 `ubuntu` 获取 RealtimeKit 实时权限；重启后 `pipewire` 音频线程为
   `RR 20`，不再报 `RTKit ... AccessDenied`。
5. **WiFi 省电关闭**：`/etc/systemd/system/wifi-powersave-off.service`
   开机执行 `iw dev wlan0 set power_save off`，降低 2.4GHz WiFi 与蓝牙
   共存抖动。
6. **麦克风软件增益**：`edge.speaker.microphone.gain` 默认 `3.0`（约 +9.5dB），
   在写 WAV/发 ROS 前用 `audioop.mul` 放大；可用
   `POST /api/microphone/gain {"gain":3.0}` 在线调整（1.0..8.0）。

检查：

```bash
# sink 播放后保持 IDLE 而不是 SUSPENDED
pactl list short sinks | grep bluez_output
# PipeWire 实时线程
ps -eLo pid,tid,cls,rtprio,ni,comm | grep -E 'pipewire|wireplumber'
# WiFi 省电
iw dev wlan0 get power_save
# 麦克风增益/录音
curl -s http://192.168.3.26/api/microphone | python3 -m json.tool
```

如果仍卡顿，优先把 Pi 的 WiFi 切到 **5GHz**（当前 `wlan0` 在 2.4GHz
channel 1，与蓝牙共用天线），或靠近路由器/音箱；这是无线共存问题，
软件缓冲只能缓解，不能完全消除 2.4GHz 干扰。
