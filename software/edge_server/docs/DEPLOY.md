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
- SLAM/导航依赖（2026-11-14）：
  - 已安装 `ros-humble-slam-toolbox`，可直接 `ros2 launch edge_server slam_stack.launch.py`；
  - `ros-humble-navigation2`、`ros-humble-robot-localization` 尚未安装；
    没有 Nav2 时 `navigation_node` 的 `backend: auto` 会回退到内置直航。
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
  - 健康检查：监控 camera/depth_camera/lidar/cliff/imu/base/odom/speaker/microphone/power 10 个设备，
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
- 激光雷达：代码/`lidar_node`/`rush-mog-lidar.service` 已安装并 enable；
  当前未接 D6、未启用 `dtoverlay=uart4`，所以 `/dev/rush_lidar` 不存在，
  service 因 `ConditionPathExists` 静默跳过；接好线并重启后自动启动
- Astra Pro：`ros-humble-orbbec-camera` + Orbbec udev 已安装；当前 `lsusb`
  没有 `2bc5` 设备，驱动未启动，`camera/depth_camera` 健康为 `unknown`；
  插上相机后按 §7.8 启动
- 悬崖：STM32 固件源码已加 `cliff_mask`，本机已编译出
  `software/body_control/build/src/body_control.bin`；**当前 STM32 仍运行
  旧 8 字段固件，尚未烧录**，所以 `/body/cliff_status` 的 `available=false`，
  dashboard 显示“无悬崖遥测”；烧录 §7.9 的固件后生效
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

当前基线：`132 passed`。

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
  `/camera/color/image_raw` 后会自动出图。
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

### 7.7 激光雷达（D6/LD19）

代码已内置 `lidar_node`（`edge_server/core/lidar.py` + `ros_nodes/lidar_node.py`），
支持 LD19/LD06/D6 的 47 字节协议（`0x54 0x2C`，CRC-8 0x4D）。

接线与系统配置：

```ini
# /boot/firmware/config.txt
dtoverlay=uart4          # GPIO8/9，D6 激光；不要同时启用 spi0
```

```bash
# /etc/udev/rules.d/99-rush-mog.rules（设备名以 ls /dev/ttyAMA* 为准）
KERNEL=="ttyAMA3", SYMLINK+="rush_lidar"
sudo udevadm control --reload-rules && sudo udevadm trigger
# 断电重启后确认
ls -l /dev/rush_lidar /dev/ttyAMA*
```

配置 `robot.yaml`：

```yaml
lidar:
  enabled: true
  port: "/dev/rush_lidar"
  baud: 230400             # 按 D6 说明书；有些批次是 921600
  protocol: "ld19"
  topic: "/scan"
  frame_id: "laser_link"
  bins: 360
  range_min: 0.02
  range_max: 12.0
  publish_tf: true
  tf_parent: "base_link"
  tf_x: 0.0
  tf_y: 0.0
  tf_z: 0.18               # 按实测安装高度改
  tf_yaw_deg: 0.0
```

单独启动（不启动全栈）：

```bash
ros2 run edge_server lidar_node --ros-args \
  -p config_file:=/opt/edge_server/config/deploy.yaml
```

或随实车 bringup：

```bash
ros2 launch edge_server robot_bringup.launch.py start_lidar:=true
```

验收：

```bash
ros2 topic hz /scan             # ≈10 Hz
ros2 topic echo /scan --once    # ranges 长度 360，frame_id=laser_link
ros2 topic echo /lidar/status --once
ros2 run tf2_ros tf2_echo base_link laser_link
```

当前这台 Pi 还没有接 D6、也没有启用 `dtoverlay=uart4`，所以 `/dev/rush_lidar`
暂时不存在；`ros-humble-slam-toolbox` 已安装。已安装并 enable 了可选的
`rush-mog-lidar.service`：

- 单元文件：`/etc/systemd/system/rush-mog-lidar.service`（仓库模板
  `software/tools/systemd/rush-mog-lidar.service`）；
- `ConditionPathExists=/dev/rush_lidar`：设备不存在时静默跳过；
- 接好线、创建 udev symlink 并重启后，服务会自动启动 `lidar_node`。

接好硬件后：

```bash
# 1) 确认 /boot/firmware/config.txt 有 dtoverlay=uart4
# 2) 按实际 ttyAMA* 建 udev symlink
sudo tee /etc/udev/rules.d/99-rush-mog.rules >/dev/null <<'RULE'
KERNEL=="ttyAMA3", SYMLINK+="rush_lidar"
RULE
sudo udevadm control --reload-rules && sudo udevadm trigger
# 3) 重启（或手动 systemctl start rush-mog-lidar）
sudo reboot
```

若波特率或协议不同，改 `lidar.baud`/`lidar.protocol`；若设备名不同，改 udev
规则或 `lidar.port`。

### 7.8 Astra Pro 深度相机

驱动：`ros-humble-orbbec-camera`（已安装），udev：
`/etc/udev/rules.d/99-obsensor-libusb.rules`（已从
`/opt/ros/humble/share/orbbec_camera/udev/` 复制）。

```bash
# 不建议经 Hub；插 Pi 4 USB3 口，Astra Pro 是 USB2.0 设备
lsusb | grep -i 2bc5          # 应看到 Orbbec/Astra（2bc5:0403 等）

ros2 launch edge_server astra.launch.py
# 话题：
#   /camera/color/image_raw
#   /camera/depth/image_raw
#   /camera/color/camera_info
#   /camera/depth/camera_info
```

`astra.launch.py` 默认：

- `depth_registration:=true`：深度对齐到彩色，bbox 可直接采深度；
- `enable_point_cloud:=false`：Pi 4B 省 CPU/带宽；
- 640×480@10Hz；
- 发布 `base_link → camera_link` 静态 TF，默认 `x=0.08,y=0,z=0.22`。

Edge 侧：

- `edge.camera.topic` / `perception.image_topic` 已切到
  `/camera/color/image_raw`；
- `perception.depth_topic` 为 `/camera/depth/image_raw`，`prefer_depth: true`；
- `target_detector_node` 用深度 bbox 中心区域中值优先测距，深度不可用时再回退
  激光/bbox 高度；
- health `depth_camera` 监控深度话题；dashboard 增加「深度相机」卡片；
- `robot_bringup.launch.py start_camera:=true` 可随整车启动。

验收：

```bash
ros2 topic hz /camera/color/image_raw   # ≈10 Hz
ros2 topic hz /camera/depth/image_raw   # ≈10 Hz
ros2 topic echo /camera/depth/image_raw --once --field encoding   # 16UC1/32FC1
ros2 topic echo /camera/depth/camera_info --once
ros2 run tf2_ros tf2_echo base_link camera_link
```

当前这台 Pi 尚未插 Astra Pro（`lsusb` 没有 2bc5 设备），因此驱动未启动。
已安装并 enable 可选的 `rush-mog-camera.service`（`ConditionPathExists=/dev/astra_pro`）：

- 相机插上且 Orbbec udev 规则生效后，重启或
  `sudo systemctl start rush-mog-camera` 即启动；
- 单元文件模板：`software/tools/systemd/rush-mog-camera.service`；
- 手动调试仍推荐 `ros2 launch edge_server astra.launch.py`。
如果 Orbbec 驱动无法识别老款 Astra Pro，先确认 `lsusb`/`ros2 topic list`；
仍不行则换 Astra Pro Plus/Gemini 2/D435i，Edge 侧话题/深度分支不用改。

### 7.9 悬崖传感器（STM32 PB3/PB4）

悬崖传感器接在 STM32 的 PB3/PB4，由 `body_control` 固件本地做 10ms 采样、
30ms 消抖，并在触发时封锁运动。Edge 侧要看到状态，STM32 必须上报悬崖掩码：

```text
$B,<seq>,<uptime_ms>,<fl>,<fr>,<rl>,<rr>,<distance_mm>,<cliff_mask>\n
                                                        ^ bit0=左前 PB3
                                                          bit1=右前 PB4
```

固件源码已更新（`encoder_sensor.c` 追加 `cliff_sensor_get_mask()`），需要重新编译/烧录：

```bash
cd software/body_control/build
make -j2
# ST-Link 接 STM32 后：
sudo st-flash write src/body_control.bin 0x8000000
```

烧录后重启 Edge Server，验证：

```bash
# 直接看 STM32 原始遥测，应为 9 个字段
ros2 topic echo /base/status --once
ros2 topic echo /body/cliff_status --once
curl -s http://127.0.0.1:8090/ros/health | python3 -m json.tool | grep -A8 '"id": "cliff"'
```

`/body/cliff_status` 示例：

```json
{"source":"stm32","online":true,"available":true,"triggered":false,
 "left":false,"right":false,"mask":0,"state":"clear","seq":123,"age_ms":80,"stale":false}
```

Dashboard「悬崖传感器」卡片：

- 左右均未触发：绿色 `CLEAR`；
- 任一触发：红色 `CLIFF`，详情显示左/右，CHIP 为 `LEFT/RIGHT CLIFF`；
- 旧固件/无遥测：显示“无悬崖遥测（需刷入带 cliff_mask 的 STM32 固件）”；
- 触发/解除会写入 Edge 运行事件（最近事件面板）。

health `cliff` 设备（`required: false`）在触发或遥测缺失时标红，不影响其它必需设备；
如果要让整体健康因悬崖遥测缺失直接失败，可把 `required` 改为 `true`。

> 先在桌面上确认左右都 `CLEAR`，再分别挡住/移到桌沿确认对应 `CLIFF`；
> 触发期间固件只允许 STOP/BACK，这是安全策略，不是 Edge 软件限制。
