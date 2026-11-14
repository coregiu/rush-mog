# Edge Server 运维手册：启停、发布、排障

> 目标机：`ubuntu@192.168.3.26`（Raspberry Pi 4B，Ubuntu 22.04 arm64）
> 部署目录：`/opt/edge_server`，服务名：`edge_server.service`
> 配套文档：[DEPLOY.md](DEPLOY.md)（装机记录/发布）、[CODE_MAP.md](CODE_MAP.md)（代码结构/调用链）、[HEALTHCHECK.md](HEALTHCHECK.md)（健康检查）

本文的启停、构建、验证命令均在 **2026-11-14 的实机**上验证过。命令分两类：

- **本地 WSL**：在 `/mnt/f/CODE/rush-mog` 下执行，用 `ssh` 操作 Pi；
- **Pi 上**：先 `ssh ubuntu@192.168.3.26`，再执行。

---

## 0. 三个先记住的结论

1. **`source /opt/ros/humble/setup.bash` 报“找不到文件”，不是 ROS 装坏了。**
   这台机器的登录 shell 已经改成 **zsh**，而 ROS 的 `setup.bash` 依赖 bash 专有变量
   `BASH_SOURCE`。在 zsh 下它会去当前目录找 `setup.sh`，于是报：

   ```text
   /opt/ros/humble/setup.bash:.:11: no such file or directory: /opt/edge_server/setup.sh
   ```

   解决：zsh 用 `setup.zsh`，或者切回 bash。详见 §7.1。

2. **应用只由 systemd 管**，不要手工 `nohup` 起服务；手工起的进程和 systemd 抢 80 端口，
   重启后还会丢。

3. **同步代码 ≠ 已生效**。必须是“rsync 同步 → `colcon build` → `systemctl restart`”三步，
   少一步都还是旧代码。

---

## 1. 系统是怎么起来的

```text
上电
 └─ systemd（multi-user.target）
     └─ edge_server.service（User=ubuntu，开机自启）
         └─ /bin/bash -lc 'source /opt/ros/humble/setup.bash
                          && source /opt/edge_server/install/setup.bash
                          && exec ros2 launch /opt/edge_server/deploy/edge_server.launch.py'
             ├─ edge_server_node    → Web :80、MJPEG :8080、串口 /dev/ttyAMA1（GPIO4/5 UART3）
             └─ health_monitor_node → ROS 健康网关 127.0.0.1:8090
```

注意：

- 服务里写死了 `bash -lc`，所以**即使登录 shell 是 zsh，服务本身也不受 §0 的坑影响**；
- 部署 launch 只启动 Edge + 健康网关，**不启动** SLAM/Nav2/感知全栈；
- 运行配置由 `EDGE_SERVER_CONFIG=/opt/edge_server/config/deploy.yaml` 指定，
  优先级高于 `install/.../config/robot.yaml`。

| 层 | 内容 | 怎么启停 |
|---|---|---|
| 整机 | Pi 电源、系统 | 上电 / `sudo shutdown -h now` |
| 应用 | `edge_server.service` | `systemctl start\|stop\|restart` |
| ROS 全栈（可选） | SLAM / Nav2 / VLA / 感知 | 手工 `ros2 launch`（§5） |
| 上位机 | `software/center_server`（PC 侧） | 与 Pi 独立，不在本文范围 |

---

## 2. 日常启停（最常用）

### 2.1 在本地 WSL 一条命令搞定

```bash
PI=ubuntu@192.168.3.26

# 看状态
ssh $PI 'systemctl status edge_server --no-pager'

# 启动 / 停止 / 重启
ssh $PI 'sudo systemctl start edge_server'
ssh $PI 'sudo systemctl stop edge_server'
ssh $PI 'sudo systemctl restart edge_server'

# 只看是否在跑（脚本里用这个）
ssh $PI 'systemctl is-active edge_server'
```

`stop` 只停应用，Pi 本身还在运行（SSH 仍可登录）。要关机见 §3。

### 2.2 开机自启开关

```bash
ssh ubuntu@192.168.3.26 'systemctl is-enabled edge_server'   # enabled = 开机自启
ssh ubuntu@192.168.3.26 'sudo systemctl disable edge_server' # 取消自启（下次开机不启动）
ssh ubuntu@192.168.3.26 'sudo systemctl enable edge_server'  # 恢复自启
```

`disable` 只是取消开机启动，**不会**停止当前正在运行的进程；要同时停用：

```bash
ssh ubuntu@192.168.3.26 'sudo systemctl disable --now edge_server'
```

### 2.3 日志

```bash
# 最近 100 行
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -n 100 --no-pager'

# 实时跟随（Ctrl-C 退出）
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -f'

# 本次开机以来的日志
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -b --no-pager'
```

ROS 自己的日志在 `/var/log/edge_server/ros/`（由 unit 里的 `ROS_LOG_DIR` 指定）。

### 2.4 启动后验证

> `systemctl restart` 是异步的：命令返回时节点还没绑好端口（实测约 6 秒）。
> 紧接着 `curl` 可能得到 `000`，先等就绪再验证：

```bash
for i in $(seq 1 20); do
  code=$(curl -s -m 2 -o /dev/null -w '%{http_code}' http://192.168.3.26/ || true)
  [ "$code" = "200" ] && break
  sleep 1
done
echo "web=$code"
```

然后看各接口：

```bash
curl -s -o /dev/null -w 'web=%{http_code}\n'      http://192.168.3.26/
curl -s -o /dev/null -w 'dash=%{http_code}\n'     http://192.168.3.26/dashboard.html
curl -s -o /dev/null -w 'stream=%{http_code}\n'   http://192.168.3.26:8080/stream
curl -s http://192.168.3.26/status; echo
curl -s http://192.168.3.26/api/modules/ros/health; echo
```

预期（当前没接相机时）：

- `web=200`、`dash=200`；
- `stream=503`（没有 `/camera/image_raw`，正常）；
- STM32 已接线时 `/status` 里 `"serial_connected":true`、`"encoder_ok":true`；
  没接或没上电时为 `false`；
- `"ros_status":"degraded"`：缺相机/雷达/IMU，属预期；
- 健康接口 `"ready":true`、`"healthy":false`，`reason` 会列出缺哪些设备。

`degraded` 不代表服务坏了，只代表外设没齐；`ready:true` 才是关键。

---

## 3. 整机启停（上电 / 关机 / 重启）

### 3.1 上电

1. 先接好相机、雷达、STM32 串口，再上电（避免热插拔 USB）；
2. 上电后等 **约 2～3 分钟**（实机 `systemd-analyze`：kernel 7s + userspace 2min19s，
   主要在等网络）；
3. 服务会自动启动，按 §2.4 验证。

开机后如果服务没起来：

```bash
ssh ubuntu@192.168.3.26 'systemctl is-enabled edge_server; systemctl status edge_server --no-pager'
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -b -n 100 --no-pager'
```

### 3.2 关机（推荐）

```bash
ssh ubuntu@192.168.3.26 'sudo shutdown -h now'
```

等 SSH 断开、Pi 的绿色 ACT 灯停止闪烁后再断总电源。**不要直接拔电**：ROS 日志、
`edge_params.json` 都在写 SD 卡，硬断电容易把卡写坏。

### 3.3 重启

```bash
ssh ubuntu@192.168.3.26 'sudo reboot'
```

重启后服务会随开机自启，无需手工干预。

### 3.4 连不上 Pi 时

```bash
ping 192.168.3.26
ssh -v ubuntu@192.168.3.26
```

- ping 不通：先看电源、Wi-Fi/网线；Pi 用的是 `wlan0` 的 `192.168.3.26`；
- ping 通但 SSH 拒绝：确认用户名 `ubuntu`，以及本机 `~/.ssh/id_ed25519` 是否已授权；
- 都正常但服务没起：按 §3.1 的 journalctl 排查。

### 3.5 急停

优先用**硬件断电/急停开关**。软件层面最快的是：

```bash
ssh ubuntu@192.168.3.26 'sudo systemctl stop edge_server'
```

停止时 systemd 发 `SIGINT`（`KillSignal=SIGINT`），launch 会一并结束两个节点；
`TimeoutStopSec=10`，超过 10 秒会强杀。底盘节点的 `/cmd_vel` 超时和 STM32 遥测超时
也会自动停车，但不要依赖软件兜底。

---

## 4. 手动前台运行（调试用）

服务方式看不到实时输出时，可以停服务、前台跑：

```bash
# 1) 先停服务，释放 80/8080/8090
ssh ubuntu@192.168.3.26 'sudo systemctl stop edge_server'

# 2) 前台启动（80 端口需要权限，所以用 sudo；配置必须显式指向 deploy.yaml）
ssh -t ubuntu@192.168.3.26 \
  'sudo bash -lc "export EDGE_SERVER_CONFIG=/opt/edge_server/config/deploy.yaml && \
                  source /opt/ros/humble/setup.bash && \
                  source /opt/edge_server/install/setup.bash && \
                  ros2 launch /opt/edge_server/deploy/edge_server.launch.py"'
```

- `Ctrl-C` 结束；
- 调试完记得 `sudo systemctl start edge_server` 交还给 systemd；
- 如果不想用 sudo，把 `deploy.yaml` 的 `web.port` 临时改成 8081 再跑，但别提交这个改动。

在 Pi 上直接用 zsh 时，交互查看环境用：

```zsh
source /opt/ros/humble/setup.zsh
source /opt/edge_server/install/setup.zsh
ros2 pkg prefix edge_server
```

但**启动服务仍用上面的 `sudo bash -lc`**：`sudo` 会重置 `PATH`，直接
`sudo ros2 ...` 会报 `ros2: No such file or directory`；在 `bash -lc` 里重新 source
一次最稳。

---

## 5. 全栈模式（SLAM / Nav2 / VLA / 感知）

### 5.1 和部署模式的区别

- **部署模式**（当前开机自启）：只跑 `edge_server_node` + `health_monitor_node`，
  负责网页、手柄、串口、图传、健康检查；
- **全栈模式**：`robot_bringup.launch.py` 会再启动 VLA、感知、定位、导航、速度仲裁等节点，
  **并且也会启动一个 `edge_server_node`**。

所以两者**不能同时跑**，否则 80 端口冲突。切全栈：

```bash
ssh ubuntu@192.168.3.26 'sudo systemctl stop edge_server'

ssh -t ubuntu@192.168.3.26 \
  'sudo bash -lc "source /opt/ros/humble/setup.bash && \
                  source /opt/edge_server/install/setup.bash && \
                  ros2 launch edge_server robot_bringup.launch.py"'
```

### 5.2 单独起 SLAM / Nav2

这两个 launch 不包含 `edge_server_node`，可以和部署服务并存，但需要先有驱动话题：

```bash
# 建图（需要 /scan、/odom、TF）
ros2 launch edge_server slam_stack.launch.py

# 定位 + 导航（需要地图 yaml；Nav2 速度重映射到 /nav/cmd_vel）
ros2 launch edge_server nav2_stack.launch.py map:=$HOME/rush_mog/maps/rush_mog.yaml
```

建议放在 `tmux` 里跑，断 SSH 不会把进程带走：

```bash
ssh ubuntu@192.168.3.26
tmux new -s stack
# 里面 source 环境后 ros2 launch ...
# Ctrl-B D 退出，tmux attach -t stack 回来
```

### 5.3 停止顺序

先停上层（Nav2/SLAM/搜索），再停边缘服务，最后断硬件电源：

```bash
# tmux 里 Ctrl-C；或按具体 launch 文件结束，避免误伤 systemd 的服务进程
pkill -INT -f 'nav2_stack.launch.py'
pkill -INT -f 'slam_stack.launch.py'
ssh ubuntu@192.168.3.26 'sudo systemctl stop edge_server'
```

---

## 6. 发布新版本（同步 → 构建 → 重启）

### 6.1 本地 WSL 一条龙（推荐）

```bash
cd /mnt/f/CODE/rush-mog
PI=ubuntu@192.168.3.26

rsync -avz --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  software/edge_server/ $PI:/opt/edge_server/src/edge_server/

rsync -avz --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  software/custom_msgs/ $PI:/opt/edge_server/src/custom_msgs/

ssh $PI 'bash -lc "source /opt/ros/humble/setup.bash && \
                   cd /opt/edge_server && \
                   colcon build --packages-select custom_msgs edge_server && \
                   sudo systemctl restart edge_server"'

# 验证（restart 后要等几秒，轮询到 200）
for i in $(seq 1 20); do
  code=$(curl -s -m 2 -o /dev/null -w '%{http_code}' http://192.168.3.26/ || true)
  [ "$code" = "200" ] && break
  sleep 1
done
echo "web=$code"
ssh $PI 'systemctl is-active edge_server'
```

> 关键：远端命令必须包在 `bash -lc "..."` 里。直接 `ssh $PI 'source ...'` 会用登录
> shell（zsh）执行，掉进 §0 的坑。

构建耗时：只改 `edge_server` 约 10 秒；首次/清理后构建 `custom_msgs` 约 7 分钟（Pi 4B）。

### 6.2 只改 Python / 网页

`custom_msgs` 没动时，可以只构建 `edge_server`：

```bash
ssh $PI 'bash -lc "source /opt/ros/humble/setup.bash && \
                   cd /opt/edge_server && \
                   colcon build --packages-select edge_server && \
                   sudo systemctl restart edge_server"'
```

网页文件会被装到 `install/.../share/edge_server/web`，构建后浏览器记得强制刷新。

### 6.3 只改远端运行配置

```bash
ssh ubuntu@192.168.3.26
sudoedit /opt/edge_server/config/deploy.yaml
sudo systemctl restart edge_server
```

改仓库里的 `config/robot.yaml` **不会**影响这台机器，因为 unit 用 `EDGE_SERVER_CONFIG`
指向了 `deploy.yaml`。

### 6.4 `src` 被误删 / 构建报 “package not found”

重新同步即可（`--delete` 只清理目标目录内部，不会碰 `config/`、`deploy/`、`install/`）：

```bash
cd /mnt/f/CODE/rush-mog
rsync -avz --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  software/edge_server/ ubuntu@192.168.3.26:/opt/edge_server/src/edge_server/
rsync -avz --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  software/custom_msgs/ ubuntu@192.168.3.26:/opt/edge_server/src/custom_msgs/
ssh ubuntu@192.168.3.26 'bash -lc "source /opt/ros/humble/setup.bash && cd /opt/edge_server && colcon build --packages-select custom_msgs edge_server"'
ssh ubuntu@192.168.3.26 'sudo systemctl restart edge_server'
```

### 6.5 回滚

发布前留一份源码快照（**放在 colcon 工作区之外**，否则 `colcon` 会扫到重名包）：

```bash
ssh ubuntu@192.168.3.26 '
  mkdir -p /opt/edge_server/backup &&
  rm -rf /opt/edge_server/backup/edge_server.bak &&
  cp -a /opt/edge_server/src/edge_server /opt/edge_server/backup/edge_server.bak'
```

回滚：

```bash
ssh ubuntu@192.168.3.26 '
  rsync -a --delete /opt/edge_server/backup/edge_server.bak/ /opt/edge_server/src/edge_server/ &&
  bash -lc "source /opt/ros/humble/setup.bash && cd /opt/edge_server && colcon build --packages-select edge_server" &&
  sudo systemctl restart edge_server'
```

---

## 7. 常见问题

### 7.1 `source /opt/ros/humble/setup.bash` 报找不到 `setup.sh`

**现象**

```text
/opt/ros/humble/setup.bash:.:11: no such file or directory: /opt/edge_server/setup.sh
```

**原因**：当前 shell 是 zsh，`setup.bash` 里的 `BASH_SOURCE` 为空，前缀退化成当前目录。

**处理**（三选一）：

```zsh
# A. zsh 里用对应的 zsh 版本（推荐给交互使用）
source /opt/ros/humble/setup.zsh
source /opt/edge_server/install/setup.zsh

# B. 临时进 bash 子 shell
bash
source /opt/ros/humble/setup.bash
source /opt/edge_server/install/setup.bash

# C. 把默认 shell 改回 bash（推荐给这台机器，文档/脚本都按 bash 写）
sudo chsh -s /bin/bash ubuntu
# 退出 SSH 重新登录后生效
```

不要改 `/opt/ros/humble/setup.bash` 本身。

### 7.2 其他高频问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `Package 'edge_server' not found` | `src/edge_server` 不在，或不在 `/opt/edge_server` 下执行 | 按 §6.4 重新同步；`cd /opt/edge_server` 再 build |
| `colcon: command not found` | PATH 异常，或 colcon 没装 | `command -v colcon` 确认；`sudo apt install python3-colcon-common-extensions` |
| `ros2: command not found` | 同上，或 zsh 下 source 失败 | 按 §7.1 处理 |
| 手动跑时报 `Permission denied`（80 端口） | 普通用户不能绑 <1024 | 用 `sudo`，或交给 systemd（unit 有 `CAP_NET_BIND_SERVICE`） |
| `/stream` 一直 503 | 没有 `/camera/image_raw` | `ros2 topic hz /camera/image_raw`；先起相机驱动 |
| `/status` 里 `serial_connected:false` | 串口设备没打开：没启用 `uart3`、设备名不对或被占用 | 查 `ls -l /dev/ttyAMA1`、`grep uart3 /boot/firmware/config.txt`、`journalctl -u edge_server`；确认 `ubuntu` 在 `dialout` 组 |
| 网页按钮无反应、`/cmd` 返回 `ok` | 命令被接受但串口没写出去，或 STM32 没收到 | 按 CODE_MAP.md §10 从 `/status`→`strace`→LED 逐层查 |
| 按住按钮时 STM32 LED 连闪 | 服务端在周期性重发（旧行为），每条命令都会翻转 LED | 新版默认已关闭；确认 `edge.motion.resend_manual` 为 `false` 并重启服务 |
| `/api/speaker` 为 `error` / `br-connection-page-timeout` | 音箱休眠、被手机占用或超出距离 | 音箱开机并断开手机；节点每 20s 自动重连，或 `curl -X POST .../api/speaker/connect` |
| `/api/speaker` 已连接但 `audio_ready=false` | PipeWire/WirePlumber 用户服务没起，或 sink 未创建 | `systemctl --user status pipewire wireplumber`；`pactl list short sinks | grep bluez_output`；见 DEPLOY.md §7.3 |
| 语音播报入队但没声音 | 默认 sink 不是蓝牙，或音箱静音/音量 0 | `pactl get-default-sink`、`pactl set-sink-mute <name> 0`、`POST /api/speaker/volume` |
| `/api/microphone` 的 `supported=false` | 耳机 card 没有 HFP/HSP profile，或音箱未连接 | `pactl list cards \| grep -A12 bluez_card`；确认 `headset-head-unit` 存在 |
| 录音报“未找到蓝牙麦克风输入源” | HFP profile 切换失败/被手机占用 | `bluetoothctl disconnect/connect`；`pactl list short sources \| grep bluez_input`；再重试 |
| 录音没声音/文件很小 | 麦克风静音、音量低或距离远 | `pactl list sources` 看 `Mute`/`Volume`；`POST /api/microphone/volume` 调大 |
| `/api/power` 为 `error` / `undervoltage=true` | 5V 供电不足、线缆压降或电源转换板异常 | 检查 12V→5V 3A 输出、USB-C 线、接线端子；`vcgencmd get_throttled`、`dmesg \| grep -i undervolt` |
| `/api/power` 为 `degraded` | 当前降频/温度限制，不一定是电源 | 看 `warnings`、`temperature_c`；先降温/降负载再复测 |
| 网页 200 但还是旧界面 | 没 build/restart，或浏览器缓存 | 重新 build + restart，浏览器 Ctrl-F5 |
| 服务反复重启 | 配置 YAML 写错/端口被占 | `journalctl -u edge_server -n 100` |
| 日志时间比北京时间慢 8 小时 | Pi 时区是 UTC | `sudo timedatectl set-timezone Asia/Shanghai`（可选） |
| 网页/接口全部超时 | 服务没起或 Pi 掉线 | `ping`、`systemctl status`、`journalctl -b` |

---

## 8. 路径 / 端口 / 配置速查

| 项 | 值 |
|---|---|
| 源码（Pi） | `/opt/edge_server/src/edge_server`、`/opt/edge_server/src/custom_msgs` |
| 构建产物 | `/opt/edge_server/install/`（不要手改） |
| 运行配置 | `/opt/edge_server/config/deploy.yaml`（`EDGE_SERVER_CONFIG`） |
| 部署 launch | `/opt/edge_server/deploy/edge_server.launch.py` |
| systemd unit | `/etc/systemd/system/edge_server.service` |
| 应用日志 | `journalctl -u edge_server` |
| ROS 日志 | `/var/log/edge_server/ros/` |
| ROS home | `/var/lib/edge_server/ros/` |
| 状态文件 | `/var/lib/edge_server/edge_params.json` |
| 音频目录 | `/var/lib/edge_server/audio`（`/api/speaker/play` 只允许此目录） |
| 录音目录 | `/var/lib/edge_server/audio/mic` |
| Web / 手柄 | `http://192.168.3.26/`（:80） |
| Dashboard | `http://192.168.3.26/dashboard.html` |
| 音箱状态 | `http://192.168.3.26/api/speaker` |
| 麦克风状态 | `http://192.168.3.26/api/microphone` |
| 电源状态 | `http://192.168.3.26/api/power` |
| MJPEG | `http://192.168.3.26:8080/stream` |
| 健康网关 | `http://127.0.0.1:8090/ros/health`（仅本机） |
| 串口 | `/dev/ttyAMA1`（GPIO4/5，UART3），115200 8N1 |

### GPIO 串口接线（UART3）

这台 Pi 通过 40Pin 的 GPIO4/5 连 STM32 USART3，不是 USB 串口：

| Pi 引脚 | Pi 功能 | STM32 | 说明 |
|---|---|---|---|
| GPIO4（物理 7） | TXD3 | PB11（USART3_RX） | Pi 发 → STM32 收 |
| GPIO5（物理 29） | RXD3 | PB10（USART3_TX） | STM32 发 → Pi 收 |
| GND（物理 6/9/14…） | GND | GND | **必须共地** |

- 双方都是 3.3V 电平；PB10/PB11 是 5V 容忍脚，不需要电平转换，**不要接 5V**；
- Pi 侧设备名是 `/dev/ttyAMA1`，由 `dtoverlay=uart3` 生成；
  `enable_uart=1` 只启用 GPIO14/15 的 UART0，**不能**代替它；
- 当前 `/boot/firmware/config.txt` 末尾已有：

  ```ini
  dtoverlay=uart3
  ```

  没生效时检查该行是否存在，改完必须重启（`sudo reboot`）。

对应运行配置（`/opt/edge_server/config/deploy.yaml`）：

```yaml
base:
  port: "/dev/ttyAMA1"
  baud: 115200
```

改完重启并确认：

```bash
ssh ubuntu@192.168.3.26 'sudo systemctl restart edge_server'
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -n 20 --no-pager | grep serial'
curl -s http://192.168.3.26/status | grep -o '"serial_connected":[a-z]*'
```

正常日志：`STM32 serial connected: /dev/ttyAMA1@115200`。

收不到遥测时按顺序查：

1. `ls -l /dev/ttyAMA1` 是否存在（不存在 = overlay 没生效/没重启）；
2. 波特率必须与固件 `BODY_UART_BAUD_RATE` 一致（当前 115200），
   在 9600 下读到的会是乱码；
3. TX/RX 是否交叉（Pi TX 接 STM32 RX），是否共地；
4. 固件是否已烧录当前版本（`software/body_control`，115200、`$B` 遥测）。
