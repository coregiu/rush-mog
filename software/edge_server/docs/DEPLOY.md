# edge_server 部署与运维手册

> **文档说明**：本文是 Edge Server 的部署与运维唯一手册，覆盖装机、发布、启停、外设接入和排障。内容合并自原 `DEPLOY.md` 与 `OPERATIONS.md`（含其第 9 节“蓝牙手动连接”），并吸收了 `CODE_MAP.md` 中属于运维的“分层排查地图”和“最小验证命令”。设计与代码结构见设计文档，健康检查字段见健康检查契约，机械/电气分析见硬件文档。
>
> 相关文档：设计文档 [`DESIGN.md`](DESIGN.md) ｜ 健康检查契约 [`HEALTHCHECK.md`](HEALTHCHECK.md) ｜ 硬件分析 [`../../../hardware/SLAM_NAV.md`](../../../hardware/SLAM_NAV.md)
>
> **2026-11-21 更新（本次重新整理）**：§2 重写为「从空机器到开机自启」的 P0–P7 手动流程，新增 §2.0 目标机变量表、§2.5 配置裁剪清单、§2.6 单元全文与字段说明、§2.7 验收对照与打勾清单；部署 launch 版本化为 `launch/edge_server_deploy.launch.py`（随 `colcon build` 安装，不再手工复制）；systemd 单元版本化为 `software/tools/systemd/edge_server.service`；§7.5 补充单元残缺/端口权限/运行目录/linger 等新故障；实测环境为 x86_64 开发机（`eagle@172.31.4.88`），树莓派路径保持原样。

## 目录
- 0. 速查与三条铁律
- 1. 系统组成与启动流程
- 2. 安装与首次部署
- 3. 日常启停与验证
- 4. 发布新版本
- 5. 运行模式
- 6. 外设接入
- 7. 排障
- 8. 速查表

## 0. 速查与三条铁律
### 0.1 目标机器
```text
实车      ubuntu@192.168.3.26（Raspberry Pi 4B，Ubuntu 22.04 arm64）
开发机    eagle@172.31.4.88（x86_64，Ubuntu 22.04.5；2026-11-21 在此完成整套部署验证）
部署目录  /opt/edge_server
服务      edge_server.service（开机自启，User 见 §2.0）
运行配置  /opt/edge_server/config/deploy.yaml（EDGE_SERVER_CONFIG）
串口      /dev/ttyAMA1（GPIO4/5 → UART3），115200 8N1（仅实车有）
```
仓库根目录在开发机上是 `/home/eagle/workspace/projects/rush-mog`；Windows/WSL 环境对应
`F:\CODE\rush-mog` 与 `/mnt/f/CODE/rush-mog`。入口地址、路径和端口统一列在 §8.1；AP 模式下手机连
`RUSH-MOG` Wi-Fi 后访问 `http://192.168.3.26/`。

### 0.2 一条命令重启
```bash
PI=ubuntu@192.168.3.26
ssh $PI 'sudo systemctl restart edge_server'
```
`restart` 是异步的，就绪轮询见 §3.3。改了代码不能只 restart，必须先 `colcon build`（§4）。

### 0.3 三条铁律
1. **systemd 拥有应用。** `edge_server.service` 是唯一入口；手工 `nohup` 或前台进程会抢 80/8080/8090。前台调试前先 `stop`，调试完交还 systemd。
2. **同步 ≠ 已部署。** 本地 `git commit` 不会到 Pi；发布 = 同步源码 → 远端 `colcon build` → `restart` → 验证。远端 `src` 只是发布快照，不是 git 工作区，也不是软链接。
3. **远端登录 shell 是 zsh。** 在 zsh 里 source `setup.bash` 会算错前缀（§7.1）。远程命令一律 `ssh ... 'bash -lc "..."'`；交互式查看环境用 `setup.zsh`。

## 1. 系统组成与启动流程
### 1.1 上电启动链路
```text
上电
 └─ systemd（multi-user.target）
     └─ edge_server.service（User=ubuntu，开机自启）
         └─ /bin/bash -lc 'source /opt/ros/humble/setup.bash
                          && source /opt/edge_server/install/setup.bash
                          && exec ros2 launch edge_server edge_server_deploy.launch.py'
             ├─ edge_server_node    → Web :80、MJPEG :8080、串口 /dev/ttyAMA1（GPIO4/5 UART3）
             └─ health_monitor_node → ROS 健康网关 127.0.0.1:8090
```
- 服务里写死了 `bash -lc`，所以**登录 shell 是 zsh 也不影响服务启动**；受影响的只是手工 SSH 命令。
- 部署 launch 是仓库里的 `launch/edge_server_deploy.launch.py`，随 `colcon build` 装进
  `install/.../share/edge_server/launch/`，**不再是手工复制的外挂文件**；它只启动 Edge Server +
  健康网关，不启动 SLAM/Nav2/感知全栈（全栈见 §5）。
- 运行配置由 `EDGE_SERVER_CONFIG=/opt/edge_server/config/deploy.yaml` 指定，优先级高于 `install/.../config/robot.yaml`。

| 层 | 内容 | 启停方式 |
|---|---|---|
| 整机 | Pi 电源、系统 | 上电 / `sudo shutdown -h now` |
| 应用 | `edge_server.service` | `systemctl start\|stop\|restart` |
| ROS 全栈（可选） | SLAM / Nav2 / VLA / 感知 | 手工 `ros2 launch`（§5） |
| 上位机 | `software/center_server`（PC 侧） | 与 Pi 独立，不在本文范围 |

### 1.2 目录职责
| 路径 | 职责 | 是否随 rsync 更新 |
|---|---|---|
| `/opt/edge_server/src/edge_server` | `software/edge_server` 的源码副本 | 是 |
| `/opt/edge_server/src/custom_msgs` | `software/custom_msgs` 的源码副本 | 是 |
| `/opt/edge_server/config/deploy.yaml` | 远程专用运行配置（`EDGE_SERVER_CONFIG`） | 否 |
| `/opt/edge_server/install/` | 构建产物，**不要手改**；部署 launch 在 `share/edge_server/launch/edge_server_deploy.launch.py` | 由 colcon 生成 |
| `/opt/edge_server/build/`、`log/` | 构建中间物、构建日志 | 由 colcon 生成 |
| `/etc/systemd/system/edge_server.service` | systemd 单元（仓库模板 `software/tools/systemd/edge_server.service`） | 否，单独安装 |
| `/var/lib/edge_server/ros` | `ROS_HOME`（节点级配置/缓存） | 否，需手工建（§2.6） |
| `/var/lib/edge_server/edge_params.json` | 网页/接口保存的运行参数 | 运行时生成 |
| `/var/lib/edge_server/audio` | 音箱播放素材与录音（`/api/speaker/play` 只允许此目录） | 否，需手工建 |
| `/var/log/edge_server/ros` | `ROS_LOG_DIR`，ROS 自己的日志 | 否，需手工建 |

配置优先级（`edge_server/utils/config.py`）：
```text
ROS 参数 config_file  >  环境变量 EDGE_SERVER_CONFIG  >  install/.../config/config.yaml
```
仓库里的 `edge_server/config/robot.yaml` 是版本管理的实车默认值；部署机以 `deploy.yaml` 为准，改
`robot.yaml` 不会影响已部署机器。仓库里的 `launch/edge_server.launch.py` 是全量联调启动文件，
服务用的是 `launch/edge_server_deploy.launch.py`（只两个节点）。

### 1.3 本地开发目录 → 部署目录
```text
本地开发机（源码，Git 管理）：<repo>/software/edge_server
（开发机 ~/workspace/projects/rush-mog；Windows 侧 F:\CODE\rush-mog；WSL /mnt/f/CODE/rush-mog）
│  rsync / tar 复制（手动发布，不是自动同步，§2.4）
▼
<目标机>:/opt/edge_server/src/edge_server（源码副本，非 git 工作区，非软链接）
│  colcon build
▼
/opt/edge_server/install/edge_server（运行产物：Python 包、launch、config、web）
│  systemd 启动（ros2 launch edge_server edge_server_deploy.launch.py）
▼
edge_server_node + health_monitor_node
```
本地是唯一开发源；远端 `src` 只是发布快照，改本地不会自动影响远端，也不能“热更新”。
`install/` 是构建产物，重新 build 会覆盖。发布流程见 §4。

## 2. 安装与首次部署

> 本章是**从一台空机器到服务开机自启**的完整手动流程：P0 定变量 → P1 系统源 → P2 ROS → P3 外设
> → P4 代码与构建 → P5 运行配置 → P6 systemd → P7 验收。每一步都有「验收门槛」，不通过不要往下走；
> 出错先查 §7.5。命令都可直接粘贴，只把 `$HOST`、`$USER` 等换成 §2.0 表里的值。
>
> 预计耗时：x86_64 有 ROS 环境约 15 分钟；树莓派从裸系统开始约 60–90 分钟（`custom_msgs` 首次编译约 7 分钟）。

### 2.0 目标机变量与前置检查（P0）

先把这张表填好，后文所有命令都用它：

| 变量 | 实车（树莓派 4B） | 开发机（2026-11-21 实测） | 用途 |
|---|---|---|---|
| `$HOST` | `ubuntu@192.168.3.26` | `eagle@172.31.4.88` | ssh / rsync 目标 |
| `$USER` / UID | `ubuntu` / `1000` | `eagle` / `1000` | systemd `User=`、`/run/user/<uid>` |
| 架构 / 系统 | aarch64 / Ubuntu 22.04 | x86_64 / Ubuntu 22.04.5 | 只影响 apt 源与构建耗时 |
| `$DEPLOY` | `/opt/edge_server` | `/opt/edge_server` | colcon 工作区 |
| Web / MJPEG | `80` / `8080` | `80` / `8080` | 80 需要 `CAP_NET_BIND_SERVICE` |
| 健康网关 | `127.0.0.1:8090` | `127.0.0.1:8090` | 仅本机可访问 |
| 硬件 | STM32(UART3)、D6 雷达(UART4)、Astra Pro(USB)、ICM-20948(I2C1)、NewMine 音箱 | 无 | 决定 §2.5 的裁剪项 |

```bash
HOST=ubuntu@192.168.3.26      # 按上表改
USER_NAME=ubuntu              # 按上表改
```

**前置检查**（四条全过再开始）：

```bash
# 1) 能登录、有 sudo、能联网
ssh $HOST 'id && (sudo -n true 2>/dev/null && echo sudo-ok || echo sudo-needs-password)'

# 2) 系统与架构：期望 Ubuntu 22.04.x + aarch64(实车) / x86_64(开发机)
ssh $HOST '. /etc/os-release && echo "$PRETTY_NAME" && dpkg --print-architecture'

# 3) 磁盘：/opt 与 /var 至少留 8G（build + install + 日志）
ssh $HOST 'df -h /opt /var | tail -n +2'

# 4) 是否已有 ROS（有则 §2.2 只做验收，不重装）
ssh $HOST 'ls -d /opt/ros/humble 2>/dev/null || echo no-ros'
```

**本次部署的实际起点**：开发机已预装 ROS Humble + colcon，`/opt/edge_server` 里有一轮上一会话留下的
半成品——`config/deploy.yaml` 存在、`colcon build` 能过，但 `/etc/systemd/system/edge_server.service`
被写成了**只有 5 行 `Environment=` 的残文件**（没有 `[Unit]`/`[Service]`/`ExecStart`），所以
`systemctl start` 报 `Unit ... has a bad unit file setting`、`is-enabled` 是 `static`。
本次从 P4 重新同步、重建，P6 重写单元后拉起。**这正是"部署到一半"最典型的样子**，下面的流程能从头复现。

**各阶段是否需要重做，一眼对照：**
| 阶段 | 裸树莓派 | 本次开发机 | 说明 |
|---|---|---|---|
| P1 apt 源 | 要做（含 ubuntu-ports） | 已就绪 | 见 §2.1 |
| P2 ROS 2 | 要装 | 已装 `/opt/ros/humble` | 见 §2.2 |
| P3 外设 | 要做 | 全部跳过 | 无 UART/I2C/音箱，见 §2.3 |
| P4 代码+构建 | 要做 | 重做（同步 + 增量构建 1 s） | 见 §2.4 |
| P5 运行配置 | 要做 | 重做（7 项裁剪） | 见 §2.5 |
| P6 systemd | 要做 | **重写单元的残文件** | 见 §2.6 |
| P7 验收 | 要做 | 通过（degraded） | 见 §2.7 |

### 2.1 系统与 apt 源（P1）

**1) 修复 arm64 apt 源（只对树莓派做）**

Ubuntu 的 arm64 包在 `ubuntu-ports`，不在 `ubuntu`；源写错会满屏 `404 Not Found`。

```bash
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak.rushmog
sudo sed -i 's|mirrors.aliyun.com/ubuntu/|mirrors.aliyun.com/ubuntu-ports/|g' /etc/apt/sources.list
sudo sed -i '/^deb-src/s/^/#/' /etc/apt/sources.list     # 注释 deb-src，少下一半索引
grep -c 'ubuntu-ports' /etc/apt/sources.list             # 期望 ≥1（通常 2–4 行）
```

> x86_64 开发机**不要**改成 `ubuntu-ports`，保持 `ubuntu`。

**2) 加 ROS 2 apt 源（USTC 镜像，两台机器都要）**

```bash
sudo apt update
sudo apt install -y curl gnupg2 ca-certificates software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.ustc.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

**门槛**
```bash
apt-cache policy ros-humble-ros-base | head -3     # 必须有 Candidate: 行
```

> `raw.githubusercontent.com` 国内常不可达：挂代理，或从任意可访问镜像取内容一致的 `ros.key`
> （该文件是二进制 keyring，直接 `curl -o` 落盘即可，不要 `gpg --dearmor` 二次处理）。

### 2.2 ROS 2 Humble 与构建依赖（P2）

**1) 必需包**

```bash
sudo apt-get install -y \
  ros-humble-ros-base ros-humble-cv-bridge ros-humble-tf2-ros \
  ros-humble-message-filters ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime ros-humble-launch-ros \
  python3-colcon-common-extensions python3-pytest \
  python3-opencv python3-numpy python3-yaml python3-serial \
  build-essential cmake
```

**2) 可选包**（只装要用的，不装不影响部署模式启动）

```bash
sudo apt-get install -y ros-humble-slam-toolbox ros-humble-robot-localization ros-humble-navigation2
```

| 不装的后果 | 影响面 |
|---|---|
| `slam-toolbox` | `slam_stack.launch.py` 起不来，2D 建图不可用 |
| `robot-localization` | `ekf.launch.py` 起不来（默认不启动，不影响部署模式） |
| `navigation2` | `navigation_node` 的 `backend: auto` 回退内置直航，其余不受影响 |

**3) 交互式环境变量**（systemd 不依赖它，自己会 source，见 §2.6）

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc     # bash
echo 'source /opt/ros/humble/setup.zsh'  >> ~/.zshrc      # zsh（树莓派默认 zsh）
```

**门槛**
```bash
source /opt/ros/humble/setup.bash
ros2 --help > /dev/null && echo ros2-ok
command -v colcon
python3 -c 'import cv2, numpy, yaml, serial; print("py-deps-ok")'
```

> **zsh 坑**（§7.1）：zsh 里 `source .../setup.bash` 会把前缀算错，报
> `no such file or directory: /opt/edge_server/setup.sh`。交互用 `setup.zsh`，
> 非交互命令一律 `bash -lc "..."`。

### 2.3 系统级外设配置（P3）

> 这一节全是**仓库之外**的系统配置（`/boot/firmware/config.txt`、udev、用户级音频服务）。
> 重装系统必须重做；不做也不影响服务启动，只是对应设备在健康页一直是 `unknown`。
> 无硬件的开发机可整节跳过。

**1) UART3 —— STM32 串口（实车必做）**

```bash
echo 'dtoverlay=uart3' | sudo tee -a /boot/firmware/config.txt
sudo usermod -aG dialout $USER_NAME        # 读写串口权限，重新登录后生效
sudo reboot
```

重启后验收：
```bash
ls -l /dev/ttyAMA1                     # 期望 crw-rw---- root dialout
id -nG | grep -o dialout               # 期望 dialout
grep uart3 /boot/firmware/config.txt
```

`enable_uart=1` 只启用 GPIO14/15 的 UART0，**不能代替** `dtoverlay=uart3`（踩坑记录见 §7.2）。

**2) UART4 —— D6/LD19 激光雷达（接雷达再做）**

```bash
sudo sed -i 's|^dtparam=spi=on|# dtparam=spi=on|' /boot/firmware/config.txt   # GPIO8/9 与 SPI0 复用
echo 'dtoverlay=uart4' | sudo tee -a /boot/firmware/config.txt
sudo reboot

# udev 规则：目标机上通常没有仓库，从开发机（仓库根目录）推过去
scp software/tools/systemd/99-rush-mog-lidar.rules $HOST:/tmp/
ssh $HOST 'sudo install -m 644 /tmp/99-rush-mog-lidar.rules /etc/udev/rules.d/ && \
           sudo udevadm control --reload-rules && sudo udevadm trigger'
```

验收：
```bash
ls -l /dev/rush_lidar                  # 期望 -> ttyAMA2
sudo dmesg | grep fe201800             # 期望出现 ttyAMA2
```

> udev 规则按 platform device（`fe201800.serial`）匹配，不要写死 `ttyAMA*` 编号——重新插拔后编号会变。

**3) I2C —— ICM-20948**

```bash
sudo apt install -y i2c-tools python3-smbus2
grep -q '^dtparam=i2c_arm=on' /boot/firmware/config.txt || \
  echo 'dtparam=i2c_arm=on' | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

验收：
```bash
ls -l /dev/i2c-1
sudo i2cdetect -y 1                    # 期望 0x68（SA0 接地）或 0x69
```

**4) 蓝牙音频 + 中文 TTS**

```bash
sudo apt-get install -y pipewire pipewire-pulse wireplumber \
  libspa-0.2-bluetooth pulseaudio-utils alsa-utils ffmpeg espeak-ng
sudo python3 -m pip install edge-tts
sudo loginctl enable-linger $USER_NAME      # 不登录也保留用户级 PipeWire（systemd 里 XDG_RUNTIME_DIR 依赖它）
```

验收：
```bash
systemctl --user is-active pipewire pipewire-pulse wireplumber   # 三个都 active
command -v parec paplay espeak-ng ffmpeg
python3 -c 'import edge_tts; print("edge-tts-ok")'
```

首次配对音箱见 §6.1（`bluetoothctl pair/trust/connect`）；配对一次后由 `edge_server_node`
按 `edge.speaker.connect_interval_sec`（默认 20 s）自动重连。

### 2.4 代码落盘与构建（P4）

**1) 首次创建部署目录**（只有第一次需要）

```bash
ssh $HOST 'sudo mkdir -p /opt/edge_server/src && sudo chown -R '"$USER_NAME"':'"$USER_NAME"' /opt/edge_server'
```

**2) 同步源码**（开发机 → 目标机，两个包都要）

```bash
cd /path/to/rush-mog                  # 仓库根目录（本项目当前为 ~/workspace/projects/rush-mog）
rsync -avz --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  software/edge_server/ $HOST:/opt/edge_server/src/edge_server/
rsync -avz --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  software/custom_msgs/ $HOST:/opt/edge_server/src/custom_msgs/
```

没有 rsync / 网络只通单跳时的等价做法（U 盘也行）：

```bash
tar czf edge_src.tgz -C software edge_server custom_msgs
scp edge_src.tgz $HOST:/tmp/
ssh $HOST 'sudo tar xzf /tmp/edge_src.tgz -C /opt/edge_server/src && sudo chown -R '"$USER_NAME"':'"$USER_NAME"' /opt/edge_server/src'
```

验收：
```bash
ssh $HOST 'ls /opt/edge_server/src/edge_server/launch/edge_server_deploy.launch.py \
              /opt/edge_server/src/custom_msgs/package.xml'
```

> `src/` 是发布快照：不是 git 工作区、不是软链接，本地提交不会自己过去（铁律 2，§0.3）。
> `--delete` 只清理这两个目录**内部**，不会碰 `config/`、`install/`、`build/`。

**3) 构建**

```bash
ssh $HOST 'bash -lc "source /opt/ros/humble/setup.bash && cd /opt/edge_server && \
  colcon build --packages-select custom_msgs edge_server"'
```

| 场景 | 耗时（参考） |
|---|---|
| 首次全量（含 Pi 4B 编 `custom_msgs`） | 约 7 分钟 |
| 只改 Python（`--packages-select edge_server`） | 1–10 秒 |

改了 `package.xml`、`setup.py`、新增 launch 时**必须**重新 `colcon build`；只改 `config/deploy.yaml` 不用。

验收：
```bash
ssh $HOST 'bash -lc "source /opt/ros/humble/setup.bash && source /opt/edge_server/install/setup.bash && \
  ros2 pkg prefix edge_server && \
  ros2 launch edge_server edge_server_deploy.launch.py --show-args"'
# 期望：打印 .../install/edge_server，以及 config_file 参数说明
```

### 2.5 运行配置 deploy.yaml（P5）

远程运行配置**不在仓库里**：由 `robot.yaml` 复制后按环境裁剪，`EDGE_SERVER_CONFIG` 指向它，
优先级高于 `install/.../config/robot.yaml`（§1.2）。改完只需重启服务，不用重新构建。

**1) 复制**

```bash
cd /opt/edge_server
mkdir -p config
cp src/edge_server/edge_server/config/robot.yaml config/deploy.yaml
```

**2) 按环境裁剪**（在 `/opt/edge_server/config` 下执行）

| 场景 | 命令 | 不改的后果 |
|---|---|---|
| 所有环境：状态文件进 `/var/lib` | `sed -i 's#^    state_file: .*#    state_file: "/var/lib/edge_server/edge_params.json"#' deploy.yaml` | 参数写进 `~/.config/...`，换运行用户后丢失 |
| 实车 UART3 串口 | `sed -i 's#^    port: "/dev/ttyUSB0".*#    port: "/dev/ttyAMA1"#' deploy.yaml` | 串口打不开，`serial_connected=false` |
| 有蓝牙音箱 | `sed -i 's#^      audio_dir: .*#      audio_dir: "/var/lib/edge_server/audio"#' deploy.yaml` | 素材/录音散落到 `~/.local/share`，systemd 下可能不可写 |
| 无蓝牙音箱 | `sed -i '/^    speaker:/,/^    power:/ s/^      enabled: true/      enabled: false/' deploy.yaml`<br>`sed -i '/^      microphone:/,/^      tts:/ s/^        enabled: true/        enabled: false/' deploy.yaml` | 每 20 s 重连失败，日志刷屏 |
| 非树莓派（无 vcgencmd） | `sed -i '/^    power:/,/^    ros_health_url:/ s/^      enabled: true/      enabled: false/' deploy.yaml` | `vcgencmd: not found`，电源模块恒 error |
| 部署模式（只跑两个节点） | `sed -i 's/^    check_nodes: true/    check_nodes: false/; s/^    safety_required: true/    safety_required: false/' deploy.yaml` | 15 个未启动节点让整体健康恒为 `error` |
| 无硬件但要显示 `degraded` | 见下方 awk | 缺相机/雷达/IMU 直接把整体打成 `error` |

```bash
# 未接入的硬件标为可选：camera / lidar / imu / base / odom
awk '/^      [a-z_]+:$/ {dev=$1; sub(":","",dev)}
     /^        required: true/ {if (dev ~ /^(camera|lidar|imu|base|odom)$/) sub("required: true","required: false")}
     {print}' deploy.yaml > deploy.yaml.new && mv deploy.yaml.new deploy.yaml
```

**为什么要同时动 `required` 和 `strict_unknown`**（`edge_server/core/health_monitor.py` 的判定）：

| `required` | `strict_unknown` | 无数据时的整体状态 | 适用 |
|---|---|---|---|
| `true` | 任意 | `error` | 实车硬件齐全：缺设备必须报警 |
| `false` | `true` | `degraded` | 硬件未接的开发机/半成品车（本次采用） |
| `false` | `false` | `ok` | 慎用：什么都没接也显示正常 |

**3) 门槛：YAML 能被解析且关键值正确**

```bash
python3 - <<'PY'
import yaml
c = yaml.safe_load(open('/opt/edge_server/config/deploy.yaml'))['edge_server']
e = c['edge']
print('web      ', e['web']['port'], e['web']['stream_port'])
print('base     ', c['base']['port'], c['base']['baud'])
print('state    ', e['state_file'])
print('speaker  ', e['speaker']['enabled'], 'mic', e['speaker']['microphone']['enabled'])
print('power    ', e['power']['enabled'])
print('health   ', c['health']['check_nodes'], c['health']['safety_required'], c['health']['strict_unknown'])
print('required ', {k: v.get('required') for k, v in c['health']['devices'].items() if 'required' in v})
PY
```

期望（本次开发机）：`web 80 8080`、`base /dev/ttyAMA1 115200`、
`state /var/lib/edge_server/edge_params.json`、`speaker False mic False power False`、
`health False False True`。

> 接好硬件后建议把 `camera/lidar/imu/base/odom` 的 `required` 改回 `true`：缺设备直接 `error`
> 比 `degraded` 更早暴露问题。

### 2.6 systemd 单元与首次启动（P6）

单元已版本化：仓库 `software/tools/systemd/edge_server.service` → 目标机
`/etc/systemd/system/edge_server.service`。

**1) 创建运行目录并交给运行用户**

```bash
sudo install -d -o $USER_NAME -g $USER_NAME -m 755 \
  /var/lib/edge_server /var/lib/edge_server/ros \
  /var/lib/edge_server/audio /var/lib/edge_server/audio/mic \
  /var/log/edge_server /var/log/edge_server/ros
```

> 漏建的典型症状：`ROS_LOG_DIR` 不可写导致节点起不来；`/api/speaker/play` 返回 404。

**2) 安装单元**（用户不是 `ubuntu` 就改 `User=`；UID 不是 1000 还要改 `/run/user/1000` 与 `user@1000.service`）

目标机上已经有仓库时（本次开发机就是这种）：

```bash
sed 's/^User=.*/User=eagle/' software/tools/systemd/edge_server.service | \
  sudo tee /etc/systemd/system/edge_server.service > /dev/null
sudo chmod 644 /etc/systemd/system/edge_server.service
```

目标机上没有仓库时（实车常态），在开发机仓库根目录执行：

```bash
sed 's/^User=.*/User=ubuntu/' software/tools/systemd/edge_server.service > /tmp/edge_server.service
scp /tmp/edge_server.service $HOST:/tmp/
ssh $HOST 'sudo install -m 644 /tmp/edge_server.service /etc/systemd/system/edge_server.service'
```

**3) 启用并启动**

```bash
sudo loginctl enable-linger $USER_NAME      # 轻量/无图形登录时也要有 /run/user/1000
sudo systemctl daemon-reload
sudo systemctl enable --now edge_server
```

**4) 门槛**

```bash
systemctl is-active edge_server        # active
systemctl is-enabled edge_server       # enabled
ss -ltn | grep -E ':80 |:8080 |:8090 ' # 三条都在监听
sudo journalctl -u edge_server -n 30 --no-pager | grep -E 'Loaded edge_server config|started'
# 期望：Loaded edge_server config: /opt/edge_server/config/deploy.yaml
#       edge_server_node started: web={'bound': True ...}, mjpeg={'bound': True ...}
```

**5) 单元全文**（与仓库模板一致；改了模板就重新安装一次）

```ini
[Unit]
Description=Rush-MoG Edge Server (web :80, MJPEG :8080, ROS health :8090)
Documentation=file:///opt/edge_server/src/edge_server/docs/DEPLOY.md
After=network-online.target bluetooth.service user@1000.service
Wants=network-online.target bluetooth.service

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/opt/edge_server
# 运行配置与运行目录：部署期唯一真相来源
Environment=EDGE_SERVER_CONFIG=/opt/edge_server/config/deploy.yaml
Environment=ROS_HOME=/var/lib/edge_server/ros
Environment=ROS_LOG_DIR=/var/log/edge_server/ros
# 用户级 PipeWire/PulseAudio 与 D-Bus（蓝牙音箱播放/录音必需）
Environment=XDG_RUNTIME_DIR=/run/user/1000
Environment=PULSE_SERVER=unix:/run/user/1000/pulse/native
Environment=DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
# 绑定 80 端口而不需要 root 运行
AmbientCapabilities=CAP_NET_BIND_SERVICE
CapabilityBoundingSet=CAP_NET_BIND_SERVICE
# SIGINT 让 launch 正常收尾（两个节点一起退出）；超时强杀
KillSignal=SIGINT
TimeoutStopSec=10
Restart=on-failure
RestartSec=5
# 服务里写死 bash -lc：登录 shell 是 zsh 时 setup.bash 会算错前缀（§7.1）
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /opt/edge_server/install/setup.bash && exec ros2 launch edge_server edge_server_deploy.launch.py'

[Install]
WantedBy=multi-user.target
```

各字段的作用与"为什么这么写"：

| 字段 | 作用 | 去掉会怎样 |
|---|---|---|
| `User=` | 用普通用户跑，不用 root | 需 root，权限过大 |
| `AmbientCapabilities=CAP_NET_BIND_SERVICE` | 非 root 绑 80 | `Permission denied`，web 起不来 |
| `ROS_HOME` / `ROS_LOG_DIR` | 构建与运行日志固定落 `/var` | 日志散落 `~/.ros`，`sudo` 下会写不进 |
| `XDG_RUNTIME_DIR` / `PULSE_SERVER` / `DBUS_SESSION_BUS_ADDRESS` | 连到用户级 PipeWire 与 D-Bus | 蓝牙播放/录音报连接失败 |
| `KillSignal=SIGINT` + `TimeoutStopSec=10` | launch 正常收尾，10 s 后强杀（§3.6） | 默认 SIGTERM，节点可能不清理 |
| `Restart=on-failure` + `RestartSec=5` | 崩溃自动拉起（配置写错会连挂 5 次后被 systemd 停下，看 `journalctl` 即可） | 崩了就一直是 failed |
| `ExecStart` 里 `bash -lc` + 两个 `source` | 与服务登录 shell 无关，环境确定 | zsh 下算错前缀，`ros2: command not found` |
| `After=/Wants=network-online.target bluetooth.service` | 等网络与蓝牙就绪 | 开机抢跑，重连要等下一轮 |

### 2.7 部署结果对照与从零检查清单（P7）

**两条档位的验收差异**

| 检查项 | 实车（硬件齐全） | 开发机（无硬件，本次实测） |
|---|---|---|
| `curl -o /dev/null -w '%{http_code}' /` | 200 | 200 |
| `/dashboard.html` | 200 | 200 |
| `:8080/stream` | 200（相机在跑） | 503（无 `/camera/color/image_raw`，正常） |
| `/status` → `serial_connected` | true | false（无串口） |
| `/status` → `ros_status` | ok / degraded | degraded |
| `/api/modules/ros/health` → `ready` | true | true |
| 同上 → `healthy` | 全绿时 true | false |
| 日志 `Loaded edge_server config` | 有 | 有 |

**本次实测记录（2026-11-21 20:40，eagle-desktop / x86_64 / Ubuntu 22.04.5 / ROS Humble）**

```text
systemctl is-active   → active          systemctl is-enabled → enabled
ss -ltn               → 0.0.0.0:80   0.0.0.0:8080   127.0.0.1:8090
restart → web 200     → 1.6 s（Pi 实测约 6 s，§3.3）
/ 200 | /dashboard.html 200 | :8080/stream 503 | :8090/ros/health 200
/status   → serial_connected=false, encoder_ok=false, ros_status=degraded, ros_ready=true
health    → ready=true, healthy=false, status=degraded
            summary: total=12 enabled=11 ok=1 degraded=2 unknown=8 disabled=1
            camera/depth_camera/lidar/imu/arm/battery=unknown, base/cliff=degraded, odom=ok, power=disabled
stop      → 端口全部释放，is-active=inactive；start 后 3 个端口恢复
单元与模板 → diff 仅 User= 一行不同
```

**从零部署检查清单**（复制到记事本，逐条打勾）

```text
[ ] P0 能 ssh、有 sudo；系统 Ubuntu 22.04；/opt 与 /var 剩余 > 8G
[ ] P1 apt update 通过；apt-cache policy ros-humble-ros-base 有 Candidate
[ ] P2 ros2 --help 正常；colcon 可用；cv2/numpy/yaml/serial 可导入
[ ] P3 实车：/dev/ttyAMA1 存在且在 dialout 组；/dev/rush_lidar -> ttyAMA2；i2cdetect 见 0x68；
       systemctl --user is-active pipewire pipewire-pulse wireplumber 全 active
[ ] P4 src 两个包到位（launch/edge_server_deploy.launch.py 存在）；colcon build 成功；
       ros2 pkg prefix edge_server 正常
[ ] P5 deploy.yaml 关键值正确（web 80/8080、state 指 /var/lib、required/strict_unknown 符合档位）
[ ] P6 /var 四个目录属于运行用户；单元安装并 daemon-reload；is-active=active、is-enabled=enabled
[ ] P7 80/8080/8090 在监听；/ 返回 200；journal 有 Loaded config 与 started
[ ] 收尾：本地 pytest 基线 180 passed；留 /opt/edge_server/backup 回滚点（§4.6）
```

## 3. 日常启停与验证
### 3.1 启停与状态
```bash
PI=ubuntu@192.168.3.26        # 开发机换成 eagle@172.31.4.88

ssh $PI 'systemctl status edge_server --no-pager'   # 看状态
ssh $PI 'sudo systemctl start edge_server'
ssh $PI 'sudo systemctl stop edge_server'
ssh $PI 'sudo systemctl restart edge_server'
ssh $PI 'systemctl is-active edge_server'           # 脚本里用这个
```
`stop` 只停应用，Pi 本身仍在运行，SSH 仍可登录。要关机见 §3.4。

### 3.2 开机自启与日志
```bash
ssh ubuntu@192.168.3.26 'systemctl is-enabled edge_server'          # enabled = 开机自启
ssh ubuntu@192.168.3.26 'sudo systemctl disable edge_server'        # 取消自启，不停止当前进程
ssh ubuntu@192.168.3.26 'sudo systemctl enable edge_server'         # 恢复自启
ssh ubuntu@192.168.3.26 'sudo systemctl disable --now edge_server'  # 取消自启并立即停止
```
```bash
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -n 100 --no-pager'  # 最近 100 行
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -f'                 # 实时跟随，Ctrl-C 退出
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -b --no-pager'      # 本次开机以来
```
ROS 自己的日志在 `/var/log/edge_server/ros/`（由 unit 里的 `ROS_LOG_DIR` 指定）。

### 3.3 启动后验证
`restart` 是异步的：命令返回时节点还没绑好端口（Pi 实测约 6 秒，x86_64 开发机约 1.6 秒），紧接着 `curl` 可能得到 `000`。先轮询就绪：
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
curl -s -o /dev/null -w 'web=%{http_code}\n'    http://192.168.3.26/
curl -s -o /dev/null -w 'dash=%{http_code}\n'   http://192.168.3.26/dashboard.html
curl -s -o /dev/null -w 'stream=%{http_code}\n' http://192.168.3.26:8080/stream
curl -s http://192.168.3.26/status; echo
curl -s http://192.168.3.26/api/modules/ros/health; echo
```
预期（当前没接相机时）：`web=200`、`dash=200`；`stream=503`（没有 `/camera/color/image_raw`，正常）；STM32 已接线时 `/status` 里 `"serial_connected":true`、`"encoder_ok":true`，没接或没上电时为 `false`；`"ros_status":"degraded"`（缺相机/雷达/IMU，属预期）；健康接口 `"ready":true`、`"healthy":false`，`reason` 列出缺哪些设备。`degraded` 不代表服务坏了，`ready:true` 才是关键。

### 3.4 整机开关机
**上电**：先接好相机、雷达、STM32 串口再上电（避免热插拔 USB）；等约 2～3 分钟（实机 `systemd-analyze`：kernel 7s + userspace 2min19s，主要在等网络）；服务自动启动，按 §3.3 验证。没起来时：
```bash
ssh ubuntu@192.168.3.26 'systemctl is-enabled edge_server; systemctl status edge_server --no-pager'
ssh ubuntu@192.168.3.26 'sudo journalctl -u edge_server -b -n 100 --no-pager'
```
**关机**：
```bash
ssh ubuntu@192.168.3.26 'sudo shutdown -h now'
```
等 SSH 断开、Pi 绿色 ACT 灯停止闪烁后再断总电源。**不要直接拔电**：ROS 日志、`edge_params.json` 都在写 SD 卡，硬断电容易把卡写坏。**重启**用 `ssh ubuntu@192.168.3.26 'sudo reboot'`，重启后服务随开机自启，无需手工干预。

### 3.5 连不上 Pi
```bash
ping 192.168.3.26
ssh -v ubuntu@192.168.3.26
```
ping 不通：查电源、Wi-Fi/网线；Pi 用 `wlan0` 的 `192.168.3.26`，Wi-Fi 未配好时可能处于 AP 模式 `192.168.3.26`。ping 通但 SSH 拒绝：确认用户名 `ubuntu`，以及本机 `~/.ssh/id_ed25519` 是否已授权。都正常但服务没起：按 §3.2 的 `journalctl` 排查。

### 3.6 急停
优先用**硬件断电/急停开关**。软件层面最快的是：
```bash
ssh ubuntu@192.168.3.26 'sudo systemctl stop edge_server'
```
停止时 systemd 发 `SIGINT`（`KillSignal=SIGINT`），launch 会一并结束两个节点；`TimeoutStopSec=10`，超过 10 秒会强杀。底盘节点的 `/cmd_vel` 超时和 STM32 遥测超时也会自动停车，但**不要依赖软件兜底**。

### 3.7 前台调试
服务方式看不到实时输出时，可以停服务、前台跑：
```bash
# 1) 先停服务，释放 80/8080/8090
ssh ubuntu@192.168.3.26 'sudo systemctl stop edge_server'

# 2) 前台启动（80 端口需要权限，所以用 sudo；配置必须显式指向 deploy.yaml）
ssh -t ubuntu@192.168.3.26 \
  'sudo bash -lc "export EDGE_SERVER_CONFIG=/opt/edge_server/config/deploy.yaml && \
                  source /opt/ros/humble/setup.bash && \
                  source /opt/edge_server/install/setup.bash && \
                  ros2 launch edge_server edge_server_deploy.launch.py"'
```
`Ctrl-C` 结束；调试完记得 `sudo systemctl start edge_server` 交还给 systemd。不想用 sudo：把 `deploy.yaml` 的 `web.port` 临时改成 8081 再跑，别提交这个改动。在 Pi 上用 zsh 交互查看环境：
```zsh
source /opt/ros/humble/setup.zsh
source /opt/edge_server/install/setup.zsh
ros2 pkg prefix edge_server
```
但**启动服务仍用上面的 `sudo bash -lc`**：`sudo` 会重置 `PATH`，直接 `sudo ros2 ...` 会报 `ros2: No such file or directory`；在 `bash -lc` 里重新 source 一次最稳。

## 4. 发布新版本
### 4.1 发布前准备（SSH 免密 + 本地测试）
```bash
ssh-keygen -t ed25519 -C "rush-mog-edge"   # 已有可跳过
ssh-copy-id ubuntu@192.168.3.26
```
之后 rsync/ssh 不再反复输入密码。发布前先在本地跑测试：
```bash
cd <repo>/software/edge_server        # 开发机 ~/workspace/projects/rush-mog；WSL /mnt/f/CODE/rush-mog
PYTHONPATH=.:$PYTHONPATH python3 -m pytest -q tests
```
当前基线：`180 passed`（2026-11-21，开发机 x86_64 / Python 3.10 / ROS Humble）。

### 4.2 同步 → 构建 → 重启
在 WSL 下执行：
```bash
cd /mnt/f/CODE/rush-mog
PI=ubuntu@192.168.3.26

rsync -avz --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  software/edge_server/ $PI:/opt/edge_server/src/edge_server/

rsync -avz --delete --exclude='__pycache__' --exclude='.pytest_cache' \
  software/custom_msgs/ $PI:/opt/edge_server/src/custom_msgs/

# 远端登录 shell 是 zsh，必须显式用 bash -lc 包一层
ssh $PI 'bash -lc "source /opt/ros/humble/setup.bash && \
                   cd /opt/edge_server && \
                   colcon build --packages-select custom_msgs edge_server && \
                   sudo systemctl restart edge_server"'
```
`--delete` 只清理远端这两个源码副本中多余的文件，不会碰 `config/`、`deploy/`、`install/`。改了 `custom_msgs`，第二条 rsync 必须执行；只改 Python/网页可以只同步 `edge_server`。构建耗时：首次 `custom_msgs` 约 7 分钟（Pi 4B）；之后只构建 `edge_server` 约 10 秒。

只改 `edge_server` 的 Python/网页、没有改消息时：
```bash
ssh $PI 'bash -lc "source /opt/ros/humble/setup.bash && \
                   cd /opt/edge_server && \
                   colcon build --packages-select edge_server && \
                   sudo systemctl restart edge_server"'
```
网页文件会被装到 `install/.../share/edge_server/web`，构建后浏览器记得强制刷新。

### 4.3 发布后验证
先按 §3.3 等 Web 就绪，再补充：
```bash
ssh $PI 'systemctl is-active edge_server'
curl -s http://192.168.3.26/status
curl -s http://192.168.3.26/api/speaker | python3 -m json.tool
curl -s http://192.168.3.26/api/modules/edge/params | python3 -m json.tool | grep -A2 speaker
ssh $PI 'sudo journalctl -u edge_server -n 50 --no-pager'
```

### 4.4 改动类型 × 发布动作
| 改动 | 需要同步 | 需要构建 | 需要重启服务 | 备注 |
|---|---|---|---|---|
| 节点 Python 逻辑 | `edge_server` | `edge_server` | 是 | 最常见 |
| `web/` 页面/JS/CSS | `edge_server` | `edge_server` | 建议重启 | 构建会把 web 装到 install；浏览器强刷 |
| `launch/`（含 `edge_server_deploy.launch.py`）、`config/robot.yaml` | `edge_server` | `edge_server` | 是 | 部署 launch 随构建装进 install；运行配置仍以远程 `deploy.yaml` 为准 |
| `package.xml` / `setup.py` / 入口点 | `edge_server` | `custom_msgs edge_server` 或清理后全量 | 是 | 入口点变化必须重装 |
| `custom_msgs` | `custom_msgs` + `edge_server` | 两者都构建 | 是 | 所有使用该消息的进程都需重启 |
| 只改远程 `config/deploy.yaml` | 不需要 rsync | 不需要 | 是 | 直接编辑远端配置（§4.5） |
| 改 `software/tools/systemd/edge_server.service` | 单独复制 | 不需要 | `daemon-reload` + restart | 单元在 `/etc`，不在 rsync 范围（§4.5） |

### 4.5 特殊改动
**只改远程运行配置：**
```bash
ssh ubuntu@192.168.3.26
sudoedit /opt/edge_server/config/deploy.yaml
sudo systemctl restart edge_server
```
**改部署 launch：** 现在它就是普通源码（`software/edge_server/launch/edge_server_deploy.launch.py`），
按 §4.2 同步 + `colcon build --packages-select edge_server` + restart 即可，**不再单独 scp**。

> 历史提示：旧部署把 launch 放在 `/opt/edge_server/deploy/edge_server.launch.py` 并手工复制。
> 如果目标机单元还是旧路径（`ExecStart` 里带 `/opt/edge_server/deploy/`），按 §4.5 的单元安装步骤换成本仓库模板一次即可。
**改 systemd 单元：**
```bash
scp /path/to/edge_server.service ubuntu@192.168.3.26:/tmp/edge_server.service
ssh ubuntu@192.168.3.26 '
  sudo cp /tmp/edge_server.service /etc/systemd/system/edge_server.service &&
  sudo systemctl daemon-reload &&
  sudo systemctl restart edge_server
'
```
**`src` 被误删 / 构建报 `Package 'edge_server' not found`：** 按 §4.2 重新同步并构建即可。`--delete` 只清理目标目录内部，不会碰 `config/`、`install/`；注意在 `/opt/edge_server` 下执行 build。

### 4.6 回滚
发布前保留上一版源码（**放在 colcon 工作区之外**，否则会被当成重名包）：
```bash
ssh ubuntu@192.168.3.26 '
  mkdir -p /opt/edge_server/backup &&
  rm -rf /opt/edge_server/backup/edge_server.bak &&
  cp -a /opt/edge_server/src/edge_server /opt/edge_server/backup/edge_server.bak'
```
回滚时把备份写回 `src/edge_server`，重新 build 并 restart：
```bash
ssh ubuntu@192.168.3.26 '
  rsync -a --delete /opt/edge_server/backup/edge_server.bak/ /opt/edge_server/src/edge_server/ &&
  bash -lc "source /opt/ros/humble/setup.bash && cd /opt/edge_server && colcon build --packages-select edge_server" &&
  sudo systemctl restart edge_server'
```
验证按 §4.3。

## 5. 运行模式
### 5.1 部署模式 vs 全栈模式
**部署模式**（当前开机自启）只跑 `edge_server_node` + `health_monitor_node`，负责网页、手柄、串口、图传、健康检查。**全栈模式**下 `robot_bringup.launch.py` 会再启动 VLA、感知、定位、导航、速度仲裁等节点，**并且也会启动一个 `edge_server_node`**。两者**不能同时跑**，否则 80 端口冲突。切全栈：
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

## 6. 外设接入
### 6.1 蓝牙音箱与语音播报
2026-11-15 起仓库默认 `model=newmine`（NewMine），可切 `am510`（HONOR AM510）；型号名/MAC 在 `edge.speaker.models.<id>` 配置。换设备后先 `bluetoothctl devices` 查实际名称/MAC，再更新 `/opt/edge_server/config/deploy.yaml`。

**自动连接**：音箱开机且不被手机/其他主机占用；`edge_server_node` 启动后立即尝试连接，之后每 20s 重试。连接成功后 PipeWire 创建 `bluez_output.<MAC>.a2dp-sink` sink，节点将其设为默认输出并应用 `edge.speaker.volume`。查看状态：
```bash
curl -s http://192.168.3.26/api/speaker | python3 -m json.tool
```
`state=connected, audio_ready=true` 可播报；`state=error` 看 `last_error`（常见 `br-connection-page-timeout`）；`state=scanning/connecting` 正在重试。手动强制重连：
```bash
curl -X POST http://192.168.3.26/api/speaker/connect
```

**手动连接 / 重配（bluetoothctl）**：
```bash
bluetoothctl paired-devices    # 取音箱 MAC，如 AA:BB:CC:DD:EE:FF
```
输出形如 `Device AA:BB:CC:DD:EE:FF 我的音箱`，其中 `AA:BB:CC:DD:EE:FF` 就是 MAC。进入交互控制台 `bluetoothctl`，依次输入（会看到 `Connection successful`，`quit` 退出）：
```text
power on
connect AA:BB:CC:DD:EE:FF
```
失败时先断开再重连：`disconnect AA:BB:CC:DD:EE:FF` → `connect AA:BB:CC:DD:EE:FF`。极端情况（之前配对记录异常）信任 + 重配：
```text
remove AA:BB:CC:DD:EE:FF
scan on
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
scan off
```
（`scan on` 后等音箱 MAC 出现再执行 `pair`。）

**语音播报**：
```bash
curl -X POST http://192.168.3.26/api/speaker/say \
  -H 'Content-Type: application/json' -d '{"text":"机器人已启动"}'

# 也可以直接用 ROS 话题
ros2 topic pub --once /body/say std_msgs/msg/String "{data: 请注意避让}"
```
- TTS 使用 `espeak-ng -v cmn`，离线可用；音色/语速在 `edge.speaker.tts.voice/speed` 调整。
- 每个 TTS 默认前置 450ms 静音（`tts.pre_roll_ms`），避免 A2DP 从 SUSPENDED 唤醒时吞掉第一个字；`playback_latency_ms`（默认 250）通过 `PULSE_LATENCY_MSEC` 请求更大播放缓冲。
- 播放队列串行，最多 20 条；`POST /api/speaker/stop` 清空并停止。播放本地文件先放到 `/var/lib/edge_server/audio/`，再 `POST /api/speaker/play`，只允许该目录（防任意路径读取），支持 wav/mp3/ogg/flac/m4a/aac：
```bash
curl -X POST http://192.168.3.26/api/speaker/play \
  -H 'Content-Type: application/json' -d '{"file":"prompt.wav"}'
```

**常见问题**：
- **`br-connection-page-timeout`**：音箱未响应经典蓝牙寻呼。多为被手机连着、休眠/关机、超出距离，或需按音箱配对键；手机端断开后，节点会在下一个 20s 周期自动连上。
- **`br-connection-profile-unavailable`**：BlueZ 已连上但 A2DP endpoint 未注册。`speaker.py` 会带 180s 冷却重启用户级 WirePlumber 后重试，dashboard 事件显示“检测到 A2DP profile 不可用...”。若同时出现 `page-timeout`/设备不广播，优先检查音箱电源。
- **已连接但 `audio_ready=false`**：PipeWire/WirePlumber 没起来：
```bash
systemctl --user is-active pipewire pipewire-pulse wireplumber
pactl list short sinks | grep bluez_output
loginctl show-user ubuntu -p Linger
```
- **能连但没声音**：确认 `pactl get-default-sink` 是 bluez sink，`wpctl status` 中该 sink 未静音；必要时 `pactl set-sink-mute <name> 0`。
- **服务里 `pactl` 报连接失败**：systemd 单元缺少 §2.6 的 `XDG_RUNTIME_DIR`/`PULSE_SERVER`，`daemon-reload` 后重启。

### 6.2 麦克风录音（HFP）
NewMine / HONOR AM510 等蓝牙音箱的麦克风只在 HFP/HSP profile 下可用。录音模块自动：停掉 A2DP 播放 → `pactl set-card-profile bluez_card.<MAC> headset-head-unit` → 等 `bluez_input.<MAC>.headset-head-unit` 输入源 → 用 `parec` 采集 8 kHz mono s16le，写 WAV 并按 100ms 发布 `/edge/audio` → 停止后恢复原 A2DP profile 和播放 sink。
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
录音期间：`GET /api/microphone` 的 `recording=true`、`source=bluez_input...`；默认 `gain=3.0` 会在写 WAV/发 ROS 前做软件增益（约 +9.5dB），可在 `/api/microphone/gain` 或 dashboard 参数里调整为 1.0..8.0；`ros2 topic hz /edge/audio` 约 10 Hz；输出 sink 变为 `...headset-head-unit` 属正常；停止后 `GET /api/speaker` 的 sink 恢复 `...a2dp-sink`。录音文件在 `/var/lib/edge_server/audio/mic`，只允许通过 API 下载。

排查：`supported=false` 表示 card 没有 `headset-head-unit` profile，检查 `pactl list cards`，确认不是只支持 A2DP 的音箱；报“未找到蓝牙麦克风输入源”是 HFP 切换失败/被手机占用，先 `bluetoothctl disconnect/connect` 再重试；文件很小或 RMS 接近 0 是麦克风静音/距离远，检查 `pactl list sources` 的 `Mute`/`Volume`，用 `/api/microphone/volume` 调大。

### 6.3 电源监控（电源及转换板）
Pi 5V 输入没有独立 ADC，使用 SoC 固件与 `rpi_volt` hwmon 判定：
```bash
vcgencmd get_throttled          # throttled=0x50000
cat /sys/class/hwmon/hwmon1/in0_lcrit_alarm   # 0=正常，1=当前欠压
vcgencmd measure_volts core
vcgencmd measure_temp
curl -s http://192.168.3.26/api/power | python3 -m json.tool
```
位定义与状态映射：`0x1` 当前欠压 → `error`；`0x2` ARM 限频、`0x4` 降频/限流、`0x8` 软温度限制 → `degraded`；`0x10000` 历史欠压、`0x40000` 历史降频、`0x80000` 历史温度限制 → 当前 `ok`、`warning=true`，dashboard 显示“历史欠压已恢复”；`/sys/class/hwmon/hwmon*/in0_lcrit_alarm=1` → `error`。

当前部署实测 `throttled=0x50000`，是此前一次瞬时欠压留下的历史位，`in0_lcrit_alarm=0`，所以电源模块显示 `ok`。需要清零历史位时，重启 Pi 或重新插拔电源后再观察；持续出现 `0x10000` 才说明供电/线缆/电源转换板需要检查。dashboard 的「电源及转换板」卡片直接使用 `/api/power`；health monitor 的 `power` 设备使用 `/body/power_status`，电源异常时整体健康会降级。

### 6.4 音频优化（卡顿、丢首字、麦克风增益）
已部署的优化（2026-11-14）：
1. **TTS 预滚 450ms**：`edge.speaker.tts.pre_roll_ms`，在 espeak 合成后给 WAV 前补静音，覆盖 A2DP 从挂起唤醒/功放启动的时间，避免第一个字被吞。
2. **播放缓冲 250ms**：`edge.speaker.playback_latency_ms`，通过 `PULSE_LATENCY_MSEC` 让 `paplay` 请求更大缓冲，降低蓝牙链路抖动卡顿。
3. **A2DP sink 常驻**：`/home/ubuntu/.config/wireplumber/bluetooth.lua.d/51-rush-mog.lua` 给 `bluez_output.*` 设置 `session.suspend-timeout-seconds=0`，播放结束后 sink 保持 IDLE 而不是 SUSPENDED，减少下次唤醒丢字。
4. **PipeWire 实时调度**：`/etc/polkit-1/localauthority/50-local.d/50-rush-mog-rtkit.pkla` 允许 `ubuntu` 获取 RealtimeKit 实时权限；重启后 `pipewire` 音频线程为 `RR 20`，不再报 `RTKit ... AccessDenied`。
5. **WiFi 省电关闭**：`/etc/systemd/system/wifi-powersave-off.service` 开机执行 `iw dev wlan0 set power_save off`，降低 2.4GHz WiFi 与蓝牙共存抖动。
6. **麦克风软件增益**：`edge.speaker.microphone.gain` 默认 `3.0`（约 +9.5dB），在写 WAV/发 ROS 前用 `audioop.mul` 放大；可用 `POST /api/microphone/gain {"gain":3.0}` 在线调整（1.0..8.0）。

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
如果仍卡顿，优先把 Pi 的 WiFi 切到 **5GHz**（当前 `wlan0` 在 2.4GHz channel 1，与蓝牙共用天线），或靠近路由器/音箱；这是无线共存问题，软件缓冲只能缓解，不能完全消除 2.4GHz 干扰。

### 6.5 激光雷达（D6/LD19）

> 接线：GPIO8=物理 pin24（TXD4，接雷达 RX，可不接），GPIO9=物理 pin21（RXD4，接雷达 TX，必须接），GND 共地，D6 通常 5V/1A 独立供电；`GPIO8/9` 与 `SPI0 CE0/CE1` 复用，必须 `# dtparam=spi=on` 关闭 SPI0。
代码已内置 `lidar_node`（`edge_server/core/lidar.py` + `ros_nodes/lidar_node.py`），支持 LD19/LD06/D6 的 47 字节协议（`0x54 0x2C`，CRC-8 0x4D）。接线与系统配置：
```ini
# /boot/firmware/config.txt
dtoverlay=uart4          # GPIO8/9，D6 激光；必须同时关闭 spi0（见下）
```
```bash
# 仓库模板：software/tools/systemd/99-rush-mog-lidar.rules
sudo cp 99-rush-mog-lidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 用 platform device 匹配（fe201800.serial=UART4），不要写死 ttyAMA 编号

# 重启后确认
ls -l /dev/rush_lidar /dev/ttyAMA*
```
配置 `robot.yaml`：
```yaml
lidar:
  enabled: true
  port: "/dev/rush_lidar"
  baud: 921600             # 国科光芯 D6 固定 921600；LD19/LD06 为 230400
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

# 或随实车 bringup
ros2 launch edge_server robot_bringup.launch.py start_lidar:=true
```
验收：
```bash
ros2 topic hz /scan             # ≈10 Hz
ros2 topic echo /scan --once    # ranges 长度 360，frame_id=laser_link
ros2 topic echo /lidar/status --once
ros2 run tf2_ros tf2_echo base_link laser_link
```
现状（2026-11-15）：已启用 `dtoverlay=uart4`、关闭 SPI0、创建 `/dev/rush_lidar`，`rush-mog-lidar.service` 已 active；但 UART4 上未收到任何字节（GPIO9 无电平跳变），需检查雷达供电/共地/TX→GPIO9 接线。诊断前先 `sudo systemctl stop rush-mog-lidar.service`，再执行 `sudo python3 /opt/edge_server/tools/lidar_probe.py`；`/lidar/status` 的 `state=no_data` 表示串口已开但无数据。单元模板在 `software/tools/systemd/`，`ConditionPathExists=/dev/rush_lidar`，设备不存在时静默跳过；接好线、创建 udev symlink 并重启后自动启动。波特率/协议不同改 `lidar.baud`/`lidar.protocol`；设备名不同改 udev 规则或 `lidar.port`。

#### 6.5.1 故障排查：孪生页面激光雷达状态显示 `unknown`

**现象**：线接好、`rush-mog-lidar.service` 也是 active，但 dashboard 数字孪生页面的「激光雷达」卡片一直显示 `unknown`。

**先理解数据链路**（`web/dashboard.js`）：

```text
D6 串口 -> lidar_node -> /scan (sensor_msgs/LaserScan, ~10Hz)
                              |
                              v
                    health_monitor_node 的 lidar 设备
                              |
                              v
              GET /api/modules 的 ros.devices[lidar]
                              |
                              v
                 dashboard.js: rosDevice('lidar')
```

`unknown` 的含义是 **health_monitor 从未收到过 `/scan` 消息**，不是“服务没启动”。只要 `/scan` 有数据，卡片会自动变成 `ok`；无需重启 dashboard。

排查顺序（每步都能独立定位）：

```bash
# 0) 服务与节点状态
systemctl is-active rush-mog-lidar.service
source /opt/ros/humble/setup.bash && source /opt/edge_server/install/setup.bash
ros2 topic echo /lidar/status --once --full-length
```

`/lidar/status` 的判定：

| state | packets/errors | 含义 | 下一步 |
|---|---|---|---|
| `receiving` | packets>0, scans>0 | 雷达数据正常 | 查 `/scan` 与 health，不是硬件问题 |
| `no_data` | packets=0, crc=0, header=0 | **串口已打开，但一个字节都没收到** | 供电/接线/雷达不输出（最常见） |
| `no_data` | packets=0, crc/header>0 | 有字节但帧头/CRC 不对 | 波特率或协议不匹配 |
| `error` | last_error 非空 | 串口打不开/权限/设备不存在 | 查 `/dev/rush_lidar`、dialout、udev |

`/lidar/status` 还提供 `rx_bytes`（串口收到的原始字节数）：`rx_bytes=0` 表示完全没有电气数据；`rx_bytes>0` 但 `packets=0` 表示有字节但帧头/CRC 不对（波特率或协议问题）。

```bash
# 1) UART4 / 设备节点 / 引脚
ls -l /dev/rush_lidar            # 应为 -> ttyAMA2
sudo dmesg | grep fe201800       # 应有 ttyAMA2
for f in $(sudo find /sys/kernel/debug/pinctrl -name pinmux-pins); do
  sudo grep -E "pin (8|9) " "$f" # 应为 fe201800.serial ... function alt4
done

# 2) 原始字节（先停服务，避免两个进程抢串口）
sudo systemctl stop rush-mog-lidar.service
sudo python3 /opt/edge_server/tools/lidar_probe.py --bauds 921600,460800,230400,115200
sudo systemctl start rush-mog-lidar.service

# 3) Pi 侧 UART 回环（排除 Pi/overlay 问题）
#    拔掉雷达，短接 GPIO8(物理 pin24) 与 GPIO9(物理 pin21)，然后：
sudo systemctl stop rush-mog-lidar.service
sudo python3 /opt/edge_server/tools/lidar_probe.py \
  --ports /dev/rush_lidar --bauds 115200 --send 55aa55aa
# 期望读到 4 字节 55 aa 55 aa；若成功说明 Pi UART4/overlay/udev 都正常
sudo systemctl start rush-mog-lidar.service

# 4) 健康接口
curl -s http://127.0.0.1:8090/ros/health/devices/lidar | python3 -m json.tool
```

**国科光芯 D6 的硬件要点（最容易踩坑）**：

1. **波特率固定 921600**（8N1，无校验），不是 LD19 的 230400；本现场已把
   `deploy.yaml` / `robot.yaml` 改为 `lidar.baud: 921600`。
2. D6 标配 **10 芯线**，**颜色会重复**（黄色有 Tx 和 IO_3，白色有 GND 和 IO_1，
   绿色有 Rx 和 IO_2，黑色有 GND 和 IO_2 In）。**必须按管脚号接线，不能按颜色**：
   - 管脚 0：红色 `+5V` → Pi 5V（pin2/4）
   - 管脚 1：白色 `GND` → Pi GND（pin6/9/14/20/25/30/34/39）
   - 管脚 2：黄色 `Tx` → **Pi GPIO9 / 物理 pin21（RXD4）**
   - 管脚 3：绿色 `Rx` → **Pi GPIO8 / 物理 pin24（TXD4）**
   - 管脚 4：黑色 `GND`（与管脚 1 内部连通，可再接一根地）
   - 管脚 5–9 是 NPN 输入/输出，接激光雷达做区域保护时才用，**不要接 Pi UART**。
3. 供电：D6 标称 **5V，启动电流 >1.5A，运行约 500mA，纹波 <40mV**。
   - 不建议直接从 Pi 5V 排针取电；优先用 D6 标配 5V 适配器/独立电源，**与 Pi 共地**；
   - 若必须用 Pi 供电，使用官方 5V/3A 以上电源，并确认 `vcgencmd get_throttled`
     没有 `0x50000`（历史上发生过欠压/降频）；
   - 若雷达电机不转，先怀疑供电/线序，而不是代码。
4. 通电后 D6 **默认自动旋转并输出点云**，一般不需要启动命令；少数批次需要
   `lidar.start_command_hex`（例如 `a5 40`），可用 probe 的 `--send` 验证。
5. 如果改用 D6 标配的 **TTL 转 USB 板**：插 Pi USB，设备通常是 `/dev/ttyUSB0`，
   把 `lidar.port` 改成 `/dev/ttyUSB0`，并同步修改 systemd 的
   `ConditionPathExists`/udev（或先手工 `ros2 run edge_server lidar_node` 验证）。

**本次实测记录（2026-11-15，供对照）**：

- `dtoverlay=uart4` 已启用，SPI0 已关闭；`GPIO8/9 = fe201800.serial, alt4`；
- `/dev/rush_lidar -> ttyAMA2` 存在，`rush-mog-lidar.service` active；
- `/lidar/status`：`state=no_data`、`packets=0`、`crc_errors=0`、`header_errors=0`；
- `/scan` 无消息（`ros2 topic hz /scan` 无输出），因此 dashboard 显示 `unknown`；
- 临时发布 `/scan`（10Hz、360 点）后 health 的 `lidar` 立即变为 `ok`，证明 dashboard/health 数据链路正常，`unknown` 只是因为真实串口没有数据；
- `lidar_probe` 在 921600/460800/230400/115200 下 **bytes=0**；
- GPIO9/GPIO8 2 秒内 **0 个边沿**；GPIO9 加下拉后仍读到 **1**（外部 TX 空闲为高，
  说明 TX 线已接、雷达有电，但没有数据跳变）；
- **Pi 侧 UART4 内部回环（PL011 LBE）成功收到 `55 aa 01 02`** → Pi UART/overlay 正常；
- 结论：Pi 侧软件/驱动正常，问题在 **D6 没有输出数据**：重点查 10 芯线管脚 0/1/2/3 是否正确、
  5V 启动电流是否足够、雷达电机是否旋转。接好后 `/lidar/status` 会变为
  `state=receiving`、`scans>0`，`/scan` 约 10Hz，health 的 `lidar` 自动变 `ok`，
  dashboard 孪生页面无需刷新即可恢复。

若 probe 能看到字节但 `/lidar/status` 仍是 `no_data` 且 `crc_errors` 增长，说明 D6
这批固件的帧格式不是 LD19 47 字节；把 `lidar_probe.py` 打印的 `sample=...` 保留下来，
再在 `edge_server/core/lidar.py` 增加对应协议解析（`lidar.protocol` 扩展）。

### 6.6 Astra Pro 深度相机
驱动 `ros-humble-orbbec-camera`（已安装）；udev `/etc/udev/rules.d/99-obsensor-libusb.rules`（已从 `/opt/ros/humble/share/orbbec_camera/udev/` 复制）。
```bash
# 不建议经 Hub；插 Pi 4 USB3 口，Astra Pro 是 USB2.0 设备
lsusb | grep -i 2bc5          # 应看到 Orbbec/Astra（2bc5:0403 等）

ros2 launch edge_server astra.launch.py
# 话题：/camera/color/image_raw、/camera/depth/image_raw、
#       /camera/color/camera_info、/camera/depth/camera_info
```
`astra.launch.py` 默认：`depth_registration:=true`（深度对齐彩色，bbox 可直接采深度）；`enable_point_cloud:=false`（Pi 4B 省 CPU/带宽）；640×480@10Hz；发布 `base_link → camera_link` 静态 TF，默认 `x=0.08,y=0,z=0.22`。

Edge 侧：`edge.camera.topic`/`perception.image_topic` 已切到 `/camera/color/image_raw`；`perception.depth_topic` 为 `/camera/depth/image_raw`，`prefer_depth: true`；`target_detector_node` 用深度 bbox 中心区域中值优先测距，深度不可用再回退激光/bbox 高度；health `depth_camera` 监控深度话题，dashboard 增加「深度相机」卡片；`robot_bringup.launch.py start_camera:=true` 可随整车启动。

验收：
```bash
ros2 topic hz /camera/color/image_raw   # ≈10 Hz
ros2 topic hz /camera/depth/image_raw   # ≈10 Hz
ros2 topic echo /camera/depth/image_raw --once --field encoding   # 16UC1/32FC1
ros2 topic echo /camera/depth/camera_info --once
ros2 run tf2_ros tf2_echo base_link camera_link
```
现状与可选服务：本机尚未插 Astra Pro（`lsusb` 没有 `2bc5`），驱动未启动。已安装并 enable 可选的 `rush-mog-camera.service`（`ConditionPathExists=/dev/astra_pro`）：相机插上且 Orbbec udev 规则生效后，重启或 `sudo systemctl start rush-mog-camera` 即启动；模板 `software/tools/systemd/rush-mog-camera.service`；手动调试仍推荐 `ros2 launch edge_server astra.launch.py`。若 Orbbec 驱动无法识别老款 Astra Pro，先确认 `lsusb`/`ros2 topic list`；仍不行则换 Astra Pro Plus/Gemini 2/D435i，Edge 侧话题/深度分支不用改。

### 6.7 悬崖传感器（STM32 PB3/PB4）
悬崖传感器接在 STM32 的 PB3/PB4，由 `body_control` 固件本地做 10ms 采样、30ms 消抖，并在触发时封锁运动。Edge 侧要看到状态，STM32 必须上报悬崖掩码：
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
Dashboard「悬崖传感器」卡片：左右均未触发为绿色 `CLEAR`；任一触发为红色 `CLIFF`，详情显示左/右，CHIP 为 `LEFT/RIGHT CLIFF`；旧固件/无遥测显示“无悬崖遥测（需刷入带 cliff_mask 的 STM32 固件）”；触发/解除会写入 Edge 运行事件（最近事件面板）。health `cliff` 设备（`required: false`）在触发或遥测缺失时标红，不影响其它必需设备；要让整体健康因悬崖遥测缺失直接失败，可把 `required` 改为 `true`。

> 先在桌面上确认左右都 `CLEAR`，再分别挡住/移到桌沿确认对应 `CLIFF`；触发期间固件只允许 STOP/BACK，这是安全策略，不是 Edge 软件限制。

### 6.8 ICM-20948（Pi I2C1）
```bash
sudo apt install i2c-tools python3-smbus2
ls -l /dev/i2c-1
sudo i2cdetect -y 1        # 应看到 68（SA0 接地）或 69（SA0 接 3.3V）
```
接线：ICM-20948 `VCC=3.3V`、`GND`、`SDA=GPIO2`、`SCL=GPIO3`；I2C 上拉 4.7k 到 3.3V。**不要接 5V 信号。** 配置（`robot.yaml` / `deploy.yaml`）：
```yaml
imu:
  enabled: true
  i2c_bus: 1
  address: "0x68"
  frame_id: "imu_link"
  rate_hz: 100.0
  retry_sec: 2.0       # 未接/未上电时自动重试
```
单独启动/验证：
```bash
ros2 launch edge_server imu.launch.py
# 或随实车 bringup（现在默认 start_imu:=true）
ros2 launch edge_server robot_bringup.launch.py

ros2 topic hz /imu/data        # ≈100 Hz
ros2 topic echo /imu/data --once
ros2 run tf2_ros tf2_echo base_link imu_link
```
`imu_node` 启动失败不会退出，会按 `retry_sec` 重试，传感器接好后无需重启节点；想临时关闭用 `imu.enabled=false` 或 `start_imu:=false`。标定：静止采集陀螺零偏填 `imu.calibration.gyro_bias`；加速度计六面校准填 `accel_bias`；磁力计 8 字校准填 `mag_bias`/`mag_scale`，并设置 `mag_declination_rad`。磁场受电机干扰时 `imu_node` 只慢速融合，不会硬切航向。

本机 I2C1 扫描为空（没有 0x68/0x69），说明 ICM-20948 还没接好或未上电；`imu_node` 已部署并每 2s 重试。接好后看：
```bash
sudo i2cdetect -y 1
ros2 topic hz /imu/data
curl -s http://127.0.0.1:8090/ros/health | python3 -m json.tool | grep -A5 '"id": "imu"'
```

### 6.9 SLAM 链路验收
| 模块 | 状态 |
|---|---|
| `slam_toolbox` | 已安装，`slam_stack.launch.py` 可启动 |
| `map_manager_node` | 由全栈/`robot_bringup` 启动；联调已验证 `/center/map_control` 可用 |
| `/scan` 驱动 | `lidar_node` 已部署；当前无雷达硬件，`/scan` 无 publisher |
| TF | `edge_server_node` 发布 `odom→base_link`；`lidar_node` 发布 `base_link→laser_link`（雷达启动时） |
| EKF | `robot_localization` 已安装，`ekf.launch.py` + `ekf.yaml` 可用；默认不启动，避免和 `base.publish_tf` 抢 TF |
| Nav2 | `nav2_bringup` 未安装；`navigation_node` 自动回退内置直航 |

启动顺序：
```bash
# 1) 实车边端（含 map_manager；若用完整栈也可 robot_bringup）
ros2 launch edge_server robot_bringup.launch.py

# 2) 建图
ros2 launch edge_server slam_stack.launch.py

# 3) 可选：EKF（先把 robot.yaml 的 base.publish_tf 改 false）
ros2 launch edge_server ekf.launch.py
```
验收：
```bash
ros2 topic hz /scan                 # 需雷达；当前 0 publisher
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /map --once         # slam_toolbox 启动后应有 OccupancyGrid
```
地图保存（已实测）：
```bash
ros2 service call /center/map_control custom_msgs/srv/MapControl \
  "{command: save, map_name: /tmp/rush_slam_test}"
# 成功时应生成 /tmp/rush_slam_test.pgm + .yaml
```
本机 `/scan` publisher 数量为 0、`/imu/data` 无数据，所以真实 SLAM 还不能跑；先按 §6.5/§6.8 接好 D6 和 ICM-20948。合成激光 + TF + slam_toolbox 已联调通过，`/map` 能生成并保存。

## 7. 排障
### 7.1 zsh：`setup.bash` 报找不到 `setup.sh`
现象：
```text
/opt/ros/humble/setup.bash:.:11: no such file or directory: /opt/edge_server/setup.sh
```
原因：当前 shell 是 zsh，`setup.bash` 里的 `BASH_SOURCE` 为空，前缀退化成当前目录。处理（三选一）：
```zsh
# A. zsh 里用对应的 zsh 版本（推荐交互使用）
source /opt/ros/humble/setup.zsh
source /opt/edge_server/install/setup.zsh
```
```bash
# B. 临时进 bash 子 shell
bash
source /opt/ros/humble/setup.bash
source /opt/edge_server/install/setup.bash
```
```bash
# C. 把默认 shell 改回 bash（推荐这台机器，文档/脚本都按 bash 写）
sudo chsh -s /bin/bash ubuntu
# 退出 SSH 重新登录后生效
```
不要改 `/opt/ros/humble/setup.bash` 本身。远程非交互命令一律用 `bash -lc "..."`（§0.3 铁律 3）。

### 7.2 串口 / GPIO 排障
正常日志：`STM32 serial connected: /dev/ttyAMA1@115200`。收不到遥测时按顺序查：
1. `ls -l /dev/ttyAMA1` 是否存在（不存在 = overlay 没生效/没重启）；`/status` 里 `serial_connected:false` 同理。
2. 波特率必须与固件 `BODY_UART_BAUD_RATE` 一致（当前 115200），在 9600 下读到的会是乱码。
3. TX/RX 是否交叉（Pi TX 接 STM32 RX），是否共地（接线见 §8.3）。
4. 固件是否已烧录当前版本（`software/body_control`，115200、`$B` 遥测）。

权限与 overlay：
```bash
ls -l /dev/ttyAMA1              # 应存在，root:dialout
groups | grep dialout          # ubuntu 应在 dialout 组
grep uart3 /boot/firmware/config.txt
```
改 `/boot/firmware/config.txt` 后必须 `sudo reboot`。

本次故障根因（记录）：① **GPIO4/5 默认不是 UART**：`config.txt` 只有 `enable_uart=1`（那是 GPIO14/15 的 UART0），没有 `dtoverlay=uart3`，所以只有 `/dev/ttyAMA0`，没有 GPIO4/5 对应的设备；② **配置指向了不存在的设备**：`deploy.yaml` 里是 `/dev/ttyUSB0`（USB 串口），实际接线是 GPIO UART；③ **波特率不一致**：配置 9600，固件 `BODY_UART_BAUD_RATE=115200`。修复后 `dtoverlay=uart3` → `/dev/ttyAMA1`，`base.port=/dev/ttyAMA1`、`base.baud=115200`，遥测 `$B,...` 正常，`serial_connected=true`。

### 7.3 分层排查：从浏览器到 LED
按顺序查，命中哪层就停在哪层：
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

HTTP `/cmd` 返回 `ok` 只代表命令被接受，**不代表串口写成功**；真正的下行确认要么看 `strace` 的 `write`，要么看 STM32 的 LED/计数器。

**LED 含义**：`execute_command()` 第一行就是 `LED = !LED`（PC13）。每成功出队一条命令，LED 翻转一次；上电初始化后 LED 先亮再灭，之后只由命令驱动；**LED 不闪 = 命令没到 `execute_command()`**，问题在 USART3 接收/队列，不在电机。网页“按一下”会发两条命令（按下 `1`、松开 `0`），LED 翻转两次；一条命令对应一次翻转；按住期间不再重发，所以不会连闪（旧版本才会 5Hz 连闪）。要连续观察，发奇数条停车命令：
```bash
for i in $(seq 1 9); do
  curl -s -X POST -H 'Content-Type: text/plain' \
    --data 'button:stop:down' http://192.168.3.26/cmd >/dev/null
  sleep 0.5
done
```
9 条命令应在 4~5 秒内看到 LED 翻转 9 次。固件里留了三个可直接用 ST-Link watch 的计数器（`ps2_receiver.c`）：
| 变量 | 含义 |
|---|---|
| `g_ps2_rx_bytes` | 收到的字节数；不涨 = 接线/波特率/引脚复用问题 |
| `g_ps2_rx_frames` | 完整帧数；bytes 涨而它不涨 = 没收到换行 |
| `g_ps2_rx_last_byte` | 最后一个字节，正常应是 `0x0A`/`0x0D` 或命令字符 |

按住按钮时 STM32 LED 连闪：服务端在周期性重发（旧行为），每条命令都会翻转 LED。新版默认已关闭；确认 `edge.motion.resend_manual` 为 `false` 并重启服务。

### 7.4 最小验证命令
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
如果第 3 步 LED 不翻转，但第 4 步能读到 `$B,...`：检查 Pi GPIO4 → STM32 PB11 这根线（TX 方向）；检查两块板是否共地；用 ST-Link 看 `g_ps2_rx_bytes` 是否增长；确认烧录的固件与 `software/body_control` 当前源码一致（115200、LED 翻转逻辑）。

### 7.5 常见问题速查
| 现象 | 原因 | 处理 |
|---|---|---|
| `Package 'edge_server' not found` | `src/edge_server` 不在，或不在 `/opt/edge_server` 下执行 | 按 §4.2 重新同步；`cd /opt/edge_server` 再 build |
| `colcon: command not found` | PATH 异常，或 colcon 没装 | `command -v colcon`；`sudo apt install python3-colcon-common-extensions` |
| `ros2: command not found` | PATH 异常，或 zsh 下 source 失败 | 按 §7.1 处理 |
| 手动跑报 `Permission denied`（80 端口） | 普通用户不能绑 <1024 | 用 `sudo`，或交给 systemd（unit 有 `CAP_NET_BIND_SERVICE`） |
| `/stream` 一直 503 | 没有 `/camera/color/image_raw` | `ros2 topic hz /camera/color/image_raw`；先起 Astra 驱动 |
| `ros2 topic hz /scan` 无数据 | `/dev/rush_lidar` 不存在、波特率/协议不对或节点未启动 | `ls -l /dev/rush_lidar`；`ros2 topic echo /lidar/status`；检查 `lidar.port/baud` 和 `dtoverlay=uart4` |
| `/lidar/status` 的 `crc_errors` 持续增加 | 波特率不对、线缆干扰或协议不是 LD19 | 确认 D6 说明书波特率（230400/921600）、共地、TX/RX 交叉、缩短线缆 |
| 没有 `/camera/color/image_raw` 或 `/camera/depth/image_raw` | Astra 未插好、USB 供电/带宽不足、驱动未启动 | `lsusb \| grep 2bc5`；`ros2 launch edge_server astra.launch.py`；插 USB3 口、不经 Hub、降分辨率/帧率 |
| `/camera/depth/image_raw` 有图但目标测距明显偏 | 深度未对齐到彩色/外参不对 | `depth_registration:=true`；检查 `base_link→camera_link` TF；必要时 `prefer_depth:=false` |
| dashboard 悬崖卡片一直“无遥测” | STM32 固件还是 8 字段 `$B`，没有 cliff_mask | 重新编译/烧录 `body_control`；`ros2 topic echo /base/status --once` 应看到 9 个字段 |
| 悬崖卡片左右状态反了/一直 CLIFF | 模块 DO 空闲电平与 `CLIFF_*_TRIGGER_LEVEL` 相反，或 PB3/PB4 悬空 | 调模块电位器；按实际改 `cliff_sensor.h` 触发宏；确认共地 |
| `/status` 里 `serial_connected:false` | 串口没打开：没启用 `uart3`、设备名不对或被占用 | `ls -l /dev/ttyAMA1`、`grep uart3 /boot/firmware/config.txt`、`journalctl -u edge_server`；确认 `ubuntu` 在 `dialout` 组（§7.2） |
| 网页按钮无反应、`/cmd` 返回 `ok` | 命令被接受但串口没写出去，或 STM32 没收到 | 按 §7.3 从 `/status`→`strace`→LED 逐层查 |
| 按住按钮时 STM32 LED 连闪 | 服务端在周期性重发（旧行为），每条命令都会翻转 LED | 新版默认已关闭；确认 `edge.motion.resend_manual` 为 `false` 并重启服务 |
| `/api/speaker` 为 `error` / `br-connection-page-timeout` | 音箱休眠、被手机占用或超出距离 | 音箱开机并断开手机；节点每 20s 自动重连，或 `POST /api/speaker/connect`（§6.1） |
| `/api/speaker` 已连接但 `audio_ready=false` | PipeWire/WirePlumber 用户服务没起，或 sink 未创建 | `systemctl --user status pipewire wireplumber`；`pactl list short sinks \| grep bluez_output`；详见 §6.1 |
| 语音播报入队但没声音 | 默认 sink 不是蓝牙，或音箱静音/音量为 0 | `pactl get-default-sink`、`pactl set-sink-mute <name> 0`、`POST /api/speaker/volume` |
| `/api/microphone` 的 `supported=false` | 耳机 card 没有 HFP/HSP profile，或音箱未连接 | `pactl list cards \| grep -A12 bluez_card`；确认 `headset-head-unit` 存在 |
| 录音报“未找到蓝牙麦克风输入源” | HFP profile 切换失败/被手机占用 | `bluetoothctl disconnect/connect`；`pactl list short sources \| grep bluez_input`；再重试 |
| 录音没声音/文件很小 | 麦克风静音、音量低或距离远 | `pactl list sources` 看 `Mute`/`Volume`；`POST /api/microphone/volume`；`POST /api/microphone/gain` |
| 语音卡顿/少第一个字 | A2DP 唤醒丢包、WiFi 2.4G 共存或 PipeWire 无实时调度 | 已配置预滚/缓冲/常驻 sink/RTKit/WiFi 省电关闭；仍卡顿优先切 5GHz WiFi（§6.4） |
| `/api/power` 为 `error` / `undervoltage=true` | 5V 供电不足、线缆压降或电源转换板异常 | 检查 12V→5V 3A 输出、USB-C 线、接线端子；`vcgencmd get_throttled`、`dmesg \| grep -i undervolt` |
| `/api/power` 为 `degraded` | 当前降频/温度限制，不一定是电源 | 看 `warnings`、`temperature_c`；先降温/降负载再复测 |
| 网页 200 但还是旧界面 | 没 build/restart，或浏览器缓存 | 重新 build + restart，浏览器 Ctrl-F5 |
| 服务反复重启 | 配置 YAML 写错/端口被占 | `journalctl -u edge_server -n 100` |
| 日志时间比北京时间慢 8 小时 | Pi 时区是 UTC | `sudo timedatectl set-timezone Asia/Shanghai`（可选） |
| 网页/接口全部超时 | 服务没起或 Pi 掉线 | `ping`、`systemctl status`、`journalctl -b` |
| `Unit edge_server.service has a bad unit file setting` / `Assignment outside of section` / `Service has no ExecStart=` | 单元文件被写残：只剩 `Environment=` 行，没有 `[Unit]`/`[Service]`/`ExecStart` | 按 §2.6 重新安装模板（`sed 's/^User=.*/User=<user>/' 模板 \| sudo tee ...`），再 `daemon-reload` |
| `is-enabled` 是 `static`、`enable` 报无 `[Install]` | 同上：单元缺少 `[Install] WantedBy=` | 重装单元后 `systemctl enable edge_server` |
| `status=203/EXEC` 或 `ros2: No such file or directory` | `ExecStart` 没走 `bash -lc`，或 ROS 不在 `PATH` | 用模板里的 `ExecStart`（`bash -lc` + 两个 `source`） |
| 节点起来但 80 报 `Permission denied`、`bound: False` | 单元缺 `AmbientCapabilities=CAP_NET_BIND_SERVICE`，或前台运行时没 sudo | 重装单元；前台调试临时改 `web.port: 8081`（§3.7） |
| `ROS_LOG_DIR`/`ROS_HOME` 报 Read-only / Permission denied | `/var/log/edge_server/ros`、`/var/lib/edge_server/ros` 没建或属主不对 | `sudo install -d -o <user> -g <user> /var/...`（§2.6 第 1 步） |
| 开机没有 `/run/user/1000`，蓝牙/音频相关全失败 | 没执行 `loginctl enable-linger <user>` | `sudo loginctl enable-linger <user>`，`systemctl restart edge_server` |
| 服务 5 秒一次反复重启 | `Restart=on-failure` 生效，配置/YAML 或端口有错 | `journalctl -u edge_server -n 100`；修好后 `systemctl reset-failed edge_server` 再 start |

## 8. 速查表
### 8.1 路径与端口
| 项 | 值 |
|---|---|
| 源码（Pi） | `/opt/edge_server/src/edge_server`、`/opt/edge_server/src/custom_msgs` |
| 构建产物 | `/opt/edge_server/install/`（不要手改） |
| 运行配置 | `/opt/edge_server/config/deploy.yaml`（`EDGE_SERVER_CONFIG`） |
| 部署 launch | `/opt/edge_server/install/edge_server/share/edge_server/launch/edge_server_deploy.launch.py`（源自 `launch/edge_server_deploy.launch.py`） |
| systemd unit | `/etc/systemd/system/edge_server.service`（模板 `software/tools/systemd/edge_server.service`） |
| 应用日志 | `journalctl -u edge_server` |
| ROS 日志 | `/var/log/edge_server/ros/` |
| ROS home | `/var/lib/edge_server/ros/` |
| 状态文件 | `/var/lib/edge_server/edge_params.json` |
| 音频目录 | `/var/lib/edge_server/audio`（`/api/speaker/play` 只允许此目录） |
| 录音目录 | `/var/lib/edge_server/audio/mic` |
| Web / 手柄 | `http://192.168.3.26/`（:80） |
| Dashboard | `http://192.168.3.26/dashboard.html` |
| 状态 / 模块 | `http://192.168.3.26/status`、`http://192.168.3.26/api/modules` |
| 图传 MJPEG | `http://192.168.3.26:8080/stream` |
| ROS 健康 | `http://192.168.3.26/api/modules/ros/health`；本机 `http://127.0.0.1:8090/ros/health` |
| 音箱 / 麦克风 / 电源 | `/api/speaker`、`/api/microphone`、`/api/power` |
| 雷达设备 | `/dev/rush_lidar`（UART4 GPIO8/9） |
| 雷达状态 | `ros2 topic echo /lidar/status --once` |
| 悬崖状态 | `ros2 topic echo /body/cliff_status --once` |
| Astra USB 规则 | `/etc/udev/rules.d/99-obsensor-libusb.rules` |
| Astra 启动 | `ros2 launch edge_server astra.launch.py` |
| WirePlumber 音频规则 | `/home/ubuntu/.config/wireplumber/bluetooth.lua.d/51-rush-mog.lua` |
| RTKit 授权 | `/etc/polkit-1/localauthority/50-local.d/50-rush-mog-rtkit.pkla` |
| WiFi 省电关闭 | `/etc/systemd/system/wifi-powersave-off.service` |
| 串口 | `/dev/ttyAMA1`（GPIO4/5，UART3），115200 8N1 |

### 8.2 配置键
| 配置 | 位置 | 本机值 |
|---|---|---|
| 运行配置 | `/opt/edge_server/config/deploy.yaml` | 远程专用，不随 rsync 更新 |
| 仓库实车配置 | `edge_server/config/robot.yaml` | 版本管理，默认值 |
| 串口 | `base.port` / `base.baud` | `/dev/ttyAMA1` / `115200` |
| 手动控制重发 | `edge.motion.resend_manual` | `false`（按下只发一次；`true`=旧行为） |
| 手动/raw 刷新周期 | `edge.motion.keepalive_ms` | `200` |
| Web / MJPEG | `edge.web` | `0.0.0.0:80` / `8080` |
| 遥测状态文件 | `edge.state_file` | `/var/lib/edge_server/edge_params.json` |
| 健康网关 | `health.http` | `127.0.0.1:8090` |

改 `deploy.yaml` 后必须 `systemctl restart edge_server`；改仓库 `robot.yaml` 不会自动影响这台机器。

### 8.3 GPIO / UART 接线（UART3）
Pi 通过 40Pin 的 GPIO4/5 连 STM32 USART3，不是 USB 串口：
| Pi 引脚 | Pi 功能 | STM32 | 说明 |
|---|---|---|---|
| GPIO4（物理 7） | TXD3 | PB11（USART3_RX） | Pi 发 → STM32 收 |
| GPIO5（物理 29） | RXD3 | PB10（USART3_TX） | STM32 发 → Pi 收 |
| GND（物理 6/9/14…） | GND | GND | **必须共地** |

双方都是 3.3V 电平；PB10/PB11 是 5V 容忍脚，不需要电平转换，**不要接 5V**。Pi 侧设备名 `/dev/ttyAMA1`，由 `dtoverlay=uart3` 生成；`enable_uart=1` 只启用 GPIO14/15 的 UART0，**不能**代替它。`/boot/firmware/config.txt` 末尾应有：
```ini
dtoverlay=uart3
```
没生效时检查该行是否存在，改完必须 `sudo reboot`。对应运行配置：
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
正常日志：`STM32 serial connected: /dev/ttyAMA1@115200`。收不到遥测按 §7.2 顺序排查。

### 8.4 蓝牙手动连接（速查）
完整步骤见 §6.1。最短路径：
```bash
bluetoothctl paired-devices    # 取音箱 MAC，如 AA:BB:CC:DD:EE:FF
bluetoothctl                   # 进入交互控制台
```
```text
power on
connect AA:BB:CC:DD:EE:FF
```
失败：`disconnect <MAC>` → `connect <MAC>`；配对记录异常：`remove <MAC>` → `scan on` → `pair <MAC>` → `trust <MAC>` → `connect <MAC>` → `scan off`。
