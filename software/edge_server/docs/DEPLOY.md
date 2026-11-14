# Edge Server 部署与发布说明

> 目标机：`ubuntu@192.168.3.26`（Raspberry Pi 4B，Ubuntu 22.04 arm64）
> 部署目录：`/opt/edge_server`
> 本文只描述当前这台机器的部署方式，不包含密码/密钥。
>
> **日常启停、日志、整机开关机、故障排查见 [OPERATIONS.md](OPERATIONS.md)。**

---

## 0. 先看：两个高频坑

### 0.1 在 zsh 里 `source setup.bash` 会报“找不到文件”

这台机器的登录 shell 是 **zsh**，而 ROS 的 `setup.bash` 依赖 bash 专有变量
`BASH_SOURCE`。在 zsh 下会报：

```text
/opt/ros/humble/setup.bash:.:11: no such file or directory: /opt/edge_server/setup.sh
```

不是 ROS 装坏了，处理方法（详见 OPERATIONS.md §7.1）：

```zsh
# zsh 里用 zsh 版本
source /opt/ros/humble/setup.zsh
source /opt/edge_server/install/setup.zsh

# 或者切回 bash（推荐，本文所有命令都按 bash 写）
sudo chsh -s /bin/bash ubuntu   # 重新登录后生效
```

因此本文里所有 `ssh ... 'source ...'` 都要写成 `ssh ... 'bash -lc "source ..."'`，
否则远端会用 zsh 执行。

### 0.2 命令到底在哪台机器上跑

- `source /opt/ros/...`、`colcon build`、`ros2 launch`：**在 Pi 上**；
- `rsync`、`scp`、`curl http://192.168.3.26/...`：在本地 WSL；
- 本地 WSL 和 Windows 上没有 `/opt/edge_server`，在本地执行必然“找不到文件”。

---

## 1. 这次安装实际做了什么

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
  - 健康检查：只监控当前 5 个设备，`check_nodes=false`、
    `safety_required=false`，缺设备按 `degraded` 而不是 `error` 上报
- `/opt/edge_server/deploy/edge_server.launch.py`：只启动
  `edge_server_node` + `health_monitor_node`，不启动 SLAM/Nav2/感知全栈。

### 1.4 systemd 服务

- 单元文件：`/etc/systemd/system/edge_server.service`
- 运行用户：`ubuntu`
- 通过 `AmbientCapabilities=CAP_NET_BIND_SERVICE` 绑定 80 端口
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
- 未接 STM32：`serial_connected=false`
- 未启动 ROS 相机驱动：`camera=0`，`/stream` 返回 503
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

当前基线：`94 passed`。

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
