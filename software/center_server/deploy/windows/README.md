# Windows（192.168.3.25）部署说明

> 本目录脚本用于把 Center Server 以 **Windows 开机任务** 方式常驻运行。
> 完整流程、增量构建/发布/维护见 [`../../README.md`](../../README.md) 第 9 节。

## 文件

| 文件 | 作用 |
|---|---|
| `install_windows.ps1` | 一次性安装：venv + 基础依赖 + 防火墙 + 开机任务（需管理员） |
| `install_vla.ps1` | 安装 torch 2.6.0+cu124 / torchvision / lerobot[smolvla]；`-Cpu` 只装 CPU 版 |
| `download_model.ps1` | 从 hf-mirror.com 下载 `lerobot/smolvla_base` 到 `models\` |
| `start_center.ps1` | 任务实际执行的启动脚本（设置环境变量/日志/PID） |
| `stop_center.ps1` | 停止 Center Server（PID + 命令行兜底） |

## 快速命令

```powershell
# 管理员 PowerShell，仓库根目录
cd F:\CODE\rush-mog\software\center_server

# 1) venv + 基础依赖 + 防火墙 + 开机任务 + 立即启动
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\install_windows.ps1

# 2) 真实模型依赖（约 2.5GB；curl 断点续传到 <repo>\wheels 后本地安装）
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\install_vla.ps1
# 无 GPU / 显存不足：
# powershell ... -File deploy\windows\install_vla.ps1 -Cpu

# 3) 下载权重（走 hf-mirror；基座 SmolVLM2 在首次 warmup 时自动下载约 2GB）
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\download_model.ps1

# 4) 重启服务使新依赖/模型生效
Stop-ScheduledTask -TaskName rush-mog-center
powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\stop_center.ps1
Start-ScheduledTask -TaskName rush-mog-center

# 5) 首次预热（会下载基座 SmolVLM2，可能数分钟）
Invoke-RestMethod -Method Post http://127.0.0.1:8000/v1/warmup
```

## 运行位置与日志

| 项目 | 路径 |
|---|---|
| 代码/venv | `F:\CODE\rush-mog\software\center_server` / `.venv` |
| 配置 | `config.yaml`（`CENTER_SERVER_CONFIG`） |
| 模型 | `F:\CODE\rush-mog\models\smolvla_base` |
| HF 缓存 | `C:\ProgramData\rush-mog-center\hf-cache` |
| 应用日志 | `C:\ProgramData\rush-mog-center\stderr.log` |
| 包装脚本日志 | `C:\ProgramData\rush-mog-center\center_server.log` |
| PID | `C:\ProgramData\rush-mog-center\center.pid` |

## 说明

- 任务以 `SYSTEM` 身份开机启动，不依赖用户登录；`F:` 为本地固定磁盘。
- 防火墙只放行 `192.168.3.0/24` 访问 TCP 8000；其他网段默认拒绝。
- 更新环境变量（IP/模型/设备）改 `start_center.ps1` 后重启任务；长期配置也可写进 `config.yaml`。
- 卸载：`Unregister-ScheduledTask -TaskName rush-mog-center -Confirm:$false`，
  并删除防火墙规则 `Rush-MOG Center Server (8000)`。
