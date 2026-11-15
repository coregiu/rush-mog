# Rush-MOG Center Server —— Windows 启动脚本（Task Scheduler 调用）
#
# 由 install_windows.ps1 注册为开机任务（SYSTEM）：
#   powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\start_center.ps1
#
# 手工调试也可直接运行本脚本（前台日志见 C:\ProgramData\rush-mog-center\）。
$ErrorActionPreference = "Stop"

$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$RepoRoot = (Resolve-Path (Join-Path $Root "..\..")).Path
$DataDir = "C:\ProgramData\rush-mog-center"
$Python = Join-Path $Root ".venv\Scripts\python.exe"
$ModelsDir = Join-Path $RepoRoot "models"

if (-not (Test-Path $Python)) {
    throw "未找到虚拟环境: $Python；请先执行 deploy/windows/install_windows.ps1"
}
New-Item -ItemType Directory -Force -Path $DataDir | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $DataDir "hf-cache") | Out-Null

$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"
# Hugging Face 国内镜像；模型已下载后可把 HF_HUB_OFFLINE 设为 1
$env:HF_ENDPOINT = "https://hf-mirror.com"
$env:HF_HOME = Join-Path $DataDir "hf-cache"
$env:CENTER_SERVER_CONFIG = Join-Path $Root "config.yaml"
$env:CENTER_SERVER_EDGE_URL = "http://192.168.3.26"
$env:CENTER_SERVER_PUBLIC_HOST = "192.168.3.25"
$env:CENTER_SERVER_MODEL_PATH = Join-Path $ModelsDir "smolvla_base"
$env:CENTER_SERVER_DEVICE = "cuda"
$env:CENTER_SERVER_DTYPE = "float16"

$stdout = Join-Path $DataDir "stdout.log"
$stderr = Join-Path $DataDir "stderr.log"
$wrapperLog = Join-Path $DataDir "center_server.log"
$pidFile = Join-Path $DataDir "center.pid"

"[$(Get-Date -Format o)] starting: $Python -m center_server --config $($env:CENTER_SERVER_CONFIG)" |
    Out-File -Append -Encoding utf8 $wrapperLog

$process = Start-Process -FilePath $Python `
    -ArgumentList @("-m", "center_server", "--config", $env:CENTER_SERVER_CONFIG) `
    -WorkingDirectory $Root `
    -PassThru -NoNewWindow `
    -RedirectStandardOutput $stdout `
    -RedirectStandardError $stderr

$process.Id | Set-Content -Encoding ascii $pidFile
"[$(Get-Date -Format o)] pid=$($process.Id)" | Out-File -Append -Encoding utf8 $wrapperLog
$process.WaitForExit()
"[$(Get-Date -Format o)] exited code=$($process.ExitCode)" |
    Out-File -Append -Encoding utf8 $wrapperLog
Remove-Item $pidFile -Force -ErrorAction SilentlyContinue
