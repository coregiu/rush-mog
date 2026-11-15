# Rush-MOG Center Server —— Windows 一次性安装/更新
#
# 需要管理员 PowerShell（防火墙 + 开机任务）：
#   powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\install_windows.ps1
#
# 参数：
#   -SkipVenv        跳过 venv/基础依赖安装
#   -SkipFirewall    不创建 8000 端口防火墙规则
#   -SkipTask        不注册开机任务
#   -NoStart         注册后不立即启动
param(
    [switch]$SkipVenv,
    [switch]$SkipFirewall,
    [switch]$SkipTask,
    [switch]$NoStart
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$venv = Join-Path $Root ".venv"
$vpy = Join-Path $venv "Scripts\python.exe"
$taskName = "rush-mog-center"
$ruleName = "Rush-MOG Center Server (8000)"
$dataDir = "C:\ProgramData\rush-mog-center"

function Assert-Admin {
    $isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator
    )
    if (-not $isAdmin) {
        throw "请用管理员 PowerShell 运行（需要防火墙/开机任务权限）"
    }
}

Assert-Admin
New-Item -ItemType Directory -Force -Path $dataDir | Out-Null

if (-not $SkipVenv) {
    if (-not (Test-Path $vpy)) {
        $py310 = & py -3.10 -c "import sys; print(sys.executable)" 2>$null
        if (-not $py310) { throw "未找到 Python 3.10（py -3.10）" }
        Write-Output "== 创建 venv: $venv"
        & $py310 -m venv $venv
    }
    # Windows 中文 locale 下 pip 读 UTF-8 requirements 需要 PYTHONUTF8
    $env:PYTHONUTF8 = "1"
    $env:PYTHONIOENCODING = "utf-8"
    Write-Output "== 安装基础依赖"
    & $vpy -m pip install --disable-pip-version-check -r (Join-Path $Root "requirements.txt")
}

if (-not $SkipFirewall) {
    Write-Output "== 防火墙: 允许 192.168.3.0/24 -> TCP 8000"
    Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue |
        Remove-NetFirewallRule -ErrorAction SilentlyContinue
    New-NetFirewallRule -DisplayName $ruleName `
        -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8000 `
        -RemoteAddress 192.168.3.0/24 -Profile Any | Out-Null
}

if (-not $SkipTask) {
    Write-Output "== 注册开机任务: $taskName"
    $action = New-ScheduledTaskAction -Execute "powershell.exe" `
        -Argument "-NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File `"$PSScriptRoot\start_center.ps1`""
    $trigger = New-ScheduledTaskTrigger -AtStartup
    $principal = New-ScheduledTaskPrincipal -UserId "SYSTEM" -LogonType ServiceAccount -RunLevel Highest
    $settings = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries `
        -ExecutionTimeLimit ([TimeSpan]::Zero) -RestartCount 3 `
        -RestartInterval (New-TimeSpan -Minutes 1) -StartWhenAvailable
    Register-ScheduledTask -TaskName $taskName -Action $action -Trigger $trigger `
        -Principal $principal -Settings $settings -Force `
        -Description "Rush-MOG Center Server (SmolVLA HTTP inference + Edge handshake)" | Out-Null
    if (-not $NoStart) {
        Start-ScheduledTask -TaskName $taskName
    }
}

Write-Output "== 完成"
Write-Output "状态:  Get-ScheduledTask -TaskName $taskName | Select TaskName,State"
Write-Output "日志:  Get-Content $dataDir\stderr.log -Tail 50"
Write-Output "停止:  powershell -File deploy\windows\stop_center.ps1"
