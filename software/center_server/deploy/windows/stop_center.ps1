# Rush-MOG Center Server —— Windows 停止脚本
$ErrorActionPreference = "Continue"

$DataDir = "C:\ProgramData\rush-mog-center"
$pidFile = Join-Path $DataDir "center.pid"

if (Test-Path $pidFile) {
    $processId = [int](Get-Content $pidFile -ErrorAction SilentlyContinue)
    if ($processId -gt 0) {
        Write-Output "stopping pid=$processId"
        Stop-Process -Id $processId -Force -ErrorAction SilentlyContinue
    }
    Remove-Item $pidFile -Force -ErrorAction SilentlyContinue
}

# 兜底：包装脚本被中断时按命令行清理本项目的 python 进程
Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
    Where-Object {
        $_.Name -like "python*" -and
        $_.CommandLine -like "*center_server*" -and
        $_.CommandLine -like "*--config*"
    } |
    ForEach-Object {
        Write-Output "stopping stray pid=$($_.ProcessId)"
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }

Write-Output "center server stopped"
