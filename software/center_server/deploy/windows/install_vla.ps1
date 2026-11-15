# Rush-MOG Center Server —— 安装真实 SmolVLA 推理栈（Windows / NVIDIA GPU）
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\install_vla.ps1
#   powershell ... -Cpu                 # 只装 CPU 版（无 GPU/显存不足）
#
# 背景：pip 直接下载 2.5GB 的 CUDA wheel 在国内镜像上容易停滞；本脚本改用
# curl 断点续传到 <repo>\wheels\，再从本地 wheel 安装（可反复续传）。
# 默认：torch 2.6.0+cu124 / torchvision 0.21.0+cu124（兼容 lerobot 0.4.4、
# GTX 10 系 sm_61；显存不足时同一份 torch 也可跑 CPU）。
param(
    [string]$TorchVersion = "2.6.0+cu124",
    [string]$TorchVisionVersion = "0.21.0+cu124",
    [string]$WheelIndex = "https://mirrors.aliyun.com/pytorch-wheels/cu124/",
    [string]$PypiIndex = "https://mirrors.aliyun.com/pypi/simple/",
    [switch]$Cpu
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$RepoRoot = (Resolve-Path (Join-Path $Root "..\..")).Path
$vpy = Join-Path $Root ".venv\Scripts\python.exe"
if (-not (Test-Path $vpy)) { throw "未找到 $vpy；先执行 install_windows.ps1" }

$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"

Write-Output "== 升级 pip/setuptools/wheel"
& $vpy -m pip install --disable-pip-version-check --upgrade pip setuptools wheel

if ($Cpu) {
    Write-Output "== 安装 CPU 版 torch/torchvision（PyPI）"
    & $vpy -m pip install --disable-pip-version-check `
        --index-url $PypiIndex "torch==2.6.0" "torchvision==0.21.0"
} else {
    $wheelDir = Join-Path $RepoRoot "wheels"
    New-Item -ItemType Directory -Force -Path $wheelDir | Out-Null
    $torchName = "torch-$TorchVersion-cp310-cp310-win_amd64.whl"
    $tvName = "torchvision-$TorchVisionVersion-cp310-cp310-win_amd64.whl"
    $torchFile = Join-Path $wheelDir $torchName
    $tvFile = Join-Path $wheelDir $tvName
    $torchUrl = $WheelIndex + $torchName.Replace("+", "%2B")
    $tvUrl = $WheelIndex + $tvName.Replace("+", "%2B")

    function Get-Wheel([string]$Url, [string]$Path, [int]$MinMB) {
        if ((Test-Path $Path) -and ((Get-Item $Path).Length -gt ($MinMB * 1MB))) {
            Write-Output "== 已存在: $Path（$([math]::Round((Get-Item $Path).Length/1MB)) MB）"
            return
        }
        Write-Output "== 下载: $Url -> $Path（支持断点续传）"
        & curl.exe -L --retry 5 --retry-delay 3 -C - -o $Path $Url
        if ($LASTEXITCODE -ne 0) { throw "下载失败: $Url" }
        if ((Get-Item $Path).Length -lt ($MinMB * 1MB)) { throw "文件不完整: $Path" }
    }

    Get-Wheel $torchUrl $torchFile 2000
    Get-Wheel $tvUrl $tvFile 4

    Write-Output "== 从本地 wheel 安装 torch/torchvision"
    & $vpy -m pip install --disable-pip-version-check `
        --index-url $PypiIndex $torchFile $tvFile
}

Write-Output "== 安装 requirements-vla.txt（lerobot[smolvla] 等）"
& $vpy -m pip install --disable-pip-version-check `
    --index-url $PypiIndex -r (Join-Path $Root "requirements-vla.txt")

Write-Output "== 校验"
& $vpy -c "import torch, torchvision; print('torch', torch.__version__, 'cuda_available', torch.cuda.is_available(), 'cuda_version', torch.version.cuda); print('torchvision', torchvision.__version__)"
& $vpy -c "import lerobot; print('lerobot', getattr(lerobot, '__version__', 'ok'))"
