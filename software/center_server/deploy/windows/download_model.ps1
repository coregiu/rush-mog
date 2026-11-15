# Rush-MOG Center Server —— 下载 SmolVLA 权重（Windows）
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File deploy\windows\download_model.ps1
#   powershell ... -Repo my-org/rush-mog-smolvla -Dest F:\models\rush-mog-smolvla
#
# 默认使用 https://hf-mirror.com（国内镜像），目标 <repo>\models\smolvla_base。
param(
    [string]$Repo = "lerobot/smolvla_base",
    [string]$Dest = "",
    [string]$Endpoint = "https://hf-mirror.com"
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$vpy = Join-Path $Root ".venv\Scripts\python.exe"
if (-not (Test-Path $vpy)) { throw "未找到 $vpy；先执行 install_vla.ps1" }

if (-not $Dest) {
    $repoRoot = (Resolve-Path (Join-Path $Root "..\..")).Path
    $Dest = Join-Path (Join-Path $repoRoot "models") (Split-Path $Repo -Leaf)
}
New-Item -ItemType Directory -Force -Path $Dest | Out-Null

$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"
$env:HF_ENDPOINT = $Endpoint
$env:HF_HOME = "C:\ProgramData\rush-mog-center\hf-cache"
$env:HF_HUB_ENABLE_HF_TRANSFER = "0"

Write-Output "== 下载 $Repo -> $Dest (endpoint=$Endpoint)"
& $vpy -c "import sys; from huggingface_hub import snapshot_download; snapshot_download(repo_id=sys.argv[1], local_dir=sys.argv[2])" $Repo $Dest

Write-Output "== 完成，目录内容："
Get-ChildItem $Dest | Select-Object -First 20 Name,Length
