#!/usr/bin/env bash
# 一键启动 Center Server（venv + 依赖 + 握手）。
#
#   ./scripts/run.sh                 # 首次自动建 .venv 并安装基础依赖
#   ./scripts/run.sh --vla           # 额外安装 torch/lerobot 真实推理栈
#   ./scripts/run.sh --reinstall --vla --warmup
#   CENTER_SERVER_EDGE_URL=http://192.168.3.26 ./scripts/run.sh
#
# 额外参数会透传给 `python -m center_server`（如 --check/--warmup/--port）。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

INSTALL_VLA=0
REINSTALL=0
ARGS=()
for arg in "$@"; do
    case "$arg" in
        --vla) INSTALL_VLA=1 ;;
        --reinstall) REINSTALL=1 ;;
        *) ARGS+=("$arg") ;;
    esac
done

if [[ ! -d .venv ]]; then
    python3 -m venv .venv
    REINSTALL=1
fi
# shellcheck disable=SC1091
source .venv/bin/activate

if [[ "$REINSTALL" == "1" ]]; then
    python -m pip install --upgrade pip
    python -m pip install -r requirements.txt
    if [[ "$INSTALL_VLA" == "1" ]]; then
        python -m pip install -r requirements-vla.txt
    fi
elif [[ "$INSTALL_VLA" == "1" ]] && ! python -c "import torch, lerobot" >/dev/null 2>&1; then
    echo ">>> 安装真实推理依赖（torch/lerobot）"
    python -m pip install -r requirements-vla.txt
fi

HAS_CONFIG=0
for arg in "${ARGS[@]}"; do
    case "$arg" in
        --config|--config=*) HAS_CONFIG=1 ;;
    esac
done
if [[ "$HAS_CONFIG" == "0" ]]; then
    ARGS=(--config "${CENTER_SERVER_CONFIG:-config.yaml}" "${ARGS[@]}")
fi

exec python -m center_server "${ARGS[@]}"
