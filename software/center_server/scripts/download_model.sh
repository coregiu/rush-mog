#!/usr/bin/env bash
# 下载/更新 SmolVLA checkpoint 到本地目录（离线部署用）。
#
#   ./scripts/download_model.sh                                   # lerobot/smolvla_base -> ./models/smolvla_base
#   ./scripts/download_model.sh my-org/rush-mog-smolvla ./models/rush-mog
#
# 也可以用 HF_TOKEN 下载私有仓库。
set -euo pipefail

MODEL="${1:-lerobot/smolvla_base}"
DEST="${2:-$(cd "$(dirname "$0")/.." && pwd)/models/$(basename "$MODEL")}"

mkdir -p "$DEST"
echo ">>> 下载 ${MODEL} -> ${DEST}"

if command -v hf >/dev/null 2>&1; then
    hf download "$MODEL" --local-dir "$DEST"
elif command -v huggingface-cli >/dev/null 2>&1; then
    huggingface-cli download "$MODEL" --local-dir "$DEST"
else
    python3 - "$MODEL" "$DEST" <<'PY'
import sys

from huggingface_hub import snapshot_download

model, dest = sys.argv[1], sys.argv[2]
snapshot_download(repo_id=model, local_dir=dest)
PY
fi

echo ">>> 完成。检查关键文件："
ls -lh "$DEST" | head -20
echo
echo "配置示例："
echo "  model:"
echo "    path: \"${DEST}\""
echo "    local_files_only: true"
