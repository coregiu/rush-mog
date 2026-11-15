# Rush-MOG Center Server（SmolVLA HTTP 推理）镜像
#
# CPU 基础镜像（默认，可跑 stub/低速推理）：
#   docker build -f deploy/Dockerfile -t rush-mog/center-server:0.2.0 .
#
# GPU 主机（构建时安装 torch/lerobot；CUDA 版本按主机驱动选择基础镜像）：
#   docker build -f deploy/Dockerfile \
#     --build-arg BASE_IMAGE=nvidia/cuda:12.4.1-cudnn-runtime-ubuntu22.04 \
#     --build-arg INSTALL_VLA=1 \
#     -t rush-mog/center-server:0.2.0-gpu .
#
# 说明：torch 的 CUDA wheel 约 2GB+，首次构建较慢；离线部署建议先在
# 可联网机器执行 scripts/download_model.sh，再挂载 /models（见 deploy/README.md）。

ARG BASE_IMAGE=python:3.11-slim
FROM ${BASE_IMAGE}

ARG INSTALL_VLA=0
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    HF_HOME=/home/app/.cache/huggingface \
    CENTER_SERVER_CONFIG=/app/config.yaml

# nvidia/cuda 基础镜像没有 pip，需要先安装 python3/pip
RUN if ! command -v pip3 >/dev/null 2>&1; then \
        apt-get update && \
        DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
            python3 python3-pip python3-venv curl && \
        rm -rf /var/lib/apt/lists/* && \
        ln -sf /usr/bin/python3 /usr/local/bin/python; \
    fi

WORKDIR /app

COPY requirements.txt requirements-vla.txt ./
RUN pip3 install --no-cache-dir -r requirements.txt
RUN if [ "$INSTALL_VLA" = "1" ]; then \
        pip3 install --no-cache-dir -r requirements-vla.txt; \
    else \
        echo "INSTALL_VLA=0: 只安装基础依赖（如需真实模型加 --build-arg INSTALL_VLA=1）"; \
    fi

COPY pyproject.toml README.md ./
COPY center_server ./center_server
COPY config.yaml config.stub.yaml ./
RUN pip3 install --no-cache-dir --no-deps -e .

# 非 root 运行；模型目录/缓存由 compose 或 -v 挂载
RUN useradd --create-home --shell /usr/sbin/nologin app && \
    mkdir -p /home/app/.cache/huggingface /models && \
    chown -R app:app /home/app /models
USER app

EXPOSE 8000

# /healthz 仅表示进程存活；模型就绪看 /v1/ready（预热后 200）
HEALTHCHECK --interval=15s --timeout=3s --start-period=20s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8000/healthz', timeout=2)"

ENTRYPOINT ["python", "-m", "center_server"]
CMD ["--config", "/app/config.yaml"]
