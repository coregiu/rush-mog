"""多模态数据预处理。

将 edge_server 上传的 ROS2 原始数据转换为 VLA 引擎可消费的 numpy 特征。
原实现依赖 torch / librosa / pydantic，但当前环境未安装这些包且 DESIGN.md
并不要求特定张量框架，因此这里改用 numpy 实现；接口保持不变，后续接入
真实模型时可在 VLAEngine 内部再转换为 torch.Tensor。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

try:  # OpenCV 在 ROS Humble 环境通常随 python3-opencv 安装
    import cv2
except ImportError:  # pragma: no cover - 兜底分支
    cv2 = None


@dataclass
class SyncedDTO:
    image: np.ndarray
    laser: np.ndarray
    audio: np.ndarray
    odom: Dict[str, float] = field(default_factory=dict)
    joint: Dict[str, float] = field(default_factory=dict)
    task_id: str = ""
    instruction: str = ""


class MultiModalPreprocessor:
    def __init__(
        self,
        img_size=(224, 224),
        n_mels: int = 64,
        n_fft: int = 512,
        hop_length: int = 160,
        max_audio_seconds: float = 5.0,
    ):
        self.img_size = (int(img_size[0]), int(img_size[1]))
        self.n_mels = int(n_mels)
        self.n_fft = int(n_fft)
        self.hop_length = int(hop_length)
        self.max_audio_seconds = float(max_audio_seconds)
        self._mel_fb: Optional[np.ndarray] = None
        self._mel_fb_sr: Optional[int] = None

    # ------------------------------------------------------------------ image
    def process_image(self, cv_img: np.ndarray) -> np.ndarray:
        """BGR/RGB/gray 图像 -> float32 CHW，范围 [0, 1]。"""
        img = np.asarray(cv_img)
        if img.size == 0:
            raise ValueError("empty image")
        if img.ndim == 2:
            img = np.repeat(img[:, :, None], 3, axis=2)
        elif img.ndim == 3 and img.shape[2] == 1:
            img = np.repeat(img, 3, axis=2)
        elif img.ndim == 3 and img.shape[2] >= 3:
            img = img[:, :, :3]
        else:
            raise ValueError(f"unsupported image shape: {img.shape}")

        img = self._resize(img)
        if np.issubdtype(img.dtype, np.integer):
            img = img.astype(np.float32) / 255.0
        else:
            img = img.astype(np.float32)
        img = np.clip(img, 0.0, 1.0)
        return np.ascontiguousarray(np.transpose(img, (2, 0, 1)))

    def _resize(self, img: np.ndarray) -> np.ndarray:
        height, width = self.img_size
        if img.shape[0] == height and img.shape[1] == width:
            return img
        if cv2 is not None:
            return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
        # 无 OpenCV 时的最近邻缩放，保证节点仍可运行
        rows = (np.arange(height) * img.shape[0] // height).clip(0, img.shape[0] - 1)
        cols = (np.arange(width) * img.shape[1] // width).clip(0, img.shape[1] - 1)
        return img[rows][:, cols]

    # ------------------------------------------------------------------ laser
    def process_laser(self, scan) -> np.ndarray:
        return np.asarray(scan, dtype=np.float32).reshape(-1)

    # ------------------------------------------------------------------ audio
    def process_audio(self, audio_bytes, sample_rate: int = 16000) -> np.ndarray:
        """PCM s16le 字节流 -> log-mel 特征，shape=(n_mels, frames)。"""
        raw = bytes(audio_bytes)
        if not raw:
            return np.zeros((self.n_mels, 1), dtype=np.float32)
        y = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
        sample_rate = int(sample_rate) if sample_rate and sample_rate > 0 else 16000
        max_len = int(self.max_audio_seconds * sample_rate)
        if max_len > 0 and y.size > max_len:
            y = y[:max_len]
        return self._log_mel(y, sample_rate)

    def _log_mel(self, y: np.ndarray, sample_rate: int) -> np.ndarray:
        if y.size == 0:
            return np.zeros((self.n_mels, 1), dtype=np.float32)
        if y.size < self.n_fft:
            y = np.pad(y, (0, self.n_fft - y.size))
        frames = np.lib.stride_tricks.sliding_window_view(y, self.n_fft)[
            :: self.hop_length
        ]
        if frames.size == 0:
            return np.zeros((self.n_mels, 1), dtype=np.float32)
        window = np.hanning(self.n_fft).astype(np.float32)
        spectrum = np.abs(np.fft.rfft(frames * window, axis=1)) ** 2
        mel = spectrum @ self._mel_filterbank(sample_rate).T
        log_mel = np.log(mel + 1e-10)
        return np.ascontiguousarray(log_mel.T.astype(np.float32))

    def _mel_filterbank(self, sample_rate: int) -> np.ndarray:
        if self._mel_fb is not None and self._mel_fb_sr == sample_rate:
            return self._mel_fb

        def hz_to_mel(hz):
            return 2595.0 * np.log10(1.0 + hz / 700.0)

        def mel_to_hz(mel):
            return 700.0 * (10.0 ** (mel / 2595.0) - 1.0)

        n_freqs = self.n_fft // 2 + 1
        fft_freqs = np.linspace(0.0, sample_rate / 2.0, n_freqs)
        mel_points = np.linspace(
            hz_to_mel(0.0), hz_to_mel(sample_rate / 2.0), self.n_mels + 2
        )
        hz_points = mel_to_hz(mel_points)
        fb = np.zeros((self.n_mels, n_freqs), dtype=np.float32)
        for m in range(1, self.n_mels + 1):
            left, center, right = hz_points[m - 1], hz_points[m], hz_points[m + 1]
            if center <= left or right <= center:
                continue
            left_slope = (fft_freqs - left) / (center - left)
            right_slope = (right - fft_freqs) / (right - center)
            fb[m - 1] = np.maximum(0.0, np.minimum(left_slope, right_slope))
        empty = fb.sum(axis=1) == 0
        fb[empty] = 1.0 / n_freqs
        self._mel_fb = fb
        self._mel_fb_sr = sample_rate
        return fb

    # ------------------------------------------------------------------- pack
    def pack(
        self,
        img_tensor,
        laser_arr,
        audio_tensor,
        odom_data: Optional[dict] = None,
        joint_data: Optional[dict] = None,
        task_id: str = "",
        instruction: str = "",
    ) -> SyncedDTO:
        return SyncedDTO(
            image=img_tensor,
            laser=laser_arr,
            audio=audio_tensor,
            odom=dict(odom_data or {}),
            joint=dict(joint_data or {}),
            task_id=task_id,
            instruction=instruction,
        )
