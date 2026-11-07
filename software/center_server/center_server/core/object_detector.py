"""目标检测器：颜色块检测（默认，无模型依赖）+ 可选 YOLO 适配。

接入真实检测模型时，实现 detect(image_bgr) -> list[Detection] 即可，检测结果
统一为归一化 bbox，后续由 TargetLocalizer 做方位/距离/坐标投影。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: tuple  # (x_min, y_min, x_max, y_max) normalized


class ColorBlobDetector:
    """按 HSV 阈值检测目标。适合贴色块/特定颜色物体的演示与联调。"""

    def __init__(self, targets: Optional[Dict[str, dict]] = None):
        self.targets = dict(targets or {})

    def detect(self, image_bgr) -> List[Detection]:
        if cv2 is None or image_bgr is None or getattr(image_bgr, "size", 0) == 0:
            return []
        height, width = image_bgr.shape[:2]
        if height <= 0 or width <= 0:
            return []
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        detections: List[Detection] = []
        for label, cfg in self.targets.items():
            hsv_min = cfg.get("hsv_min")
            hsv_max = cfg.get("hsv_max")
            if not hsv_min or not hsv_max:
                continue
            mask = cv2.inRange(
                hsv,
                _hsv_array(hsv_min),
                _hsv_array(hsv_max),
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            min_area = float(cfg.get("min_area_ratio", 0.002)) * width * height
            max_area = float(cfg.get("max_area_ratio", 0.8)) * width * height
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < min_area or area > max_area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if w <= 1 or h <= 1:
                    continue
                area_ratio = area / float(width * height)
                confidence = min(0.95, 0.55 + area_ratio * 4.0)
                detections.append(
                    Detection(
                        label=label,
                        confidence=confidence,
                        bbox=(
                            x / float(width),
                            y / float(height),
                            (x + w) / float(width),
                            (y + h) / float(height),
                        ),
                    )
                )
        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections


class YoloDetector:
    """可选 YOLO 适配器。需要自行安装 ultralytics 并提供模型文件。"""

    def __init__(self, model_path: str, allowed_labels=None, conf: float = 0.35):
        from ultralytics import YOLO  # pragma: no cover - 可选依赖

        self.model = YOLO(model_path)
        self.allowed = set(allowed_labels or [])
        self.conf = float(conf)

    def detect(self, image_bgr) -> List[Detection]:  # pragma: no cover
        import numpy as np

        height, width = image_bgr.shape[:2]
        results = self.model.predict(image_bgr, conf=self.conf, verbose=False)
        detections: List[Detection] = []
        for result in results:
            names = result.names or {}
            for box in result.boxes:
                cls_id = int(box.cls[0])
                label = names.get(cls_id, str(cls_id))
                if self.allowed and label not in self.allowed:
                    continue
                x0, y0, x1, y1 = [float(v) for v in box.xyxy[0].tolist()]
                detections.append(
                    Detection(
                        label=label,
                        confidence=float(box.conf[0]),
                        bbox=(
                            max(0.0, x0 / width),
                            max(0.0, y0 / height),
                            min(1.0, x1 / width),
                            min(1.0, y1 / height),
                        ),
                    )
                )
        detections.sort(key=lambda d: d.confidence, reverse=True)
        return detections


def make_detector(cfg: Optional[dict], logger=None):
    cfg = cfg or {}
    kind = str(cfg.get("detector", "color")).lower()
    targets = cfg.get("targets") or {}
    if kind == "none":
        return None
    if kind == "yolo":
        try:
            return YoloDetector(
                str(cfg.get("model_path", "")),
                allowed_labels=cfg.get("allowed_labels") or targets.keys(),
                conf=float(cfg.get("min_confidence", 0.35)),
            )
        except Exception as exc:  # noqa: BLE001
            if logger is not None:
                logger.warning(f"YOLO unavailable ({exc}); falling back to color detector")
            return ColorBlobDetector(targets)
    return ColorBlobDetector(targets)


def _hsv_array(values):
    import numpy as np

    arr = np.asarray(values, dtype=np.uint8).reshape(1, 3)
    return arr
