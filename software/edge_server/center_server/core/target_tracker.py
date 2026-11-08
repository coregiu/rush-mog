"""轻量目标跟踪（IoU 匹配 + 轨迹稳定度），用于确认“同一个目标”而不是闪烁检测。"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass
class DetectionInput:
    label: str
    confidence: float
    bbox: Tuple[float, float, float, float]
    position: Optional[Tuple[float, float]] = None
    position_frame: str = ""
    distance_m: float = 0.0
    bearing_rad: float = 0.0
    distance_source: str = "none"


@dataclass
class Track:
    track_id: str
    label: str
    bbox: Tuple[float, float, float, float]
    confidence: float
    position: Optional[Tuple[float, float]] = None
    position_frame: str = ""
    hits: int = 1
    misses: int = 0
    first_seen: float = 0.0
    last_seen: float = 0.0
    history: Deque[Tuple[float, float]] = field(default_factory=lambda: deque(maxlen=5))
    distance_m: float = 0.0
    bearing_rad: float = 0.0
    distance_source: str = "none"

    def stable(self, radius_m: float = 0.3) -> bool:
        if self.position is None or len(self.history) < 2:
            return True
        xs = [p[0] for p in self.history]
        ys = [p[1] for p in self.history]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        return all(math.hypot(x - cx, y - cy) <= radius_m for x, y in zip(xs, ys))

    @property
    def position_tuple(self) -> Optional[Tuple[float, float]]:
        return self.position


def iou(a: Sequence[float], b: Sequence[float]) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter
    return inter / union if union > 1e-9 else 0.0


class TargetTracker:
    def __init__(
        self,
        iou_threshold: float = 0.25,
        max_age_sec: float = 1.0,
        max_tracks: int = 32,
        history_size: int = 5,
    ):
        self.iou_threshold = float(iou_threshold)
        self.max_age = float(max_age_sec)
        self.max_tracks = int(max_tracks)
        self.history_size = int(history_size)
        self.tracks: Dict[str, Track] = {}
        self._next_id = 1
        self._last_time = 0.0

    def update(self, detections: Iterable[DetectionInput], now: float) -> List[Track]:
        now = float(now)
        self._last_time = now
        dets = list(detections)
        candidates: List[Tuple[float, int, str]] = []
        for ti, (track_id, track) in enumerate(self.tracks.items()):
            for di, det in enumerate(dets):
                if det.label != track.label:
                    continue
                score = iou(track.bbox, det.bbox)
                if score >= self.iou_threshold:
                    candidates.append((score, di, track_id))
        candidates.sort(reverse=True)

        matched_dets = set()
        matched_tracks = set()
        for score, di, track_id in candidates:
            if di in matched_dets or track_id in matched_tracks:
                continue
            matched_dets.add(di)
            matched_tracks.add(track_id)
            self._apply(self.tracks[track_id], dets[di], now)

        existing_ids = set(self.tracks.keys())
        for di, det in enumerate(dets):
            if di in matched_dets:
                continue
            track = self._new_track(det, now)
            self.tracks[track.track_id] = track

        for track_id in existing_ids:
            track = self.tracks.get(track_id)
            if track is None:
                continue
            if track_id not in matched_tracks:
                track.misses += 1
            if now - track.last_seen > self.max_age:
                self.tracks.pop(track_id, None)

        # 控制轨迹数量
        if len(self.tracks) > self.max_tracks:
            ordered = sorted(self.tracks.values(), key=lambda t: t.last_seen, reverse=True)
            self.tracks = {t.track_id: t for t in ordered[: self.max_tracks]}
        return list(self.tracks.values())

    def best(
        self,
        label: Optional[str] = None,
        min_confidence: float = 0.0,
        min_hits: int = 1,
        now: Optional[float] = None,
    ) -> Optional[Track]:
        candidates = [
            t
            for t in self.tracks.values()
            if (label is None or t.label == label)
            and t.confidence >= float(min_confidence)
            and t.hits >= int(min_hits)
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda t: (t.confidence, t.hits, t.last_seen))

    def get(self, track_id: str) -> Optional[Track]:
        return self.tracks.get(track_id)

    def reset(self):
        self.tracks.clear()

    # ---------------------------------------------------------------- internal
    def _new_track(self, det: DetectionInput, now: float) -> Track:
        track_id = f"trk_{self._next_id:04d}"
        self._next_id += 1
        return Track(
            track_id=track_id,
            label=det.label,
            bbox=tuple(det.bbox),
            confidence=float(det.confidence),
            position=det.position,
            position_frame=det.position_frame,
            hits=1,
            misses=0,
            first_seen=now,
            last_seen=now,
            history=deque([det.position] if det.position else [], maxlen=self.history_size),
            distance_m=det.distance_m,
            bearing_rad=det.bearing_rad,
            distance_source=det.distance_source,
        )

    def _apply(self, track: Track, det: DetectionInput, now: float):
        track.bbox = tuple(det.bbox)
        track.confidence = float(det.confidence)
        track.distance_m = float(det.distance_m)
        track.bearing_rad = float(det.bearing_rad)
        track.distance_source = det.distance_source
        if det.position is not None:
            track.position = det.position
            track.position_frame = det.position_frame
            track.history.append(det.position)
        track.hits += 1
        track.misses = 0
        track.last_seen = now
