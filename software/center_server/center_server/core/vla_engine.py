"""VLA 推理引擎。

当前仓库没有真实的 VLA 模型权重，DESIGN.md 也把模型服务列为外部依赖。
因此本文件实现一个可运行的规则桩（rule-based stub）：根据任务指令文本输出
动作意图，保持与 DESIGN.md 5.3 节一致的 predict(image/radar/audio/state)
接口形状。接入真实模型时只需替换 _load_model / predict 内部实现。
"""

from __future__ import annotations

from dataclasses import dataclass

from .data_preprocessor import SyncedDTO
from ..utils.logger import get_logger


@dataclass
class ActionIntentDTO:
    task_id: str = ""
    target: str = "base"
    intent: str = "stop"
    value: float = 0.0
    confidence: float = 0.0
    reverse_request: bool = False


class VLAEngine:
    # 关键词 -> (意图, 目标)。按顺序匹配，停止类关键词优先。
    _RULES = (
        (("停止", "停下", "别走", "停", "stop", "halt"), "stop", "base"),
        (("后退", "向后", "倒车", "backward", "back"), "backward", "base"),
        (("左转", "向左", "left"), "turn_left", "base"),
        (("右转", "向右", "right"), "turn_right", "base"),
        (("前进", "向前", "前走", "forward", "go"), "forward", "base"),
    )

    def __init__(self, cfg):
        self.cfg = cfg or {}
        self.device = str(self.cfg.get("device", "cpu"))
        self.thresh = float(self.cfg.get("confidence_threshold", 0.6))
        self.default_linear = float(self.cfg.get("default_linear_speed", 0.3))
        self.default_angular = float(self.cfg.get("default_angular_speed", 0.5))
        self.model = None
        self._logger = get_logger("center_server.vla")
        self._load_model()

    def _load_model(self):
        """真实部署时在这里加载策略模型。

        示例：
            import torch
            self.model = torch.load(model_path, map_location=self.device)
            self.model.eval()
        """
        model_name = str(self.cfg.get("model_name", "")).strip()
        if not model_name:
            return
        self._logger.info(
            "VLA model '%s' is not bundled; running rule-based stub on %s",
            model_name,
            self.device,
        )

    def predict(self, data: SyncedDTO) -> ActionIntentDTO:
        instruction = (getattr(data, "instruction", "") or "").strip().lower()
        task_id = getattr(data, "task_id", "") or ""

        for keywords, intent, target in self._RULES:
            if any(keyword in instruction for keyword in keywords):
                value = 0.0
                if intent in ("forward", "backward"):
                    value = self.default_linear
                elif intent in ("turn_left", "turn_right"):
                    value = self.default_angular
                return ActionIntentDTO(
                    task_id=task_id,
                    target=target,
                    intent=intent,
                    value=value,
                    # 规则桩的置信度固定为 0.9；真实模型应返回 softmax 概率
                    confidence=0.9,
                )

        # 无匹配指令：保持停止并给出低于阈值的置信度，交由上层请求确认
        return ActionIntentDTO(
            task_id=task_id,
            target="base",
            intent="stop",
            value=0.0,
            confidence=0.5,
        )

    def unload(self):
        self.model = None
        self._logger.info("VLA engine unloaded")
