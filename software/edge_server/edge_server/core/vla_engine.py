"""VLA 推理引擎（规则桩）。

当前仓库没有真实的 VLA 模型权重，DESIGN.md 也把模型服务列为外部依赖。
本文件实现一个可运行的规则桩（rule-based stub）：

- 单步底盘动作：前进/后退/左转/右转/左移/右移/停止；
- 长任务意图：建图、探索、搜索、导航到航点/坐标、回家。

接入真实模型时只需替换 _load_model / predict 内部实现，保持 predict(data)
返回 ActionIntentDTO 的接口形状即可。
"""

from __future__ import annotations

from dataclasses import dataclass

from .data_preprocessor import SyncedDTO
from .instruction_parser import InstructionParser
from ..utils.logger import get_logger


@dataclass
class ActionIntentDTO:
    task_id: str = ""
    target: str = "base"
    intent: str = "stop"
    value: float = 0.0
    confidence: float = 0.0
    reverse_request: bool = False
    # --- SLAM/导航/搜索扩展 ---
    task_type: str = "vla"
    object_label: str = ""
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_yaw: float = 0.0
    waypoint: str = ""


class VLAEngine:
    # 关键词 -> (意图, 目标)。按顺序匹配，停止类关键词优先。
    _RULES = (
        (("停止", "停下", "别走", "停", "stop", "halt"), "stop", "base"),
        (("左移", "向左平移", "strafe left", "strafe_left"), "strafe_left", "base"),
        (("右移", "向右平移", "strafe right", "strafe_right"), "strafe_right", "base"),
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
        self.parser = InstructionParser(
            self.cfg.get("instruction") or self.cfg.get("instruction_parser") or self.cfg
        )
        self.model = None
        self._logger = get_logger("edge_server.vla")
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

        # 1) 停止/单步动作优先，避免“停止前进”被识别成前进
        for keywords, intent, target in self._RULES:
            if any(keyword in instruction for keyword in keywords):
                value = 0.0
                if intent in ("forward", "backward"):
                    value = self.default_linear
                elif intent in ("turn_left", "turn_right"):
                    value = self.default_angular
                elif intent in ("strafe_left", "strafe_right"):
                    value = self.default_linear
                return ActionIntentDTO(
                    task_id=task_id,
                    target=target,
                    intent=intent,
                    value=value,
                    # 规则桩的置信度固定为 0.9；真实模型应返回 softmax 概率
                    confidence=0.9,
                )

        # 2) 长任务意图（建图/搜索/导航/回家）
        mission = self.parser.parse(instruction)
        if mission.is_mission and mission.confidence >= 0.7:
            return ActionIntentDTO(
                task_id=task_id,
                target="mission",
                intent=mission.mission_type,
                value=0.0,
                confidence=mission.confidence,
                task_type=mission.mission_type,
                object_label=mission.target_label,
                goal_x=mission.x,
                goal_y=mission.y,
                goal_yaw=mission.yaw,
                waypoint=mission.waypoint,
            )

        # 3) 无匹配指令：保持停止并给出低于阈值的置信度，交由上层请求确认
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
