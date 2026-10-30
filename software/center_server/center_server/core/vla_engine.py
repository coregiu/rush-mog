import torch
from pydantic import BaseModel
from .data_preprocessor import SyncedDTO

class ActionIntentDTO(BaseModel):
    task_id: str
    target: str
    intent: str
    value: float
    confidence: float

class VLAEngine:
    def __init__(self, cfg):
        self.cfg = cfg
        self.device = torch.device(cfg["device"])
        self.thresh = cfg["confidence_threshold"]
        self.model = None
        self._load_model()

    def _load_model(self):
        # 这里替换成真实VLA策略加载
        # self.model = AutoModel.from_pretrained(...)
        pass

    def predict(self, data: SyncedDTO) -> ActionIntentDTO:
        with torch.no_grad():
            # dummy inference，实际替换为模型前向
            intent = ActionIntentDTO(
                task_id="dummy",
                target="robot",
                intent="stop",
                value=0.0,
                confidence=0.9
            )
        return intent

    def unload(self):
        del self.model
        torch.cuda.empty_cache()
