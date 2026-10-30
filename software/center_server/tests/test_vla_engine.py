from center_server.core.vla_engine import VLAEngine
from center_server.core.data_preprocessor import SyncedDTO
import torch
import numpy as np

def test_vla_engine():
    cfg = {"device":"cpu", "confidence_threshold":0.6}
    eng = VLAEngine(cfg)
    dto = SyncedDTO(
        image=torch.rand(3,224,224),
        laser=np.random.rand(10),
        audio=torch.rand(64,40),
        odom={"x":0.0},
        joint={}
    )
    res = eng.predict(dto)
    assert res.confidence > 0
