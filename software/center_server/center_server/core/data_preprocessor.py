import cv2
import numpy as np
import torch
import librosa
from pydantic import BaseModel

class SyncedDTO(BaseModel):
    image: torch.Tensor
    laser: np.ndarray
    audio: torch.Tensor
    odom: dict
    joint: dict

class MultiModalPreprocessor:
    def __init__(self, img_size=(224,224)):
        self.img_size = img_size

    def process_image(self, cv_img: np.ndarray) -> torch.Tensor:
        img = cv2.resize(cv_img, self.img_size)
        img = img.astype(np.float32)/255.0
        tensor = torch.from_numpy(img).permute(2,0,1)
        return tensor

    def process_laser(self, scan: list) -> np.ndarray:
        return np.array(scan, dtype=np.float32)

    def process_audio(self, audio_bytes: bytes, sr=16000) -> torch.Tensor:
        y = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32)/32768.0
        mel = librosa.feature.melspectrogram(y=y, sr=sr, n_mels=64)
        mel_db = librosa.power_to_db(mel, ref=np.max)
        return torch.from_numpy(mel_db)

    def pack(self, img_tensor, laser_arr, audio_tensor, odom_data, joint_data) -> SyncedDTO:
        return SyncedDTO(
            image=img_tensor,
            laser=laser_arr,
            audio=audio_tensor,
            odom=odom_data,
            joint=joint_data
        )
