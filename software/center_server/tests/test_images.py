import base64
import io

import numpy as np
import pytest
from PIL import Image

from center_server.errors import BadRequest, PayloadTooLarge
from center_server.images import decode_image, encode_image_b64


def _png_b64(width=8, height=6, color=(255, 0, 0)) -> str:
    image = Image.new("RGB", (width, height), color)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def test_decode_plain_base64():
    array = decode_image(_png_b64())
    assert array.shape == (3, 6, 8)
    assert array.dtype == np.float32
    assert array[0, 0, 0] == pytest.approx(1.0)
    assert array[1, 0, 0] == pytest.approx(0.0)


def test_decode_data_uri_and_dict():
    encoded = _png_b64()
    for value in (
        f"data:image/png;base64,{encoded}",
        {"data": encoded},
        {"data": f"data:image/png;base64,{encoded}"},
    ):
        array = decode_image(value)
        assert array.shape == (3, 6, 8)


def test_decode_rejects_corrupt_and_oversized():
    with pytest.raises(BadRequest):
        decode_image(base64.b64encode(b"not an image").decode())
    with pytest.raises(PayloadTooLarge):
        decode_image(_png_b64(), max_bytes=10)


def test_encode_roundtrip():
    image = np.zeros((3, 6, 8), dtype=np.float32)
    image[0] = 1.0
    data_uri = encode_image_b64(image)
    assert data_uri.startswith("data:image/png;base64,")
    decoded = decode_image(data_uri)
    assert decoded.shape == (3, 6, 8)
    assert decoded[0, 0, 0] == pytest.approx(1.0)


def test_encode_accepts_hwc_uint8():
    image = np.zeros((6, 8, 3), dtype=np.uint8)
    image[:, :, 2] = 255
    decoded = decode_image(encode_image_b64(image))
    assert decoded.shape == (3, 6, 8)
    assert decoded[2, 0, 0] == pytest.approx(1.0)
