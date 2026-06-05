"""Minimal ROS Image <-> OpenCV conversion without cv_bridge."""

from __future__ import annotations

import cv2
import numpy as np

try:
    from sensor_msgs.msg import Image
except ModuleNotFoundError:
    Image = None


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
        "8uc1": 1,
        "8uc3": 3,
        "8uc4": 4,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    data = np.frombuffer(msg.data, dtype=np.uint8)
    if channels == 1:
        rows = data.reshape((msg.height, msg.step))
        gray = rows[:, : msg.width].copy()
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    row_pixels = msg.width * channels
    rows = data.reshape((msg.height, msg.step))
    image = rows[:, :row_pixels].reshape((msg.height, msg.width, channels)).copy()

    if encoding in ("bgr8", "8uc3"):
        return image
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding in ("rgba8", "8uc4"):
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def cv2_to_image_msg(array: np.ndarray, encoding: str, header) -> Image:
    if Image is None:
        raise RuntimeError("sensor_msgs is not available; source ROS2 before ROS topic mode.")
    if not array.flags["C_CONTIGUOUS"]:
        array = np.ascontiguousarray(array)

    msg = Image()
    msg.header = header
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = 1 if array.ndim == 2 else int(array.shape[2])
    msg.step = int(msg.width * channels * array.dtype.itemsize)
    msg.data = array.tobytes()
    return msg
