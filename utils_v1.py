import math
import numpy as np


def quat2axisangle(quat):
    """
    Converts quaternion to axis-angle format.
    Returns a unit vector direction scaled by its angle in radians.

    Args:
        quat (np.array): (x,y,z,w) vec4 float angles

    Returns:
        np.array: (ax,ay,az) axis-angle exponential coordinates
    """
    # clip quaternion
    if quat[3] > 1.0:
        quat[3] = 1.0
    elif quat[3] < -1.0:
        quat[3] = -1.0

    den = np.sqrt(1.0 - quat[3] * quat[3])
    if math.isclose(den, 0.0):
        # This is (close to) a zero degree rotation, immediately return
        return np.zeros(3)

    return (quat[:3] * 2.0 * math.acos(quat[3])) / den


def axisangle2quat(vec):
    """
    Converts scaled axis-angle to quat.

    Args:
        vec (np.array): (ax,ay,az) axis-angle exponential coordinates

    Returns:
        np.array: (x,y,z,w) vec4 float angles
    """
    # Grab angle
    angle = np.linalg.norm(vec)

    # handle zero-rotation case
    if math.isclose(angle, 0.0):
        return np.array([0.0, 0.0, 0.0, 1.0])

    # make sure that axis is a unit vector
    axis = vec / angle

    q = np.zeros(4)
    q[3] = np.cos(angle / 2.0)
    q[:3] = axis * np.sin(angle / 2.0)
    return q


import requests
from io import BytesIO
from typing import Dict, Optional, Tuple

def _bytes_to_file_tuple(image_bytes: bytes, filename: str = "image.jpg") -> Tuple[str, BytesIO, str]:
    """Convert bytes (JPG encoded) into a tuple for multipart upload."""
    return filename, BytesIO(image_bytes), "image/jpeg"

def get_answer_from_server(
    rgb_main: bytes,
    question: str,
    rgb_wrist: Optional[bytes] = None,
    server_url: str = "http://localhost:8000",
    timeout: int = 120,
) -> dict:
    """
    Send a request to the Qwen3-VL FastAPI server and get the answer.

    Args:
        rgb_main: Bytes of JPG-encoded main RGB image (required).
        question: Text prompt/question for the model (required).
        rgb_wrist: Optional bytes of JPG-encoded wrist camera image.
        server_url: Base URL of the FastAPI server. Defaults to "http://localhost:8000".
        timeout: Request timeout in seconds. Defaults to 120.

    Returns:
        dict: The JSON response from the server containing output_text and generation_time.

    Raises:
        requests.RequestException: If the request fails.
    """
    data = {
        "question": question,
    }

    files: Dict[str, Tuple[str, BytesIO, str]] = {
        "rgb_main": _bytes_to_file_tuple(rgb_main, "rgb_main.jpg"),
    }
    if rgb_wrist:
        files["rgb_wrist"] = _bytes_to_file_tuple(rgb_wrist, "rgb_wrist.jpg")

    response = requests.post(
        f"{server_url.rstrip('/')}/generate",
        data=data,
        files=files,
        timeout=timeout,
    )

    response.raise_for_status()
    return dict(response.json())