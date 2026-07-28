"""Camera intrinsics as plain Python. No YAML, no calibration files.

Three ways to get K and D, best first:

1. sensor_msgs/CameraInfo. The k4a and realsense ROS drivers both publish
   factory intrinsics on .../camera_info. Use them if you can - they are per
   unit, not per model. See intrinsics_from_camera_info().

2. Run calibrate_camera.py once, paste the printed dict into PRESETS below.

3. Fall back to the nominal-FOV estimates below. These assume a perfect
   pinhole with zero distortion and the datasheet field of view. Good enough
   to verify detection works and to get distances within a few percent.
   Not good enough to grasp with.
"""

import numpy as np


def K_from_fov(width: int, height: int, hfov_deg: float,
               vfov_deg: float = None) -> np.ndarray:
    """Pinhole camera matrix from image size and field of view."""
    fx = (width / 2.0) / np.tan(np.deg2rad(hfov_deg) / 2.0)
    if vfov_deg is None:
        fy = fx                      # square pixels
    else:
        fy = (height / 2.0) / np.tan(np.deg2rad(vfov_deg) / 2.0)
    return np.array([[fx, 0.0, width / 2.0],
                     [0.0, fy, height / 2.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def scale_intrinsics(K: np.ndarray, from_size, to_size) -> np.ndarray:
    """Rescale K when you change resolution but keep the same FOV/binning."""
    sx = to_size[0] / float(from_size[0])
    sy = to_size[1] / float(from_size[1])
    K = np.asarray(K, dtype=np.float64).copy()
    K[0, 0] *= sx
    K[0, 2] *= sx
    K[1, 1] *= sy
    K[1, 2] *= sy
    return K


ZERO_D = np.zeros((5, 1), dtype=np.float64)

# --------------------------------------------------------------- presets
# Replace any of these with real calibration output when you have it.

PRESETS = {
    # Azure Kinect DK colour camera, 1080p, 16:9 mode. Nominal 90 x 59 deg.
    "arm_camera": {
        "size": (1920, 1080),
        "K": K_from_fov(1920, 1080, 90.0, 59.0),
        "D": ZERO_D.copy(),
    },
    # RealSense D435/D435i colour stream at 1280x720. Nominal 69.4 x 42.5 deg.
    "realsense_color": {
        "size": (1280, 720),
        "K": K_from_fov(1280, 720, 69.4, 42.5),
        "D": ZERO_D.copy(),
    },
    # Generic USB webcam / laptop cam, 640x480, ~60 deg horizontal.
    "secondary_color": {
        "size": (640, 480),
        "K": K_from_fov(640, 480, 60.0),
        "D": ZERO_D.copy(),
    },
    "webcam_640x480": {
        "size": (640, 480),
        "K": K_from_fov(640, 480, 60.0),
        "D": ZERO_D.copy(),
    },
}


def get_intrinsics(name: str, image_size=None):
    """Return (K, D) for a preset, rescaled if the actual frame size differs."""
    if name not in PRESETS:
        raise KeyError(f"no preset {name!r}. have: {sorted(PRESETS)}")
    p = PRESETS[name]
    K, D = p["K"].copy(), p["D"].copy()
    if image_size is not None and tuple(image_size) != tuple(p["size"]):
        K = scale_intrinsics(K, p["size"], image_size)
    return K, D


def intrinsics_from_camera_info(msg):
    """(K, D) from a sensor_msgs/CameraInfo message. Prefer this."""
    K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    D = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
    if D.size == 0:
        D = ZERO_D.copy()
    return K, D