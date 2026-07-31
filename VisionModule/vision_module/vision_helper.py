"""
Vision_helper

contains Supported Apriltag Families aliases for different input handling
and a conversion method for Rotation Matrix (numpy) to ROS Standard (quarternion)

"""
import numpy as np
import cv2

HAS_ARUCO = hasattr(cv2.aruco, "ArucoDetector")

SUPPORTED_FAMILIES = {
    "tag36h11": "DICT_APRILTAG_36h11",
    "tag25h9":  "DICT_APRILTAG_25h9",
    "tag16h5":  "DICT_APRILTAG_16h5",
    "tagCircle21h7": "DICT_APRILTAG_36h10",  # closest OpenCV has; rarely used
}

_FAMILY_ALIASES = {
    "36h11": "tag36h11",
    "tag36h11": "tag36h11",
    "dict_apriltag_36h11": "tag36h11",
    "apriltag_36h11": "tag36h11",
    "25h9": "tag25h9",
    "tag25h9": "tag25h9",
    "dict_apriltag_25h9": "tag25h9",
    "16h5": "tag16h5",
    "tag16h5": "tag16h5",
    "dict_apriltag_16h5": "tag16h5",
}


def normalize_family(name: str) -> str:
    """Accept 'tag36h11', '36h11' or 'DICT_APRILTAG_36h11' and return the
    canonical key into SUPPORTED_FAMILIES."""
    key = _FAMILY_ALIASES.get(str(name).strip().lower())
    if key is None:
        raise ValueError(
            f"unknown tag family {name!r}. supported: {sorted(SUPPORTED_FAMILIES)}"
        )
    return key


def get_dictionary(family: str):
    """cv2.aruco dictionary object for a (possibly aliased) family name."""
    canonical = normalize_family(family)
    const_name = SUPPORTED_FAMILIES[canonical]
    if not hasattr(cv2.aruco, const_name):
        raise RuntimeError(
            f"this OpenCV build has no cv2.aruco.{const_name} "
            f"(opencv {cv2.__version__}); install opencv-contrib-python"
        )
    dict_id = getattr(cv2.aruco, const_name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def rotation_matrix_to_quaternion(R: np.ndarray):
    """(x, y, z, w), ROS convention. Shepperd's method, numerically stable."""
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))