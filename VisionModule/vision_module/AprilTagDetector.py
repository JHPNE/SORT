import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from topic_handler.TopicList import TopicList, TopicSpec
from topic_handler.TopicHandlerSubscriber import TopicHandlerSubscriber


class AprilTagDetector:
    def __init__(self, tag_family: str = 'DICT_APRILTAG_25h9'):
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, tag_family)
        )
        self.params = cv2.aruco.DetectorParameters_create() \
            if hasattr(cv2.aruco, 'DetectorParameters_create') \
            else cv2.aruco.DetectorParameters()


    def detect(self, image: np.ndarray):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.params
        )
        return corners, ids, rejected

    def estimate_poses(self, corners):
        if self.tag_size_m is None or self.camera_matrix is None:
            raise ValueError("tag_size_m and camera_matrix required for pose estimation")

        half = self.tag_size_m / 2
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        poses = []
        for c in corners:
            ok, rvec, tvec = cv2.solvePnP(
                obj_points, c[0], self.camera_matrix, self.dist_coeffs
            )
            poses.append((rvec, tvec) if ok else (None, None))
        return poses

    def draw(self, image: np.ndarray, corners, ids):
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
        return image