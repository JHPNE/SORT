"""
AprilTagPublisher Node – erkennt AprilTags im Kamerabild und publiziert
die 3D-Pose (Position x,y,z in Metern + Orientierungs-Quaternion) auf /vision/apriltag_pose.

Der KinovaController / KinovaMover abonniert genau diesen Topic, um den Arm per IK
oder Tracking zum AprilTag zu bewegen.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

from topic_handler.TopicList import TopicList
from vision_module.vision_helper import get_dictionary, rotation_matrix_to_quaternion


class AprilTagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')

        self.bridge = CvBridge()
        self.topics = TopicList()

        # Parameter
        self.declare_parameter('target_tag_id', -1)          # Welcher Tag gesucht wird (-1 für beliebigen)
        self.declare_parameter('tag_size_m', 0.10)         # Tag-Seitenlänge in Metern (z. B. 10 cm)
        self.declare_parameter('tag_family', 'tag36h11')

        self.target_id = self.get_parameter('target_tag_id').value
        self.tag_size = float(self.get_parameter('tag_size_m').value)
        family = self.get_parameter('tag_family').value

        # AprilTag / ArUco Wörterbuch & Detektor initialisieren
        self.dictionary = get_dictionary(family)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
            self._use_new_aruco = True
        else:
            self.params = cv2.aruco.DetectorParameters_create()
            self._use_new_aruco = False

        # Publisher für /vision/apriltag_pose
        self._pub = self.create_publisher(
            PoseStamped,
            self.topics.arm.apriltag_pose.name,
            10
        )

        # Subscriber für RealSense Kamera
        self.create_subscription(
            Image,
            self.topics.camera.realsense_color.name,
            self._image_callback,
            10
        )

        self.get_logger().info(
            f'AprilTagPublisher gestartet! Lausche auf {self.topics.camera.realsense_color.name}, '
            f'publiziere Pose auf {self.topics.arm.apriltag_pose.name} (Target Tag ID: {self.target_id})'
        )

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Exception: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # Marker / AprilTags im Bild suchen
        if self._use_new_aruco:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)

        if ids is None or len(ids) == 0:
            return

        ids = ids.flatten()

        # Kameramatrix aus CameraInstrinsics Presets (RealSense D435 / nominal FOV)
        from vision_module.CameraInstrinsics import PRESETS
        preset = PRESETS.get("realsense_color")
        if preset and preset["size"] == (w, h):
            camera_matrix = preset["K"]
        else:
            fx = (w / 2.0) / np.tan(np.deg2rad(69.4) / 2.0)
            fy = (h / 2.0) / np.tan(np.deg2rad(42.5) / 2.0)
            cx, cy = w / 2.0, h / 2.0
            camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

        dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # 3D-Modellecken des AprilTags (Zentrum bei 0,0,0 im Tag-Frame)
        # Reihenfolge passend zu OpenCV detectMarkers: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        # Im Kamera-Frame: +X rechts, +Y unten -> Top-Left ist (-s, -s)
        s = self.tag_size / 2.0
        obj_points = np.array([
            [-s, -s, 0],
            [ s, -s, 0],
            [ s,  s, 0],
            [-s,  s, 0]
        ], dtype=np.float32)

        for i, tag_id in enumerate(ids):
            if self.target_id != -1 and tag_id != self.target_id:
                continue

            img_points = corners[i].reshape(-1, 2).astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

            if not success:
                continue

            # Rotationsmatrix & Quaternion berechnen
            R, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

            # PoseStamped Nachricht erstellen
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'camera_frame'

            pose_msg.pose.position.x = float(tvec[0, 0])
            pose_msg.pose.position.y = float(tvec[1, 0])
            pose_msg.pose.position.z = float(tvec[2, 0])

            pose_msg.pose.orientation.x = float(qx)
            pose_msg.pose.orientation.y = float(qy)
            pose_msg.pose.orientation.z = float(qz)
            pose_msg.pose.orientation.w = float(qw)

            # Auf /vision/apriltag_pose publizieren
            self._pub.publish(pose_msg)

            self.get_logger().info(
                f'AprilTag ID {tag_id} erkannt: x={tvec[0,0]:.2f}m, y={tvec[1,0]:.2f}m, z={tvec[2,0]:.2f}m'
            )
            break


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
