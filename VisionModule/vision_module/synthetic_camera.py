"""
Synthetic Camera Publisher Node for VisionModule.

Generates real OpenCV camera frames containing an AprilTag (tag36h11, ID: 3)
and publishes ROS 2 Image messages to /camera/arm_camera/image_raw.

Interactive Controls:
- Mouse: Click & Drag the AprilTag in the OpenCV GUI window!
- ROS 2 Topic: Send Point coordinates to /vision/move_tag:
  ros2 topic pub /vision/move_tag geometry_msgs/msg/Point "{x: 100, y: 150, z: 0}" --once
- Keyboard (in GUI window):
  W / S or Up / Down    : Move Up / Down
  A / D or Left / Right : Move Left / Right
  + / -                 : Zoom In / Zoom Out (Closer / Further)
  R                     : Reset to Center
"""

import time
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from topic_handler.TopicList import TopicList
from vision_module.vision_helper import get_dictionary


class SyntheticCameraNode(Node):
    def __init__(self):
        super().__init__('synthetic_camera_node')
        self.bridge = CvBridge()
        self.topics = TopicList()
        self.start_time = time.time()

        # Mode: 'move' (automatic wandering), 'center' (snapped center), 'hold', 'manual'
        self.mode = 'move'
        self.x_offset = 250
        self.y_offset = 170
        self.marker_size = 140
        self.dragging = False
        self.gui_enabled = True

        image_topic = self.topics.camera.arm_camera.name
        self.publisher = self.create_publisher(Image, image_topic, 10)

        # Topic for interactive offset commands from terminal
        self.create_subscription(
            Point,
            '/vision/move_tag',
            self._move_tag_callback,
            10
        )

        self.dictionary = get_dictionary('tag36h11')
        self.tag_id = 3
        self._update_marker_img()

        # Setup OpenCV GUI Window & Mouse Handler if DISPLAY is available
        try:
            self.win_name = "Synthetic AprilTag Camera (WASD / Mouse Control)"
            cv2.namedWindow(self.win_name, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.win_name, self._mouse_callback)
        except Exception:
            self.gui_enabled = False

        self.timer = self.create_timer(0.1, self.publish_frame)

        self.get_logger().info(
            f'SyntheticCameraNode gestartet! Publiziere auf "{image_topic}".\n'
            f' Interaktive Steuerung:\n'
            f'   - MAUS: AprilTag im Vorschau-Fenster anklicken & ziehen!\n'
            f'   - TASTATOR: W/A/S/D (Verschieben), R (Zentrieren), M (Bewegen)\n'
            f'   - ROS 2 TOPIC: ros2 topic pub /vision/move_tag geometry_msgs/msg/Point "{{x: 100, y: 150, z: 0}}" --once'
        )

    def _mouse_callback(self, event, x, y, flags, param):
        """Allows dragging AprilTag with mouse in OpenCV GUI window."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.mode = 'manual'
            self.x_offset = int(max(0, min(640 - self.marker_size, x - self.marker_size // 2)))
            self.y_offset = int(max(0, min(480 - self.marker_size, y - self.marker_size // 2)))
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.x_offset = int(max(0, min(640 - self.marker_size, x - self.marker_size // 2)))
            self.y_offset = int(max(0, min(480 - self.marker_size, y - self.marker_size // 2)))
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False

    def _update_marker_img(self):
        """Generates or resizes AprilTag marker image."""
        size = max(40, min(350, self.marker_size))
        self.marker_size = size
        if hasattr(cv2.aruco, 'generateImageMarker'):
            self.marker_img = cv2.aruco.generateImageMarker(self.dictionary, self.tag_id, self.marker_size)
        else:
            self.marker_img = cv2.aruco.drawMarker(self.dictionary, self.tag_id, self.marker_size)

    def _move_tag_callback(self, msg: Point):
        """Allows switching between move (z=-1), stop & center (z=0), stop & hold (z=-2), and manual (x,y)."""
        if msg.z == -1:
            self.mode = 'move'
            self.get_logger().info('[Fake Kamera] Modus gewechselt zu: BEWEGUNG (AprilTag wandert)')
        elif msg.z == 0:
            self.mode = 'center'
            self.x_offset = 250
            self.y_offset = 170
            self.get_logger().info('[Fake Kamera] Modus gewechselt zu: GESTOPPT & ZENTRIERT (x=250, y=170)')
        elif msg.z == -2:
            self.mode = 'hold'
            self.get_logger().info(f'[Fake Kamera] Modus gewechselt zu: GESTOPPT (Position gehalten bei x={self.x_offset}, y={self.y_offset})')
        else:
            self.mode = 'manual'
            self.x_offset = int(max(0, min(640 - self.marker_size, msg.x)))
            self.y_offset = int(max(0, min(480 - self.marker_size, msg.y)))
            if msg.z > 0:
                self.marker_size = int(msg.z)
                self._update_marker_img()
            self.get_logger().info(f'[Fake Kamera] Modus gewechselt zu MANUELL: x={self.x_offset}, y={self.y_offset}')

    def publish_frame(self):
        # 640x480 Weißes Kamerabild erstellen
        canvas = np.ones((480, 640), dtype=np.uint8) * 255

        if self.mode == 'move':
            t = time.time() - self.start_time
            x = int(250 + 140 * math.sin(t * 0.6))
            y = int(170 + 80 * math.cos(t * 0.4))
            self.x_offset = x
            self.y_offset = y
        elif self.mode == 'center':
            x = 250
            y = 170
        else:  # hold or manual
            x = max(0, min(640 - self.marker_size, self.x_offset))
            y = max(0, min(480 - self.marker_size, self.y_offset))

        canvas[y:y + self.marker_size, x:x + self.marker_size] = self.marker_img
        frame_bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

        # Info-Text auf Kamerabild rendern
        cv2.putText(
            frame_bgr,
            f"AprilTag Mode:{self.mode.upper()} Pos:({x},{y}) [WASD/Mouse Control]",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 100, 0),
            2
        )

        msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        self.publisher.publish(msg)

        # OpenCV GUI window processing
        if self.gui_enabled:
            try:
                cv2.imshow(self.win_name, frame_bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('w') or key == ord('W'):
                    self.mode = 'manual'
                    self.y_offset = max(0, self.y_offset - 15)
                elif key == ord('s') or key == ord('S'):
                    self.mode = 'manual'
                    self.y_offset = min(480 - self.marker_size, self.y_offset + 15)
                elif key == ord('a') or key == ord('A'):
                    self.mode = 'manual'
                    self.x_offset = max(0, self.x_offset - 15)
                elif key == ord('d') or key == ord('D'):
                    self.mode = 'manual'
                    self.x_offset = min(640 - self.marker_size, self.x_offset + 15)
                elif key == ord('r') or key == ord('R'):
                    self.mode = 'center'
                    self.x_offset = 250
                    self.y_offset = 170
                elif key == ord('m') or key == ord('M'):
                    self.mode = 'move'
                elif key == 27 or key == ord('q') or key == ord('Q'):
                    # ESC or Q key pressed -> quit node cleanly
                    raise KeyboardInterrupt
            except KeyboardInterrupt:
                raise
            except Exception:
                self.gui_enabled = False


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
