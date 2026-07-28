import os
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from topic_handler.TopicList import TopicList
from topic_handler.TopicHandlerSubscriber import TopicHandlerSubscriber


class TestCamera(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()
        self.topics = TopicList()

        self.output_dir = os.path.expanduser('~/camera_snapshots')
        os.makedirs(self.output_dir, exist_ok=True)

        self.saved = set()  # tracks which cameras already got their one-time snapshot

        self._subs = []
        cam_topics = {
            'k4a_rgb': self.topics.camera.k4a_rgb,
            'realsense_color': self.topics.camera.realsense_color,
            'secondary_color': self.topics.camera.secondary_color,
        }

        for name, spec in cam_topics.items():
            handler = TopicHandlerSubscriber(
                node=self,
                topic_spec=spec,
                callback=self._make_callback(name),
                qos=10,
            )
            self._subs.append(handler)

    def _make_callback(self, cam_name: str):
        def callback(msg: Image):
            if cam_name in self.saved:
                return  

            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except CvBridgeError as e:
                self.get_logger().error(f'cv_bridge error on {cam_name}: {e}')
                return

            filepath = os.path.join(self.output_dir, f'{cam_name}.png')
            cv2.imwrite(filepath, cv_image)
            self.saved.add(cam_name)
            self.get_logger().info(f'Saved snapshot: {filepath}')

            if len(self.saved) == len(self._subs):
                self.get_logger().info('All camera snapshots captured.')

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = TestCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()