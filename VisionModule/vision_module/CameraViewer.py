import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from topic_handler.TopicList import TopicList, TopicSpec
from topic_handler.TopicHandlerSubscriber import TopicHandlerSubscriber

class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")

        self.bridge = CvBridge()
        self.topics = TopicList()

        self.frames: dict[str, "cv2.typing.MatLike"] = {}

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
                qos=10
            )
            self._subs.append(handler)

        # Timer just to pump cv2's event loop so imshow windows stay responsive
        self.create_timer(0.03, self._spin_gui)

    
    def _make_callback(self, window_name: str):
        def callback(msg: Image):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except CvBridgeError as e:
                self.get_logger().error(f'cv bridge error on  {window_name}: {e}')
                return
            self.frames[window_name] = cv_image
        return callback
    
    def _spin_gui(self):
        for window_name, frame in self.frames.items():
            cv2.imshow(window_name, frame)
        if self.frames:
            cv2.waitKey(1)
        
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()