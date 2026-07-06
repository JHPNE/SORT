import rclpy
from rclpy.node import Node

from TopicList import TopicList
from TopicHandlerPublisher import TopicHandlerPublisher
from TopicHandlerSubscriber import TopicHandlerSubscriber 


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        topics = TopicList()

        # Subscriber: RGB image in
        self.rgb_sub = TopicHandlerSubscriber(
            self,
            topics.camera.rgb_image_raw,
            self.rgb_callback,
        )

        

    def rgb_callback(self, msg):
        #bridge comes here
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()