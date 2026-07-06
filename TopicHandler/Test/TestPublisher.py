import rclpy
from rclpy.node import Node
from topic_list import TopicList
from topic_handler_publisher import TopicHandlerPublisher

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        topics = TopicList()
        self.rgb_pub = TopicHandlerPublisher(self, topics.camera.rgb_image_raw)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()