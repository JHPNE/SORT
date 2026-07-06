import importlib
from rclpy.node import Node
from TopicList import TopicSpec 


def resolve_msg_type(msg_type: str):
    """Converts 'sensor_msgs/msg/Image' -> sensor_msgs.msg.Image class"""
    pkg, _, cls_name = msg_type.split('/')
    module = importlib.import_module(f'{pkg}.msg')
    return getattr(module, cls_name)


class TopicHandlerPublisher:
    def __init__(self, node: Node, topic_spec: TopicSpec, qos: int = 10):
        self.node = node
        self.spec = topic_spec
        msg_cls = resolve_msg_type(topic_spec.msg_type)
        self.publisher = node.create_publisher(msg_cls, topic_spec.name, qos)
        node.get_logger().info(
            f'Publishing on {topic_spec.name} ({topic_spec.msg_type})')

    def publish(self, msg):
        self.publisher.publish(msg)