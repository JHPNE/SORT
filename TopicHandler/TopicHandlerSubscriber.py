import importlib
from rclpy.node import Node
from typing import Callable
from topic_list import TopicSpec


def resolve_msg_type(msg_type: str):
    pkg, _, cls_name = msg_type.split('/')
    module = importlib.import_module(f'{pkg}.msg')
    return getattr(module, cls_name)


class TopicHandlerSubscriber:
    def __init__(self, node: Node, topic_spec: TopicSpec, callback: Callable, qos: int = 10):
        self.node = node
        self.spec = topic_spec
        msg_cls = resolve_msg_type(topic_spec.msg_type)
        self.subscription = node.create_subscription(
            msg_cls, topic_spec.name, callback, qos)
        node.get_logger().info(
            f'Subscribed to {topic_spec.name} ({topic_spec.msg_type})')