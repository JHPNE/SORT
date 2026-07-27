import rclpy
from rclpy.node import Node
from topic_handler.TopicList import TopicList
from topic_handler.TopicHandlerSubscriber import TopicHandlerSubscriber


class KinovaTestServer(Node):
    """
    Test server node that subscribes to the arm's joint_trajectory topic
    (or a remapped topic) and logs whatever trajectory commands are received.
    """
    def __init__(self):
        super().__init__('kinova_test_server')
        self.topics = TopicList()

        self._subscriber = TopicHandlerSubscriber(
            node=self,
            # topic_spec=self.topics.arm.joint_trajectory,
            topic_spec=self.topics.arm.publish_test,
            callback=self.on_trajectory_received,
            qos=10
        )
        self.get_logger().info(
            f'Test server node initialized. Listening on topic: {self.topics.arm.publish_test.name}'
        )

    def on_trajectory_received(self, msg):
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'RECEIVED TRAJECTORY COMMAND ON {self.topics.arm.publish_test.name}:')
        self.get_logger().info(f'  Joint Names : {msg.joint_names}')
        self.get_logger().info(f'  Points Count: {len(msg.points)}')
        for idx, point in enumerate(msg.points):
            self.get_logger().info(
                f'  Point #{idx + 1}: Positions={point.positions}, '
                f'TimeFromStart={point.time_from_start.sec}s'
            )
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = KinovaTestServer()
    node.get_logger().info('Kinova Test Server is spinning... Press Ctrl+C to stop.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test server stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
