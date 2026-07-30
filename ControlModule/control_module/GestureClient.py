"""ROS 2 Publisher Client for sending gesture requests to GestureNode or Kinova arm.

=============================================================================
USAGE EXAMPLE IN YOUR ROS 2 NODE:
=============================================================================
  from control_module.GestureClient import GestureClient

  class MyNode(Node):
      def __init__(self):
          super().__init__('my_node')
          self.gesture_client = GestureClient(self)

      def do_something(self):
          self.gesture_client.nod()     # Publishes "nod" to /arm/gesture
          self.gesture_client.shake()   # Publishes "shake" to /arm/gesture
          self.gesture_client.home()    # Publishes "home" to /arm/gesture
=============================================================================
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GestureClient:
    """Client class to publish gesture commands to /arm/gesture topic."""

    def __init__(self, node: Node, topic_name: str = "/arm/gesture"):
        self.node = node
        self.topic_name = topic_name
        self.publisher = node.create_publisher(String, topic_name, 10)

    def trigger(self, gesture_name: str):
        """Publish raw gesture string command."""
        msg = String()
        msg.data = gesture_name.strip().lower()
        self.node.get_logger().info(f"Publishing gesture command: '{msg.data}' to {self.topic_name}")
        self.publisher.publish(msg)

    def nod(self):
        """Send 'nod' gesture command."""
        self.trigger("nod")

    def shake(self):
        """Send 'shake' gesture command."""
        self.trigger("shake")

    def tilt(self):
        """Send 'tilt' gesture command."""
        self.trigger("tilt")


    def search(self):
        """Send 'search' gesture command."""
        self.trigger("search")

    def home(self):
        """Send 'home' gesture command."""
        self.trigger("home")


def main(args=None):
    """CLI client to publish gesture command via parameters."""
    rclpy.init(args=args)
    node = Node("gesture_client_cli")
    node.declare_parameter("gesture", "nod")
    gesture_name = str(node.get_parameter("gesture").value).strip().lower()

    client = GestureClient(node)

    # Allow publisher connection to settle before sending single message
    time.sleep(0.5)
    client.trigger(gesture_name)
    time.sleep(0.2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
