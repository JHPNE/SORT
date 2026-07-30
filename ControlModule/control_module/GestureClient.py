"""ROS 2 Publisher Client for sending gesture requests to GestureNode or Kinova arm.

=============================================================================
USAGE EXAMPLES IN YOUR ROS 2 NODE:
=============================================================================
  from control_module.GestureClient import GestureClient

  class MyNode(Node):
      def __init__(self):
          super().__init__('my_node')
          self.gesture_client = GestureClient(self)

      def do_something(self):
          self.gesture_client.nod()     # Publishes "nod" to /arm/gesture

Status Check
# -----------------------------------------------------------------
# OPTION A: Non-blocking check (Status polling / State Machine)
# -----------------------------------------------------------------
if self.gesture_client.get_is_currently_moving():
    self.get_logger().info("Arm is currently moving...")

# -----------------------------------------------------------------
# OPTION B: Blocking wait (Wait until gesture is completed)
# -----------------------------------------------------------------
self.gesture_client.nod()
finished = self.gesture_client.wait_until_finished(timeout_sec=15.0)
if finished:
    self.get_logger().info("Gesture finished! Proceeding to next step.")
=============================================================================
"""

import json
from typing import Optional
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from topic_handler.TopicList import TopicList
from control_module.Gestures import DEFAULT_TAG_ID


class GestureClient:
    """Client class to publish gesture commands to /arm/gesture topic."""

    def __init__(self, node: Node, topic_name: Optional[str] = None, status_topic: Optional[str] = None):
        self.node = node
        topics = TopicList()
        self.topic_name = topic_name or topics.arm.gesture.name
        self.status_topic = status_topic or topics.arm.is_moving.name
        self.publisher = node.create_publisher(String, self.topic_name, 10)
        self.is_currently_moving: bool = False

        # Status subscriber: Receives automatic True/False updates from GestureNode
        self.status_sub = node.create_subscription(
            Bool,
            self.status_topic,
            self._status_callback,
            10
        )

    def _status_callback(self, msg: Bool):
        self.is_currently_moving = bool(msg.data)

    def get_is_currently_moving(self) -> bool:
        """Getter for is_currently_moving status."""
        return self.is_currently_moving

    def set_is_currently_moving(self, value: bool):
        """Setter for is_currently_moving status."""
        self.is_currently_moving = bool(value)

    def wait_until_finished(self, timeout_sec: float = 15.0) -> bool:
        """Block until the current arm gesture movement is completed."""
        start_time = time.time()
        while self.is_currently_moving and (time.time() - start_time) < timeout_sec:
            time.sleep(0.1)
        return not self.is_currently_moving

    def trigger(self, gesture_name: str, tag_id: Optional[int] = None):
        """Publish raw gesture string command with optional tag_id."""
        self.is_currently_moving = True
        name_clean = gesture_name.strip().lower()

        payload = {"name": name_clean}
        if tag_id is not None:
            payload["tag_id"] = int(tag_id)

        msg = String()
        if len(payload) > 1:
            msg.data = json.dumps(payload)
        else:
            msg.data = name_clean

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

    def pin_point_tag(self, tag_id: int = DEFAULT_TAG_ID):
        """Send 'pin_point_tag' gesture command to search and approach specific tag_id."""
        self.trigger("pin_point_tag", tag_id=tag_id)

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
