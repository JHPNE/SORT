"""ROS 2 Node to execute arm gestures using MoveIt (ControlModule).

Listens on /arm/gesture topic (std_msgs/String) or executes a gesture specified via parameter:
  ros2 run control_module gesture_node --ros-args -p gesture:=nod -p execute:=true
"""

import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from control_module.MoveGroupClient import MoveGroupClient
from control_module.Gestures import ArmGestures, NOD_POSITION


class GestureNode(Node):
    def __init__(self):
        super().__init__("gesture_node")

        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("tool_link", "end_effector_link")
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("velocity_scaling", 0.10)
        self.declare_parameter("gesture", "")            # nod, shake, search, home
        self.declare_parameter("execute", False)         # False = dry run plan only

        p = self.get_parameter
        self.gesture_param = str(p("gesture").value).strip().lower()
        self.execute = bool(p("execute").value)

        self.move = MoveGroupClient(
            self,
            group_name=p("planning_group").value,
            tool_link=p("tool_link").value,
            reference_frame=p("reference_frame").value,
            velocity_scaling=float(p("velocity_scaling").value),
            acceleration_scaling=float(p("velocity_scaling").value)
        )

        self.gestures = ArmGestures(self.move)

        # Topic subscriber for /arm/gesture
        self.create_subscription(
            String,
            "/arm/gesture",
            self._gesture_callback,
            10
        )

    def _gesture_callback(self, msg: String):
        name = msg.data.strip().lower()
        self.get_logger().info(f"Received gesture request: '{name}'")
        thread = threading.Thread(
            target=self.gestures.execute_gesture,
            kwargs={"name": name, "plan_only": not self.execute},
            daemon=True
        )
        thread.start()

    def run(self):
        if not self.move.wait(timeout_sec=15.0):
            self.get_logger().error("move_group is not reachable.")
            return

        if self.gesture_param:
            self.get_logger().info(
                f"Executing parameter gesture '{self.gesture_param}' "
                f"({'EXECUTE' if self.execute else 'plan only'})...")
            self.gestures.execute_gesture(
                self.gesture_param, plan_only=not self.execute)


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
