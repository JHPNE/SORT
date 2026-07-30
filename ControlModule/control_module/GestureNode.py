"""ROS 2 Node to execute arm gestures using MoveIt (ControlModule).

=============================================================================
USAGE EXAMPLES:
=============================================================================

1. DIRECT CLI EXECUTION VIA PARAMETER (SINGLE GESTURE):
   # A) Plan-Only / Dry-Run (Safe - arm does NOT move, MoveIt planning only):
   ros2 run control_module gesture_node --ros-args -p gesture:=nod -p execute:=false

   # B) Execute actual physical motion on arm (nod, shake, tilt, search, home):
   ros2 run control_module gesture_node --ros-args -p gesture:=home
   ros2 run control_module gesture_node --ros-args -p gesture:=nod
   ros2 run control_module gesture_node --ros-args -p gesture:=tilt
   ros2 run control_module gesture_node --ros-args -p gesture:=shake -p velocity_scaling:=0.50

2. CONTROL DISPATCHER NODE VIA ROS 2 TOPIC:
   # Terminal 1: Launch GestureNode in execution mode (default):
   ros2 run control_module gesture_node

   # Terminal 2: Publish commands over topic:
   ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'nod'"
   ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'tilt'"
   ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'shake'"
   ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'search'"
   ros2 topic pub --once /arm/gesture std_msgs/msg/String "data: 'home'"
=============================================================================
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
        self.declare_parameter("velocity_scaling", 0.40)
        self.declare_parameter("gesture", "")            # nod, shake, tilt, search, home

        self.declare_parameter("execute", True)          # True = execute by default (False = dry run plan only)

        p = self.get_parameter
        self.gesture_param = str(p("gesture").value).strip().lower()
        self.execute = bool(p("execute").value)
        self.vel_scale = float(p("velocity_scaling").value)

        self.move = MoveGroupClient(
            self,
            group_name=p("planning_group").value,
            tool_link=p("tool_link").value,
            reference_frame=p("reference_frame").value,
            velocity_scaling=self.vel_scale,
            acceleration_scaling=self.vel_scale
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
            kwargs={"name": name, "plan_only": not self.execute, "velocity_scaling": self.vel_scale},
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
                f"({'EXECUTE' if self.execute else 'plan only'}) at speed {self.vel_scale:.2f}...")
            self.gestures.execute_gesture(
                self.gesture_param, plan_only=not self.execute, velocity_scaling=self.vel_scale)
        else:
            self.get_logger().info(
                f"GestureNode active and listening on /arm/gesture topic "
                f"({'EXECUTE' if self.execute else 'plan only'} mode, speed {self.vel_scale:.2f}). "
                f"Press Ctrl+C to exit.")
            while rclpy.ok():
                time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        time.sleep(2.0)
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
