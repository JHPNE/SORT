"""First MoveIt test. No vision, no gripper, no tags.

Reads where the tool is right now, asks move_group to plan to a point a few
centimetres away, and reports. Defaults to plan-only, so running it cannot
move the arm until you explicitly say so.

Relative motion on purpose: it needs no knowledge of your workspace
coordinates and cannot ask for something unreachable, so a failure here is a
real plumbing failure rather than a bad target.

    # 1. plan only - safe, arm does not move
    ros2 run arm_motion motion_test

    # 2. same but actually move, 5 cm up and back
    ros2 run arm_motion motion_test --ros-args -p execute:=true

    # 3. different axis / distance
    ros2 run arm_motion motion_test --ros-args \\
        -p execute:=true -p axis:=x -p distance_m:=0.08

Run it from the VM. Everything it touches is a network interface.
"""
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

from control_module.MoveGroupClient import MoveGroupClient


class MotionTestNode(Node):
    def __init__(self):
        super().__init__("motion_test")

        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("tool_link", "tool_frame")
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("axis", "z")           # x, y or z
        self.declare_parameter("distance_m", 0.02)
        self.declare_parameter("execute", False)      # False = plan only
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("velocity_scaling", 0.07)

        p = self.get_parameter
        self.tool_link = p("tool_link").value
        self.ref_frame = p("reference_frame").value
        self.axis = str(p("axis").value).lower()
        self.distance = float(p("distance_m").value)
        self.execute = bool(p("execute").value)
        self.return_home = bool(p("return_to_start").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move = MoveGroupClient(
            self,
            group_name=p("planning_group").value,
            tool_link=self.tool_link,
            reference_frame=self.ref_frame,
            velocity_scaling=float(p("velocity_scaling").value),
            acceleration_scaling=float(p("velocity_scaling").value))

    # ---------------------------------------------------------------- utils

    def current_pose(self, timeout_s: float = 10.0) -> Optional[PoseStamped]:
        """Where tool_link is now, straight from TF. No matrix maths - we
        reuse the quaternion verbatim so the test never changes orientation."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.ref_frame, self.tool_link, rclpy.time.Time())
            except Exception:
                time.sleep(0.2)
                continue

            pose = PoseStamped()
            pose.header.frame_id = self.ref_frame
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            return pose

        self.get_logger().error(
            f"no TF {self.ref_frame} <- {self.tool_link} after {timeout_s}s.\n"
            f"  If tf2 sees nothing at all: ROS_DOMAIN_ID or "
            f"RMW_IMPLEMENTATION differs from the robot PC.\n"
            f"  If other frames exist but not this one: wrong tool_link name. "
            f"Check with: ros2 run tf2_tools view_frames")
        return None

    def offset(self, pose: PoseStamped, metres: float) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = pose.header.frame_id
        out.pose.orientation = pose.pose.orientation
        out.pose.position.x = pose.pose.position.x
        out.pose.position.y = pose.pose.position.y
        out.pose.position.z = pose.pose.position.z
        if self.axis == "x":
            out.pose.position.x += metres
        elif self.axis == "y":
            out.pose.position.y += metres
        else:
            out.pose.position.z += metres
        return out

    # ------------------------------------------------------------- sequence

    def run(self):
        self.get_logger().info(
            "=== MoveIt connectivity test ===\n"
            f"  mode        : {'EXECUTE (arm will move)' if self.execute else 'plan only'}\n"
            f"  motion      : {self.distance:+.3f} m along {self.axis}")

        if not self.move.wait(timeout_sec=15.0):
            self.get_logger().error(
                "move_group is not reachable. From this machine, check:\n"
                "  ros2 node list | grep move_group\n"
                "  ros2 action list | grep move_action\n"
                "If the node shows but the action does not, move_group is "
                "still starting up or crashed after launch.")
            return

        start = self.current_pose()
        if start is None:
            return
        s = start.pose.position
        self.get_logger().info(
            f"tool at ({s.x:+.3f}, {s.y:+.3f}, {s.z:+.3f}) in {self.ref_frame}")

        target = self.offset(start, self.distance)

        # Always dry-run first, whatever the execute setting says.
        if not self.move.go(target, plan_only=True, label="dry run"):
            self.get_logger().error(
                "planning failed even though the pose is a few centimetres "
                "from the current one. Look at the error name above: "
                "INVALID_GROUP_NAME or INVALID_LINK_NAME means a name is "
                "wrong, NO_IK_SOLUTION means the solver could not reach it, "
                "START_STATE_IN_COLLISION means move_group thinks the arm is "
                "already touching something.")
            return

        if not self.execute:
            self.get_logger().info(
                "plan succeeded. Rerun with -p execute:=true to move.")
            return

        if not self.move.go(target, plan_only=False, label="out"):
            return
        if self.return_home:
            time.sleep(0.5)
            self.move.go(start, plan_only=False, label="back")

        self.get_logger().info("=== test complete ===")


def main(args=None):
    rclpy.init(args=args)
    node = MotionTestNode()
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