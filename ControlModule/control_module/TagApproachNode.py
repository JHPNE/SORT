import threading
import time
from typing import List, Optional
 
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
 
from control_module.MoveGroupClient import MoveGroupClient
from vision_module.WorldClient import TagWorld

class TagApproachNode(Node):
    def __init__(self):
        super().__init__("tag_approach")
 
        self.declare_parameter("tag_id", 3)
        self.declare_parameter("step_m", 0.05)
        self.declare_parameter("min_standoff_m", 0.25)
        self.declare_parameter("min_z_m", 0.05)       # never plan below this
        self.declare_parameter("execute", False)
        self.declare_parameter("samples", 10)
 
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("tool_link", "end_effector_link")
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("velocity_scaling", 0.07)
 
        p = self.get_parameter
        self.tag_id = int(p("tag_id").value)
        self.step = float(p("step_m").value)
        self.standoff = float(p("min_standoff_m").value)
        self.min_z = float(p("min_z_m").value)
        self.execute = bool(p("execute").value)
        self.samples = int(p("samples").value)
        self.tool_link = p("tool_link").value
        self.ref_frame = p("reference_frame").value
 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        self.world = TagWorld(self, max_age_s=0.5)
 
        self.move = MoveGroupClient(
            self,
            group_name=p("planning_group").value,
            tool_link=self.tool_link,
            reference_frame=self.ref_frame,
            velocity_scaling=float(p("velocity_scaling").value),
            acceleration_scaling=float(p("velocity_scaling").value))

    def tool_pose(self, timeout_s: float = 10.0) -> Optional[PoseStamped]:
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
            f"no TF {self.ref_frame} <- {self.tool_link}")
        return None

    def latch_tag(self, timeout_s: float = 5.0) -> Optional[np.ndarray]:
        """Median tag position over several packets.
 
        Median rather than mean: a single bad detection at a shallow viewing
        angle can be tens of centimetres out, and a mean carries that error
        into the target while a median discards it.
        """
        positions: List[np.ndarray] = []
        deadline = time.monotonic() + timeout_s
        while len(positions) < self.samples and time.monotonic() < deadline:
            pos = self.world.position(self.tag_id)
            if pos is not None:
                positions.append(np.asarray(pos, dtype=np.float64))
            time.sleep(0.05)
 
        if len(positions) < max(3, self.samples // 2):
            self.get_logger().error(
                f"tag {self.tag_id}: only {len(positions)} detections in "
                f"{timeout_s:.0f}s - too few to aim at")
            return None
 
        q = self.world.quality(self.tag_id) or {}
        self.get_logger().info(
            f"tag {self.tag_id} latched from {len(positions)} samples "
            f"[method={q.get('method')} cameras={q.get('cameras')} "
            f"fit_rms={q.get('fit_rms_m')}]")
        return np.median(np.stack(positions), axis=0)

    def run(self):
        if not self.move.wait(timeout_sec=15.0):
            return
 
        # --- vision is alive and speaking the right language ---------------
        deadline = time.monotonic() + 10.0
        while not self.world.fresh and time.monotonic() < deadline:
            time.sleep(0.2)
        if not self.world.fresh:
            self.get_logger().error(
                "no fresh data on /vision/tags. Is world_space running? "
                "Check: ros2 topic hz /vision/tags")
            return
 
        if self.world.frame != self.ref_frame:
            self.get_logger().error(
                f"vision publishes poses in '{self.world.frame}' but this "
                f"node plans in '{self.ref_frame}'. Relaunch world_space "
                f"with reference_frame:={self.ref_frame} - otherwise the "
                f"numbers look plausible and are in the wrong frame, which "
                f"is the worst kind of wrong.")
            return
 
        visible = self.world.tag_ids()
        if self.tag_id not in visible:
            self.get_logger().error(
                f"tag {self.tag_id} not visible. Currently seen: {visible}")
            return
 
        tag_pos = self.latch_tag()
        if tag_pos is None:
            return
        start = self.tool_pose()
        if start is None:
            return
 
        tool_pos = np.array([start.pose.position.x,
                             start.pose.position.y,
                             start.pose.position.z])
        delta = tag_pos - tool_pos
        distance = float(np.linalg.norm(delta))
 
        self.get_logger().info(
            f"tool at ({tool_pos[0]:+.3f}, {tool_pos[1]:+.3f}, "
            f"{tool_pos[2]:+.3f})\n"
            f"  tag  at ({tag_pos[0]:+.3f}, {tag_pos[1]:+.3f}, "
            f"{tag_pos[2]:+.3f})\n"
            f"  distance {distance:.3f} m")
 
        if distance < 1e-6:
            self.get_logger().error("tool and tag coincide - suspect frames")
            return
        if distance <= self.standoff:
            self.get_logger().info(
                f"already within min_standoff_m ({self.standoff:.3f} m). "
                f"Nothing to do - lower it if you want to go closer.")
            return
 
        # Never overshoot past the standoff, whatever step_m says.
        step = min(self.step, distance - self.standoff)
        target_pos = tool_pos + (delta / distance) * step
 
        if target_pos[2] < self.min_z:
            self.get_logger().error(
                f"target z {target_pos[2]:+.3f} is below min_z_m "
                f"({self.min_z:.3f}) - refusing. Either the tag pose is "
                f"wrong or the tag really is that low.")
            return
 
        target = PoseStamped()
        target.header.frame_id = self.ref_frame
        target.pose.orientation = start.pose.orientation   # unchanged
        target.pose.position.x = float(target_pos[0])
        target.pose.position.y = float(target_pos[1])
        target.pose.position.z = float(target_pos[2])
 
        self.get_logger().info(
            f"stepping {step:.3f} m toward tag {self.tag_id}")
 
        # --- plan, then maybe move ----------------------------------------
        if not self.move.go(target, plan_only=True, label="dry run"):
            self.get_logger().error(
                "planning failed. On a 6-DoF arm holding orientation fixed "
                "is a real constraint - try a smaller step_m, or loosen "
                "orientation_tolerance in MoveGroupClient.")
            return
 
        if not self.execute:
            self.get_logger().info(
                "plan ok. Rerun with -p execute:=true to move.")
            return
 
        if not self.move.go(target, plan_only=False, label="approach"):
            return
 
        time.sleep(1.0)
        after = self.tool_pose()
        tag_now = self.latch_tag(timeout_s=3.0)
        if after is None or tag_now is None:
            self.get_logger().info("moved, but could not re-measure")
            return
 
        new_pos = np.array([after.pose.position.x,
                            after.pose.position.y,
                            after.pose.position.z])
        new_distance = float(np.linalg.norm(tag_now - new_pos))
        closed = distance - new_distance
 
        self.get_logger().info(
            f"distance {distance:.3f} -> {new_distance:.3f} m "
            f"(closed {closed:+.3f}, commanded {step:.3f})")
        if closed < step * 0.5:
            self.get_logger().warn(
                "closed much less than commanded. Either the arm did not "
                "reach the goal, or the tag pose shifted when the "
                "arm-mounted camera moved - which points at hand-eye "
                "calibration.")
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TagApproachNode()
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
