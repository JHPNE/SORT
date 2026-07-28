"""Read the unified tag world and print everything that is in it.

Two purposes:
  1. Eyeball check: run it and see every tag with position, distance and
     quality, in the world reference frame (base_link if world_space was
     launched that way).
  2. Reference implementation for your motion node: copy the pattern in
     `example_target_query()` - it shows how to pick a tag, check the pose
     is trustworthy, and get position/distance. Your motion node should
     import TagWorld directly rather than subscribing to this node.

Run:
    ros2 run vision_module tag_reader
    ros2 run vision_module tag_reader --ros-args -p watch_tag:=5
"""
import numpy as np
import rclpy
from rclpy.node import Node

from vision_module.WorldClient import TagWorld


class TagReaderNode(Node):
    def __init__(self):
        super().__init__("tag_reader")

        # Set to a tag id to additionally print a motion-style report for it.
        self.declare_parameter("watch_tag", -1)
        self.declare_parameter("print_rate_hz", 2.0)

        self.watch_tag = int(self.get_parameter("watch_tag").value)
        rate = float(self.get_parameter("print_rate_hz").value)

        self.world = TagWorld(self, max_age_s=0.5)
        self.create_timer(1.0 / rate, self._print_world)

    # ------------------------------------------------------------- display

    def _print_world(self):
        if not self.world.fresh:
            self.get_logger().warn(
                "no fresh world state on /vision/tags - is world_space "
                "running and seeing tags?", throttle_duration_sec=5.0)
            return

        ids = self.world.tag_ids()
        if not ids:
            self.get_logger().info(f"world '{self.world.frame}': no tags",
                                   throttle_duration_sec=2.0)
            return

        lines = [f"world '{self.world.frame}': {len(ids)} tag(s)"]
        for tid in ids:
            p = self.world.position(tid)
            d = self.world.distance(tid)
            q = self.world.quality(tid)
            lines.append(
                f"  tag {tid:>3}  "
                f"xyz=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m  "
                f"dist={d:.3f} m  "
                f"[{q['method']}, {len(q['cameras'])}cam"
                + (f", fit={q['fit_rms_m'] * 1000:.1f}mm"
                   if q['fit_rms_m'] is not None else "")
                + "]")
        self.get_logger().info("\n".join(lines))

        if self.watch_tag >= 0:
            self.example_target_query(self.watch_tag)

    # --------------------------------------- pattern for your motion node

    def example_target_query(self, tag_id: int):
        """This is the exact sequence a motion node should run each cycle.

        In your motion node:
            self.world = TagWorld(self)          # once, in __init__
            ...then per control cycle:
        """
        # 1. Is the world state itself fresh? Never move on stale data.
        if not self.world.fresh:
            self.get_logger().warn("world stale - hold position")
            return None

        # 2. Is the target currently visible?
        pose = self.world.pose(tag_id)           # 4x4 ref_T_tag
        if pose is None:
            self.get_logger().warn(f"tag {tag_id} not visible - hold")
            return None

        # 3. Is the pose good enough to act on? A single-camera PnP pose has
        #    much worse depth than a triangulated one; gate on it if your
        #    motion needs precision.
        q = self.world.quality(tag_id)
        precise = (q["method"] == "triangulated"
                   and (q["fit_rms_m"] is None or q["fit_rms_m"] < 0.005))

        position = pose[:3, 3]                   # metres, reference frame
        distance = float(np.linalg.norm(position))
        z_axis = pose[:3, 2]                     # tag normal, for approach dir

        self.get_logger().info(
            f"TARGET tag {tag_id}: dist={distance:.3f} m "
            f"pos=({position[0]:+.3f}, {position[1]:+.3f}, "
            f"{position[2]:+.3f}) precise={precise} "
            f"normal=({z_axis[0]:+.2f}, {z_axis[1]:+.2f}, {z_axis[2]:+.2f})")

        # Your motion node would now hand `pose` / `position` to its planner.
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = TagReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()