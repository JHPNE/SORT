"""Read the unified tag world, print everything in it, and test zones.

Three layers of output, each answering one question:
  1. World table   - which tags are visible, where, how trustworthy?
  2. Sort state    - which cube is flagged in which zone right now?
  3. Local coords  - where is each cube IN EACH ZONE'S OWN FRAME?

Layer 3 is the calibration tool: put a cube tag on a zone's top-left
corner and the printed local (x, y) IS that zone's (x_min, y_max) -
copy it straight into Zones.py. Repeat at the bottom-right corner for
(x_max, y_min). z should read roughly the cube height.

Run:
    ros2 run vision_module tag_reader
    ros2 run vision_module tag_reader --ros-args -p show_local:=false
"""
import numpy as np
import rclpy
from rclpy.node import Node

from vision_module.WorldClient import TagWorld
from vision_module.Zone import ZoneMap, ZONES
from vision_module import TagRegistry


class TagReaderNode(Node):
    def __init__(self):
        super().__init__("tag_reader")

        self.declare_parameter("print_rate_hz", 2.0)
        # Local per-zone coordinates: verbose, but it is how you verify and
        # calibrate the zone rectangles. Turn off once the bounds are right.
        self.declare_parameter("show_local", True)

        rate = float(self.get_parameter("print_rate_hz").value)
        self.show_local = bool(self.get_parameter("show_local").value)

        self.world = TagWorld(self, max_age_s=0.5)
        self.zones = ZoneMap(self.world, ZONES)
        self.cube_ids = TagRegistry.ids_in("trash")

        self.create_timer(1.0 / rate, self._report)

    # ------------------------------------------------------------- report

    def _report(self):
        if not self.world.fresh:
            self.get_logger().warn(
                "no fresh world state on /vision/tags - is world_space "
                "running and seeing tags?", throttle_duration_sec=5.0)
            return

        ids = self.world.tag_ids()
        if not ids:
            self.get_logger().info("world: no tags visible",
                                   throttle_duration_sec=2.0)
            return

        lines = [f"world '{self.world.frame}': {len(ids)} tag(s)"]
        lines += self._world_table(ids)
        lines += self._sort_state()
        if self.show_local:
            lines += self._local_coords()
        self.get_logger().info("\n".join(lines))

    def _world_table(self, ids):
        out = []
        for tid in ids:
            p = self.world.position(tid)
            q = self.world.quality(tid)
            reg = TagRegistry.info(tid)
            name = reg.name if reg else "UNREGISTERED"
            out.append(
                f"  tag {tid:>3} {name:<14} "
                f"xyz=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m  "
                f"dist={self.world.distance(tid):.3f} m  "
                f"[{q['method']}, {len(q['cameras'])}cam]")
        return out

    def _sort_state(self):
        state = self.zones.sort_state(self.cube_ids)
        out = ["  --- zones ---"]
        visible = {z.zone_id for z in self.zones.visible_zones()}
        for zid, zone in self.zones.zones.items():
            marker = "" if zid in visible else "  (fewer than 3 corners visible)"
            cubes = state.get(zone.name, [])
            cube_str = (", ".join(
                f"{cid}:{TagRegistry.info(cid).name}" for cid in cubes)
                if cubes else "-")
            out.append(f"  {zone.name:<14} [zone {zid}]: {cube_str}{marker}")
        if state.get("unassigned"):
            out.append("  unassigned    : "
                       + ", ".join(str(c) for c in state["unassigned"]))
        return out

    def _local_coords(self):
        """Zone geometry and each cube's position relative to it."""
        return ["  --- zone geometry ---"] + self.zones.describe(self.cube_ids)


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