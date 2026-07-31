"""Query the unified tag world from any node. No fusion logic here.

Attach a TagWorld to an existing node and ask it questions:

    from vision_module.world_client import TagWorld

    class MyNode(Node):
        def __init__(self):
            super().__init__("grasp_planner")
            self.world = TagWorld(self)

        def plan(self):
            pose = self.world.pose(5)          # 4x4 ref_T_tag, or None
            dist = self.world.distance(5)      # metres from reference origin
            rel  = self.world.between(5, 7)    # 4x4 tag5_T_tag7

All poses are in the world node's reference frame (the robot base if you
launched it with reference_frame:=base_link). distance() from the origin is
therefore "how far from the arm base". For distance from the gripper, look up
the tag_<id> TF frame against your end-effector frame with tf2 instead - the
world node broadcasts every tag as TF exactly so you can do that.
"""
import time
from typing import Dict, List, Optional

import numpy as np
from rclpy.node import Node
from std_msgs.msg import String

from vision_module import TagMessage 


class TagWorld:
    def __init__(self, node: Node, topic: str = "/vision/tags",
                 max_age_s: float = 0.5):
        self._node = node
        self._max_age = float(max_age_s)
        self._packet: Optional[TagMessage.FusedPacket] = None
        self._rx_walltime = 0.0
        self._sub = node.create_subscription(String, topic, self._cb, 10)

    def _cb(self, msg: String):
        try:
            self._packet = TagMessage.decode_fused(msg.data)
            self._rx_walltime = time.monotonic()
        except TagMessage.SchemaMismatch as e:
            self._node.get_logger().error(f"world packet rejected: {e}",
                                          throttle_duration_sec=5.0)

    @property
    def fresh(self) -> bool:
        """True if we heard from the world node recently."""
        return (self._packet is not None
                and time.monotonic() - self._rx_walltime < self._max_age)

    @property
    def frame(self) -> Optional[str]:
        """Reference frame all poses are expressed in."""
        return self._packet.frame_id if self._packet else None

    def tag_ids(self) -> List[int]:
        if not self.fresh:
            return []
        return sorted(t["id"] for t in self._packet.tags)

    def _entry(self, tag_id: int) -> Optional[Dict]:
        if not self.fresh:
            return None
        for t in self._packet.tags:
            if t["id"] == tag_id:
                return t
        return None

    def pose(self, tag_id: int) -> Optional[np.ndarray]:
        """4x4 ref_T_tag, or None if the tag is not currently visible."""
        t = self._entry(tag_id)
        return None if t is None else TagMessage.pose_matrix(t)

    def position(self, tag_id: int) -> Optional[np.ndarray]:
        """(3,) tag position in the reference frame."""
        t = self._entry(tag_id)
        return None if t is None else np.asarray(t["t"], np.float64)

    def distance(self, tag_id: int,
                 point: Optional[np.ndarray] = None) -> Optional[float]:
        """Metres from `point` (default: reference-frame origin) to the tag."""
        p = self.position(tag_id)
        if p is None:
            return None
        origin = np.zeros(3) if point is None else np.asarray(point, np.float64)
        return float(np.linalg.norm(p - origin))

    def between(self, tag_a: int, tag_b: int) -> Optional[np.ndarray]:
        """4x4 transform a_T_b between two currently visible tags."""
        Ta, Tb = self.pose(tag_a), self.pose(tag_b)
        if Ta is None or Tb is None:
            return None
        return np.linalg.inv(Ta) @ Tb

    def quality(self, tag_id: int) -> Optional[Dict]:
        """method / cameras / fit_rms_m / ray_angle_deg for a tag, for
        deciding whether to trust the pose before grasping at it."""
        t = self._entry(tag_id)
        if t is None:
            return None
        return {k: t[k] for k in
                ("method", "cameras", "fit_rms_m", "ray_angle_deg")}