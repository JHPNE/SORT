"""Arm Gestures for Kinova Gen3 using MoveIt MoveGroupClient (Method 2).

Defines joint-space gestures (nod, shake, search, home).
Easily expandable: add a new method to ArmGestures and register its name in self._gestures.

=============================================================================
PYTHON USAGE IN YOUR OWN CODE:
=============================================================================
  from control_module.MoveGroupClient import MoveGroupClient
  from control_module.Gestures import ArmGestures

  move = MoveGroupClient(node, group_name="manipulator")
  gestures = ArmGestures(move)

  # Execute nod, shake, or home:
  gestures.nod(plan_only=False)
  gestures.shake(plan_only=False, velocity_scaling=0.50)
  gestures.home(plan_only=False)
=============================================================================
"""

import math
import time
from typing import Callable, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

from control_module.MoveGroupClient import MoveGroupClient
from vision_module.WorldClient import TagWorld


DEFAULT_TAG_ID: int = 6


HOME_POSITION: List[float] = [
    -0.3442678993789787,
    -1.788839445251826,
    -0.05060219124037779,
    0.033443887503647206,
    -1.9649835829107563,
    0.00582217572740141,
]

NOD_POSITION: List[float] = [
    -0.38765505459411376,
    0.029880097347768208,
    1.4698936804656475,
    -0.03375929524228116,
    0.11421366641206165,
    0.0030747562775915375,
]


class ArmGestures:
    """Manager for joint-space gestures executed via MoveGroupClient."""

    def __init__(self, move_client: MoveGroupClient):
        self.move = move_client
        self._world: Optional[TagWorld] = None
        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None

        self._gestures: dict[str, Callable[..., bool]] = {
            'nod': self.nod,
            'shake': self.shake,
            'tilt': self.tilt,
            'search': self.search,
            'pin_point_tag': self.pin_point_tag,
            'pinpoint': self.pin_point_tag,
            'home': self.home,
        }

    def _init_vision(self):
        """Lazy initialization for TagWorld and TF listener."""
        if self._world is None:
            self._world = TagWorld(self.move.node, max_age_s=0.5)
        if self._tf_buffer is None:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self.move.node)

    def _latch_tag(self, tag_id: int, samples: int = 10, timeout_s: float = 5.0) -> Optional[np.ndarray]:
        """Median tag position over several vision samples."""
        positions: List[np.ndarray] = []
        deadline = time.monotonic() + timeout_s
        while len(positions) < samples and time.monotonic() < deadline:
            pos = self._world.position(tag_id)
            if pos is not None:
                positions.append(np.asarray(pos, dtype=np.float64))
            time.sleep(0.05)

        if len(positions) < max(3, samples // 2):
            self.move.node.get_logger().error(
                f"tag {tag_id}: only {len(positions)} detections in {timeout_s:.0f}s - too few to aim at")
            return None

        q = self._world.quality(tag_id) or {}
        self.move.node.get_logger().info(
            f"tag {tag_id} latched from {len(positions)} samples "
            f"[method={q.get('method')} fit_rms={q.get('fit_rms_m')}]")
        return np.median(np.stack(positions), axis=0)

    def _tool_pose(self, timeout_s: float = 10.0) -> Optional[PoseStamped]:
        """Lookup current end-effector tool pose via TF."""
        deadline = time.monotonic() + timeout_s
        ref_frame = self.move.ref_frame
        tool_link = self.move.tool_link
        while time.monotonic() < deadline:
            try:
                tf = self._tf_buffer.lookup_transform(
                    ref_frame, tool_link, rclpy.time.Time())
                pose = PoseStamped()
                pose.header.frame_id = ref_frame
                pose.pose.position.x = tf.transform.translation.x
                pose.pose.position.y = tf.transform.translation.y
                pose.pose.position.z = tf.transform.translation.z
                pose.pose.orientation = tf.transform.rotation
                return pose
            except Exception:
                time.sleep(0.1)
        self.move.node.get_logger().error(f"no TF {ref_frame} <- {tool_link}")
        return None

    def execute_gesture(self, name: str, base_joints: Optional[List[float]] = None,
                        plan_only: bool = False, velocity_scaling: Optional[float] = None,
                        tag_id: int = DEFAULT_TAG_ID) -> bool:
        """Dispatch gesture by string name."""
        name_clean = name.strip().lower()
        fn = self._gestures.get(name_clean)
        if fn is None:
            known = ', '.join(self._gestures.keys())
            self.move.node.get_logger().error(
                f"Unknown gesture '{name_clean}'. Known gestures: [{known}]")
            return False

        if name_clean == 'home':
            if velocity_scaling is not None:
                return fn(plan_only=plan_only, velocity_scaling=velocity_scaling)
            return fn(plan_only=plan_only)

        if name_clean in ('pin_point_tag', 'pinpoint'):
            if velocity_scaling is not None:
                return fn(tag_id=tag_id, base_joints=base_joints, plan_only=plan_only, velocity_scaling=velocity_scaling)
            return fn(tag_id=tag_id, base_joints=base_joints, plan_only=plan_only)

        if velocity_scaling is not None:
            return fn(base_joints=base_joints, plan_only=plan_only, velocity_scaling=velocity_scaling)
        return fn(base_joints=base_joints, plan_only=plan_only)

    def nod(self, base_joints: Optional[List[float]] = None, plan_only: bool = False,
            velocity_scaling: float = 1.0) -> bool:
        """
        Nodding gesture: pitches wrist (joint_5) up and down.

        :param base_joints: Starting 6-joint position array. Defaults to NOD_POSITION.
        :param plan_only: If True, only plans with MoveIt without executing movement.
        :param velocity_scaling: Speed scaling factor (0.40 = 40% max speed for snappy motion).
        """
        base = list(base_joints) if base_joints is not None else list(NOD_POSITION)

        nod_down = list(base); nod_down[4] += math.radians(30)
        nod_up   = list(base); nod_up[4]   -= math.radians(30)
        nod_end  = list(base)   # Return to base position

        sequence = [
            (nod_down, "nod_down1"),
            (nod_up,   "nod_up1"),
            (nod_down, "nod_down2"),
            # (nod_up,   "nod_up2"),
            (nod_end,  "nod_end"),
        ]

        self.move.node.get_logger().info(
            f"Starting gesture 'nod' ({'plan_only' if plan_only else 'execute'}) at speed {velocity_scaling:.2f}")

        for joint_target, label in sequence:
            if not self.move.go_joint(joint_target, plan_only=plan_only, label=label, velocity_scaling=velocity_scaling):
                self.move.node.get_logger().error(f"Gesture 'nod' aborted at step '{label}'")
                return False

        self.move.node.get_logger().info("Gesture 'nod' completed successfully.")
        self.home()
        return True

    def shake(self, base_joints: Optional[List[float]] = None, plan_only: bool = False,
              velocity_scaling: float = 1.0) -> bool:
        """
        Head-shake gesture: rotates wrist plane (joint_4 by +90°), swivels joint_5 left/right, and returns.

        :param base_joints: Starting 6-joint position array. Defaults to NOD_POSITION.
        :param plan_only: If True, only plans with MoveIt without executing movement.
        :param velocity_scaling: Speed scaling factor (0.40 = 40% max speed for snappy motion).
        """
        orig_base = list(base_joints) if base_joints is not None else list(NOD_POSITION)

        shake_base = list(orig_base)
        shake_base[3] += math.pi / 2  # Rotate wrist plane 90° for head shake (dq = +1.57 rad)

        left  = list(shake_base); left[4]  += math.radians(30)
        right = list(shake_base); right[4] -= math.radians(30)

        sequence = [
            (shake_base, "shake_rotate_plane"),
            (left,       "shake_left_1"),
            (right,      "shake_right_1"),
            (left,       "shake_left_2"),
            # (right,      "shake_right_2"),
            (shake_base, "shake_center"),
            (orig_base,  "shake_reset_plane"),
        ]

        self.move.node.get_logger().info(
            f"Starting gesture 'shake' ({'plan_only' if plan_only else 'execute'}) at speed {velocity_scaling:.2f}")

        for joint_target, label in sequence:
            if not self.move.go_joint(joint_target, plan_only=plan_only, label=label, velocity_scaling=velocity_scaling):
                self.move.node.get_logger().error(f"Gesture 'shake' aborted at step '{label}'")
                return False

        self.move.node.get_logger().info("Gesture 'shake' completed successfully.")
        self.home()
        return True

    def tilt(self, base_joints: Optional[List[float]] = None, plan_only: bool = False,
             velocity_scaling: float = 0.80) -> bool:
        """
        Head-tilt gesture: rotates wrist joint (joint_4) back and forth 3 times.

        :param base_joints: Starting 6-joint position array. Defaults to NOD_POSITION.
        :param plan_only: If True, only plans with MoveIt without executing movement.
        :param velocity_scaling: Speed scaling factor (0.40 = 40% max speed for snappy motion).
        """
        orig_base = list(base_joints) if base_joints is not None else list(NOD_POSITION)

        rot_right = list(orig_base); rot_right[3] += math.radians(50)
        rot_left  = list(orig_base); rot_left[3]  -= math.radians(50)

        sequence = [
            (rot_right, "tilt_right_1"),
            (rot_left,  "tilt_left_1"),
            (rot_right, "tilt_right_2"),
            # (rot_left,  "tilt_left_2"),
            (orig_base, "tilt_center"),
        ]

        self.move.node.get_logger().info(
            f"Starting gesture 'tilt' ({'plan_only' if plan_only else 'execute'}) at speed {velocity_scaling:.2f}")

        for joint_target, label in sequence:
            if not self.move.go_joint(joint_target, plan_only=plan_only, label=label, velocity_scaling=velocity_scaling):
                self.move.node.get_logger().error(f"Gesture 'tilt' aborted at step '{label}'")
                return False

        self.move.node.get_logger().info("Gesture 'tilt' completed successfully.")
        self.home()
        return True


    def search(self, base_joints: Optional[List[float]] = None, plan_only: bool = False,
               velocity_scaling: float = 0.15) -> bool:
        """
        Search gesture: slow scanning sweep using joint_4 and joint_5.
        """
        orig_base = list(base_joints) if base_joints is not None else list(NOD_POSITION)

        sweep_base = list(orig_base)
        sweep_base[3] += math.radians(90)  # Rotate joint_4 90° for horizontal panning

        left  = list(sweep_base); left[4]  -= math.radians(90)
        right = list(sweep_base); right[4] += math.radians(90)

        sequence = [
            (sweep_base, "search_rotate_plane"),
            (left,       "search_pan_left"),
            (sweep_base, "search_pan_center_1"),
            (right,      "search_pan_right"),
            (sweep_base, "search_pan_center_2"),
            (orig_base,  "search_reset_plane"),
        ]

        self.move.node.get_logger().info(
            f"Starting gesture 'search' ({'plan_only' if plan_only else 'execute'}) at speed {velocity_scaling:.2f}")

        for joint_target, label in sequence:
            if not self.move.go_joint(joint_target, plan_only=plan_only, label=label, velocity_scaling=velocity_scaling):
                self.move.node.get_logger().error(f"Gesture 'search' aborted at step '{label}'")
                return False

        self.move.node.get_logger().info("Gesture 'search' completed successfully.")
        return True

    def pin_point_tag(self, tag_id: int = DEFAULT_TAG_ID, base_joints: Optional[List[float]] = None,
                      plan_only: bool = False, velocity_scaling: float = 0.15,
                      step_m: float = 0.05, min_standoff_m: float = 0.25, min_z_m: float = 0.05) -> bool:
        """
        Search gesture that scans for tag_id in TagWorld, stops as soon as it is seen,
        and then steps toward that tag (like TagApproachNode).
        """
        self._init_vision()

        orig_base = list(base_joints) if base_joints is not None else list(NOD_POSITION)
        sweep_base = list(orig_base)
        sweep_base[3] += math.radians(90)

        left  = list(sweep_base); left[4]  -= math.radians(90)
        right = list(sweep_base); right[4] += math.radians(90)

        sequence = [
            (sweep_base, "search_rotate_plane"),
            (left,       "search_pan_left"),
            (sweep_base, "search_pan_center_1"),
            (right,      "search_pan_right"),
            (sweep_base, "search_pan_center_2"),
            (orig_base,  "search_reset_plane"),
        ]

        self.move.node.get_logger().info(
            f"Starting 'pin_point_tag' search for tag {tag_id} ({'plan_only' if plan_only else 'execute'})...")

        tag_found = False
        for joint_target, label in sequence:
            if tag_id in self._world.tag_ids():
                self.move.node.get_logger().info(f"Tag {tag_id} detected before '{label}'! Stopping search sweep.")
                tag_found = True
                break

            if not self.move.go_joint(joint_target, plan_only=plan_only, label=label, velocity_scaling=velocity_scaling):
                self.move.node.get_logger().error(f"Gesture 'pin_point_tag' search step '{label}' failed.")
                return False

            if tag_id in self._world.tag_ids():
                self.move.node.get_logger().info(f"Tag {tag_id} detected after '{label}'! Stopping search sweep.")
                tag_found = True
                break

        if not tag_found:
            # Poll for up to 2.0s in case vision data just arrived
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if tag_id in self._world.tag_ids():
                    tag_found = True
                    break
                time.sleep(0.1)

        if not tag_found:
            self.move.node.get_logger().error(f"Tag {tag_id} was not seen during search sweep.")
            return False

        # --- Tag found! Proceeding with approach step towards tag ---
        self.move.node.get_logger().info(f"Tag {tag_id} located! Latching position...")
        tag_pos = self._latch_tag(tag_id=tag_id)
        if tag_pos is None:
            return False

        start = self._tool_pose()
        if start is None:
            return False

        tool_pos = np.array([start.pose.position.x, start.pose.position.y, start.pose.position.z])
        delta = tag_pos - tool_pos
        distance = float(np.linalg.norm(delta))

        self.move.node.get_logger().info(
            f"Tool at ({tool_pos[0]:+.3f}, {tool_pos[1]:+.3f}, {tool_pos[2]:+.3f})\n"
            f"Tag {tag_id} at ({tag_pos[0]:+.3f}, {tag_pos[1]:+.3f}, {tag_pos[2]:+.3f})\n"
            f"Distance: {distance:.3f}m")

        if distance <= min_standoff_m:
            self.move.node.get_logger().info(f"Already within min_standoff ({min_standoff_m:.3f}m). Done.")
            return True

        step = min(step_m, distance - min_standoff_m)
        target_pos = tool_pos + (delta / distance) * step

        if target_pos[2] < min_z_m:
            self.move.node.get_logger().error(
                f"Target Z {target_pos[2]:+.3f} is below min_z ({min_z_m:.3f}m) - refusing motion.")
            return False

        target = PoseStamped()
        target.header.frame_id = self.move.ref_frame
        target.pose.orientation = start.pose.orientation
        target.pose.position.x = float(target_pos[0])
        target.pose.position.y = float(target_pos[1])
        target.pose.position.z = float(target_pos[2])

        self.move.node.get_logger().info(f"Stepping {step:.3f}m toward tag {tag_id}...")
        return self.move.go(target, plan_only=plan_only, label=f"pinpoint_approach_tag_{tag_id}")

    def home(self, plan_only: bool = False, velocity_scaling: float = 0.80) -> bool:
        """Move arm to predefined HOME_POSITION."""
        return self.move.go_joint(HOME_POSITION, plan_only=plan_only, label="home", velocity_scaling=velocity_scaling)
