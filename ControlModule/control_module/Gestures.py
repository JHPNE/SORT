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
from control_module.MoveGroupClient import MoveGroupClient


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
        self._gestures: dict[str, Callable[..., bool]] = {
            'nod': self.nod,
            'shake': self.shake,
            'tilt': self.tilt,
            'search': self.search,
            'home': self.home,
        }

    def execute_gesture(self, name: str, base_joints: Optional[List[float]] = None,
                        plan_only: bool = False, velocity_scaling: float = 0.40) -> bool:
        """Dispatch gesture by string name."""
        name_clean = name.strip().lower()
        fn = self._gestures.get(name_clean)
        if fn is None:
            known = ', '.join(self._gestures.keys())
            self.move.node.get_logger().error(
                f"Unknown gesture '{name_clean}'. Known gestures: [{known}]")
            return False
        return fn(base_joints=base_joints, plan_only=plan_only, velocity_scaling=velocity_scaling) if name_clean != 'home' else fn(plan_only=plan_only, velocity_scaling=velocity_scaling)

    def nod(self, base_joints: Optional[List[float]] = None, plan_only: bool = False,
            velocity_scaling: float = 0.40) -> bool:
        """
        Nodding gesture: pitches wrist (joint_5) up and down.

        :param base_joints: Starting 6-joint position array. Defaults to NOD_POSITION.
        :param plan_only: If True, only plans with MoveIt without executing movement.
        :param velocity_scaling: Speed scaling factor (0.40 = 40% max speed for snappy motion).
        """
        base = list(base_joints) if base_joints is not None else list(NOD_POSITION)

        nod_down = list(base); nod_down[4] += 0.30   # Wrist pitch down (+0.30 rad / ~17.2°)
        nod_up   = list(base); nod_up[4]   -= 0.30   # Wrist pitch up   (-0.30 rad / ~17.2°)
        nod_end  = list(base)                        # Return to base position

        sequence = [
            (nod_down, "nod_down"),
            (nod_up,   "nod_up"),
            (nod_end,  "nod_end"),
        ]

        self.move.node.get_logger().info(
            f"Starting gesture 'nod' ({'plan_only' if plan_only else 'execute'}) at speed {velocity_scaling:.2f}")

        for joint_target, label in sequence:
            if not self.move.go_joint(joint_target, plan_only=plan_only, label=label, velocity_scaling=velocity_scaling):
                self.move.node.get_logger().error(f"Gesture 'nod' aborted at step '{label}'")
                return False

        self.move.node.get_logger().info("Gesture 'nod' completed successfully.")
        return True

    def shake(self, base_joints: Optional[List[float]] = None, plan_only: bool = False,
              velocity_scaling: float = 0.40) -> bool:
        """
        Head-shake gesture: rotates wrist plane (joint_4 by +90°), swivels joint_5 left/right, and returns.

        :param base_joints: Starting 6-joint position array. Defaults to NOD_POSITION.
        :param plan_only: If True, only plans with MoveIt without executing movement.
        :param velocity_scaling: Speed scaling factor (0.40 = 40% max speed for snappy motion).
        """
        orig_base = list(base_joints) if base_joints is not None else list(NOD_POSITION)

        shake_base = list(orig_base)
        shake_base[3] += math.pi / 2  # Rotate wrist plane 90° for head shake (dq = +1.57 rad)

        left  = list(shake_base); left[4]  += 0.25   # Shake left  (dq = +0.25 rad)
        right = list(shake_base); right[4] -= 0.25   # Shake right (dq = -0.25 rad)

        sequence = [
            (shake_base, "shake_rotate_plane"),
            (left,       "shake_left_1"),
            (right,      "shake_right_1"),
            (left,       "shake_left_2"),
            (right,      "shake_right_2"),
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
        return True

    def tilt(self, base_joints: Optional[List[float]] = None, plan_only: bool = False,
             velocity_scaling: float = 0.40) -> bool:
        """
        Head-tilt gesture: rotates wrist joint (joint_4) back and forth 3 times.

        :param base_joints: Starting 6-joint position array. Defaults to NOD_POSITION.
        :param plan_only: If True, only plans with MoveIt without executing movement.
        :param velocity_scaling: Speed scaling factor (0.40 = 40% max speed for snappy motion).
        """
        orig_base = list(base_joints) if base_joints is not None else list(NOD_POSITION)

        rot_right = list(orig_base); rot_right[3] += 0.35   # Rotate joint_4 right (dq = +0.35 rad)
        rot_left  = list(orig_base); rot_left[3]  -= 0.35   # Rotate joint_4 left  (dq = -0.35 rad)

        sequence = [
            (rot_right, "tilt_right_1"),
            (rot_left,  "tilt_left_1"),
            (rot_right, "tilt_right_2"),
            (rot_left,  "tilt_left_2"),
            (rot_right, "tilt_right_3"),
            (rot_left,  "tilt_left_3"),
            (orig_base, "tilt_center"),
        ]

        self.move.node.get_logger().info(
            f"Starting gesture 'tilt' ({'plan_only' if plan_only else 'execute'}) at speed {velocity_scaling:.2f}")

        for joint_target, label in sequence:
            if not self.move.go_joint(joint_target, plan_only=plan_only, label=label, velocity_scaling=velocity_scaling):
                self.move.node.get_logger().error(f"Gesture 'tilt' aborted at step '{label}'")
                return False

        self.move.node.get_logger().info("Gesture 'tilt' completed successfully.")
        return True


    def search(self, base_joints: Optional[List[float]] = None, plan_only: bool = False,
               velocity_scaling: float = 0.15) -> bool:
        """
        Search gesture: slow scanning sweep using joint_4 and joint_5.
        """
        orig_base = list(base_joints) if base_joints is not None else list(NOD_POSITION)

        sweep_base = list(orig_base)
        sweep_base[3] += math.pi / 2  # Rotate joint_4 90° for horizontal panning

        left  = list(sweep_base); left[4]  -= 0.4   # Pan left  (dq = -0.4 rad)
        right = list(sweep_base); right[4] += 0.4   # Pan right (dq = +0.4 rad)

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

    def home(self, plan_only: bool = False, velocity_scaling: float = 0.15) -> bool:
        """Move arm to predefined HOME_POSITION."""
        return self.move.go_joint(HOME_POSITION, plan_only=plan_only, label="home", velocity_scaling=velocity_scaling)
