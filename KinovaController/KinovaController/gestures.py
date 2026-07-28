"""
Gesture definitions for the Kinova Gen3 Arm.

To add a new gesture:
  1. Define a method here in ArmGestures.
  2. Register it in KinovaMover._gestures (arm_mover.py).
"""

import math
from .positions import HOME_POSITION, NOD_POSITION


class ArmGestures:
    """Mixin providing predefined gesture movement methods for KinovaMover."""

    def nod(self) -> None:
        """Nodding gesture: pitches the wrist up and down."""
        base = list(getattr(self, '_current_oriented_position', NOD_POSITION))
        nod_down = list(base); nod_down[4] += 0.4   # Wrist pitch down
        nod_up   = list(base); nod_up[4]   -= 0.4   # Wrist pitch up
        nod_end  = list(base)                       # Return to oriented base position

        self.move_sequence([
            (nod_down, 2),
            (nod_up,   4),
            (nod_end,  6),
        ])

    def shake(self) -> None:
        """Head-shake gesture: rolls/swivels the wrist left and right."""
        base = list(getattr(self, '_current_oriented_position', NOD_POSITION))
        base[3] += math.pi / 2  # Rotate wrist plane 90° for head shake
        left  = list(base); left[4]  += 0.3         # Shake left
        right = list(base); right[4] -= 0.3         # Shake right

        self.move_sequence([
            (base,  1),
            (left,  2),
            (right, 3),
            (left,  4),
            (right, 5),
            (base,  6),
        ])

    def search(self) -> None:
        """Search gesture: slow scanning sweep using joint_4 and joint_5."""
        base = list(getattr(self, '_current_oriented_position', NOD_POSITION))
        base[3] += math.pi / 2  # Rotate joint_4 90° for horizontal panning

        left  = list(base); left[4]  -= 0.4         # Pan left
        right = list(base); right[4] += 0.4         # Pan right

        self.move_sequence([
            (left,  3),
            (base,  6),
            (right, 9),
            (base, 12),
        ])



