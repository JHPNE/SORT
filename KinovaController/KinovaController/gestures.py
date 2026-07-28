"""
Gesture definitions for the Kinova arm.

To add a new gesture:
  1. Define a method here in ArmGestures.
  2. Register it in KinovaMover._gestures (arm_mover.py).
"""

from .positions import HOME_POSITION, NOD_POSITION


class ArmGestures:
    """Mixin that provides gesture methods for KinovaMover."""

    def nod(self):
        nod_down = list(NOD_POSITION);  nod_down[4] += 0.5   # Wrist pitch down
        nod_up   = list(NOD_POSITION);  nod_up[4]   -= 0.5   # Wrist pitch up
        nod_end  = list(HOME_POSITION); nod_end[4]  += 0.5   # Return near home

        self.move_sequence([
            (nod_down, 2),
            (nod_up,   4),
            (nod_end,  6),
        ])

    def shake(self):
        import math
        # Rotate joint_4 (index 3) by 90° to reorient the wrist plane,
        # then oscillate joint_5 (index 4) left/right for a head shake.
        base  = list(NOD_POSITION);  base[3]  += math.pi / 2  # Rotate wrist plane 90°
        left  = list(base);          left[4]  += 0.3           # Shake left
        right = list(base);          right[4] -= 0.3           # Shake right

        self.move_sequence([
            (base,  1),
            (left,  2),
            (right, 3),
            (left,  4),
            (right, 5),
            (base,  6),
        ])

