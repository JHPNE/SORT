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
        base = list(getattr(self, '_current_oriented_position', NOD_POSITION))
        nod_down = list(base);  nod_down[4] += 0.4   # Wrist pitch down
        nod_up   = list(base);  nod_up[4]   -= 0.4   # Wrist pitch up
        nod_end  = list(base);                       # Return to oriented base position

        self.move_sequence([
            (nod_down, 2),
            (nod_up,   4),
            (nod_end,  6),
        ])

    def shake(self):
        import math
        base  = list(getattr(self, '_current_oriented_position', NOD_POSITION))
        base[3] += math.pi / 2  # Rotate wrist plane 90° for head shake
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

