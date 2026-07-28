"""
Gesture definitions for the Kinova arm.

To add a new gesture:
  1. Define a method here in GestureMixin.
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
