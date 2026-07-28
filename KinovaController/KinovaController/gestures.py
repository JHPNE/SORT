"""
Gesture definitions for the Kinova Gen3 Arm (6-DoF).

=============================================================================
PHYSIKALISCHE GELENK-GESCHWINDIGKEITSGRENZEN (Kinova Gen3 Hardware-Limits):
  - Joint 1, 2, 3 (Basis & Schulter) : Max 1.396 rad/s (~80.0°/s)
  - Joint 4, 5, 6 (Handgelenk/Kamera): Max 1.221 rad/s (~70.0°/s)
=============================================================================
"""

import math
from .positions import HOME_POSITION, NOD_POSITION


class ArmGestures:
    """Mixin providing predefined gesture movement methods for KinovaMover."""

    def nod(self) -> None:
        """
        Nodding gesture: pitches wrist (joint_5) up and down.
        
        Geschwindigkeits-Berechnung:
          - Gelenk: joint_5 (Max Limit: 1.221 rad/s)
          - Delta Angle: dq = 0.30 rad (~17.2°)
          - Delta Time : dt = 0.8s
          - Speed      : v = 0.30 / 0.8 = 0.375 rad/s (ca. 30% des Max-Limits -> Sicher & Snappy)
        """
        base = list(getattr(self, '_current_oriented_position', NOD_POSITION))
        nod_down = list(base); nod_down[4] += 0.30   # Wrist pitch down (+0.30 rad)
        nod_up   = list(base); nod_up[4]   -= 0.30   # Wrist pitch up   (-0.30 rad)
        nod_end  = list(base)                        # Return to oriented base position

        self.move_sequence([
            (nod_down, 0.8),  # dt = 0.8s  (v = 0.375 rad/s)
            (nod_up,   1.6),  # dt = 0.8s  (v = 0.750 rad/s für 0.60 rad Gesamtweg)
            (nod_end,  2.4),  # dt = 0.8s  (v = 0.375 rad/s)
        ])

    def shake(self) -> None:
        """
        Head-shake gesture: swivels wrist (joint_5) left and right after 90° joint_4 rotation.
        
        Geschwindigkeits-Berechnung:
          - Turn joint_4 by 90° (1.57 rad):
            dt = 1.6s -> v = 1.57 / 1.6 = 0.98 rad/s (ca. 80% von max 1.221 rad/s -> Snappy & hardware-sicher)
          - Swivel joint_5 left/right (dq = 0.50 rad):
            dt = 0.5s -> v = 0.50 / 0.5 = 1.00 rad/s (ca. 82% von max 1.221 rad/s -> Sehr zackig)
        """
        orig_base = list(getattr(self, '_current_oriented_position', NOD_POSITION))

        shake_base = list(orig_base)
        shake_base[3] += math.pi / 2  # Rotate wrist plane 90° for head shake (dq = +1.57 rad)

        left  = list(shake_base); left[4]  += 0.25   # Shake left  (dq = +0.25 rad)
        right = list(shake_base); right[4] -= 0.25   # Shake right (dq = -0.25 rad)

        self.move_sequence([
            (shake_base, 1.6),  # dt = 1.6s  | joint_4 = +1.57 rad (v = 0.98 rad/s <= 1.221 MAX)
            (left,       2.1),  # dt = 0.5s  | joint_5 = +0.25 rad (v = 0.50 rad/s)
            (right,      2.6),  # dt = 0.5s  | joint_5 = -0.50 rad (v = 1.00 rad/s <= 1.221 MAX)
            (left,       3.1),  # dt = 0.5s  | joint_5 = +0.50 rad (v = 1.00 rad/s)
            (right,      3.6),  # dt = 0.5s  | joint_5 = -0.50 rad (v = 1.00 rad/s)
            (shake_base, 4.1),  # dt = 0.5s  | joint_5 = +0.25 rad (v = 0.50 rad/s)
            (orig_base,  5.7),  # dt = 1.6s  | joint_4 = -1.57 rad (v = 0.98 rad/s <= 1.221 MAX)
        ])

    def search(self) -> None:
        """
        Search gesture: slow scanning sweep using joint_4 and joint_5.
        
        Geschwindigkeits-Berechnung:
          - Turn joint_4 by 90°: dt = 1.6s (v = 0.98 rad/s)
          - Sweep joint_5 left/right (dq = 0.80 rad): dt = 2.0s (v = 0.40 rad/s -> Fließender Such-Sweep)
        """
        orig_base = list(getattr(self, '_current_oriented_position', NOD_POSITION))

        sweep_base = list(orig_base)
        sweep_base[3] += math.pi / 2  # Rotate joint_4 90° for horizontal panning

        left  = list(sweep_base); left[4]  -= 0.4   # Pan left  (dq = -0.4 rad)
        right = list(sweep_base); right[4] += 0.4   # Pan right (dq = +0.4 rad)

        self.move_sequence([
            (sweep_base, 1.6),  # dt = 1.6s  | joint_4 = +1.57 rad (v = 0.98 rad/s)
            (left,       3.6),  # dt = 2.0s  | joint_5 = -0.40 rad (v = 0.20 rad/s)
            (sweep_base, 5.6),  # dt = 2.0s  | joint_5 = +0.40 rad (v = 0.20 rad/s)
            (right,      7.6),  # dt = 2.0s  | joint_5 = +0.40 rad (v = 0.20 rad/s)
            (sweep_base, 9.6),  # dt = 2.0s  | joint_5 = -0.40 rad (v = 0.20 rad/s)
            (orig_base,  11.2), # dt = 1.6s  | joint_4 = -1.57 rad (v = 0.98 rad/s)
        ])

    def home(self) -> None:
        """
        Move arm to predefined HOME_POSITION.
        
        Geschwindigkeits-Berechnung:
          - Gesamtdauer: 5.0s für freie Bewegung aus beliebiger Haltung in HOME_POSITION.
        """
        self._current_oriented_position = list(HOME_POSITION)
        self.move_arm_to(HOME_POSITION, duration=5)




