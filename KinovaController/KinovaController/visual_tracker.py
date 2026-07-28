"""
Visual Tracking & Search Sweep Mixin for Kinova Gen3.

Handles AprilTag pose subscription, dynamic 2D/3D centering loop (orient_to_person),
and slow search sweep (_slow_search_sweep) using joint_4 and joint_5.
"""

import time
import math
import threading
from geometry_msgs.msg import PoseStamped
from .positions import NOD_POSITION

# ---------------------------------------------------------------------------
# Visual Tracking Parameters
# ---------------------------------------------------------------------------
_SCALE_H = 0.20          # joint_1 (Basis, horizontal)
_SCALE_V = 0.15          # joint_5 (Handgelenk-Neigung, vertikal)
_DEAD_ZONE = 0.05        # ~2.8°
_SEARCH_TIMEOUT_S = 2.0  # Wait time per step for tag message
_MAX_ALIGN_TIME_S = 10.0 # Max alignment loop duration
_STEP_DURATION_S = 2.0   # Trajectory duration per step


class VisualTracker:
    """Mixin providing visual tracking and search capabilities."""

    def _init_visual_tracker(self):
        """Initialize tracker internal states."""
        self._last_position: tuple[float, float, float] | None = None
        self._last_orientation: tuple[float, float, float, float] | None = None
        self._current_oriented_position: list[float] = list(NOD_POSITION)
        self._tag_event = threading.Event()

    def _tag_callback(self, msg: PoseStamped) -> None:
        """
        Empfängt die 3D-Pose des AprilTags vom VisionModule, loggt alle empfangenen
        Informationen und setzt das Event für die Ausrichtung / IK-Bewegung.
        """
        p = msg.pose.position
        q = msg.pose.orientation
        dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        self._last_position = (p.x, p.y, p.z)
        self._last_orientation = (q.x, q.y, q.z, q.w)
        self._tag_event.set()

        self.get_logger().info(
            f'[AprilTag empfangen] '
            f'Position: (x={p.x:+.3f}m, y={p.y:+.3f}m, z={p.z:+.3f}m) | '
            f'Entfernung: {dist:.2f}m | '
            f'Orientierung Quaternion: (x={q.x:.2f}, y={q.y:.2f}, z={q.z:.2f}, w={q.w:.2f})'
        )

    def _slow_search_sweep(self) -> bool:
        """
        Schwenkt den Kamera-Kopf langsam über joint_4 und joint_5 (wie bei shake, aber langsam),
        um im Raum nach einem AprilTag zu suchen.

        Gibt True zurück, sobald ein Tag während des Sweeps erkannt wurde, sonst False.
        """
        self.get_logger().info('[Suche] Kein AprilTag im Sichtfeld – starte langsamen Umschau-Sweep mit joint_4 & joint_5...')

        # Basis für den Umschau-Sweep: joint_4 um 90° drehen (damit joint_5 horizontal schwenkt)
        base = list(NOD_POSITION)
        base[3] += math.pi / 2  # joint_4 (Kamera-Drehung)

        # Schwenkpositionen für joint_5 (langsam links, mitte, rechts, mitte)
        sweep_offsets = [-0.4, 0.0, 0.4, 0.0]

        for offset in sweep_offsets:
            sweep_pos = list(base)
            sweep_pos[4] += offset

            self.get_logger().info(f'[Suche] Schwenke Kamera langsam (joint_5 Offset: {offset:+.2f} rad)...')
            self._current_oriented_position = sweep_pos
            self.move_arm_to(sweep_pos, duration=3)

            # Während und nach der Bewegung auf Tag-Signal prüfen
            self._tag_event.clear()
            found = self._tag_event.wait(timeout=3.5)

            if found:
                self.get_logger().info('[Suche] AprilTag während des Umschauens entdeckt!')
                return True

        return False

    def orient_to_person(self) -> None:
        """
        Richtet den Arm in einer dynamischen Regelschleife zur erkannten Person aus,
        bevor eine Geste ausgeführt wird.

        Falls aktuell kein AprilTag im Sichtfeld ist, wird ein langsamer Umschau-Sweep
        über joint_4/joint_5 gestartet, um den Tag zu suchen.
        """
        self.get_logger().info('[Ausrichtung] Starte Suche & Ausrichtung zur Person...')
        start_time = time.time()
        step_count = 0

        while time.time() - start_time < _MAX_ALIGN_TIME_S:
            step_count += 1
            self._tag_event.clear()

            self.get_logger().info(f'[Ausrichtung Schritt {step_count}] Warte auf AprilTag-Signal (max {_SEARCH_TIMEOUT_S}s)...')
            tag_found = self._tag_event.wait(timeout=_SEARCH_TIMEOUT_S)

            if not tag_found:
                self.get_logger().info('[Ausrichtung] Keinen AprilTag im aktuellen Sichtfeld – starte Umschau-Sweep...')
                tag_found = self._slow_search_sweep()

            if not tag_found:
                self.get_logger().warn('[Ausrichtung] Auch nach dem Umschau-Sweep kein AprilTag gefunden.')
                break

            x, y, z = self._last_position
            h_angle = math.atan2(x, z)
            v_angle = math.atan2(y, z)

            self.get_logger().info(
                f'[Ausrichtung Schritt {step_count}] Tag-Position im Kamera-Frame: '
                f'x={x:+.3f}m, y={y:+.3f}m, z={z:+.3f}m | '
                f'Winkelversatz: Horiz={math.degrees(h_angle):+.1f}°, Vert={math.degrees(v_angle):+.1f}°'
            )

            # Prüfe, ob die Person im Bild zentriert ist (innerhalb Deadzone ~2.8°)
            if abs(h_angle) < _DEAD_ZONE and abs(v_angle) < _DEAD_ZONE:
                self.get_logger().info('[Ausrichtung] AprilTag erfolgreich im Kamerabild zentriert!')
                break

            # Zielposition berechnen
            target = list(self._current_oriented_position)
            target[0] += h_angle * _SCALE_H   # joint_1: Basis horizontal
            target[4] += v_angle * _SCALE_V   # joint_5: Handgelenk vertikal

            self.get_logger().info(
                f'[Ausrichtung Schritt {step_count}] Passe Gelenke an: '
                f'joint_1 += {h_angle * _SCALE_H:+.3f} rad, joint_5 += {v_angle * _SCALE_V:+.3f} rad'
            )

            self._current_oriented_position = target
            self.move_arm_to(target, duration=int(_STEP_DURATION_S))
            time.sleep(_STEP_DURATION_S)

        self.get_logger().info('[Ausrichtung] Fertig. Starte Geste...')
        time.sleep(0.5)
