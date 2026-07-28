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
_MAX_ALIGN_TIME_S = 30.0 # Max alignment loop duration (30 seconds)
_STEP_DURATION_S = 2.0   # Trajectory duration per step


class VisualTracker:
    """Mixin providing visual tracking and search capabilities for KinovaMover."""

    def _init_visual_tracker(self) -> None:
        """Initialize tracker internal states."""
        self._last_position: tuple[float, float, float] | None = None
        self._last_orientation: tuple[float, float, float, float] | None = None
        self._current_oriented_position: list[float] = list(NOD_POSITION)
        self._tag_event = threading.Event()
        self._last_log_time: float = 0.0

    def _tag_callback(self, msg: PoseStamped) -> None:
        """
        Empfängt die 3D-Pose des AprilTags vom VisionModule, loggt empfangene
        Informationen gedrosselt und setzt das Event für die Ausrichtung / IK-Bewegung.
        """
        p = msg.pose.position
        q = msg.pose.orientation
        dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        self._last_position = (p.x, p.y, p.z)
        self._last_orientation = (q.x, q.y, q.z, q.w)
        self._tag_event.set()

        # Dynamic Obstacle Tracking für Weg 2: Tag-Position in Worldspace umrechnen & registrieren
        if hasattr(self, 'ik_solver') and self.ik_solver.is_available and self.ik_solver.model:
            from .positions import HOME_POSITION
            q_current = getattr(self, '_current_oriented_position', HOME_POSITION)
            fk_res = self.ik_solver.get_forward_kinematics(q_current)
            if fk_res is not None:
                p_ee, R_ee = fk_res
                target_cam = np.array([p.x, p.y, p.z])
                target_base = p_ee + R_ee @ target_cam
                self.ik_solver.collision_handler.update_dynamic_obstacle(
                    "apriltag_person",
                    float(target_base[0]),
                    float(target_base[1]),
                    float(target_base[2]),
                    radius=0.20
                )

        now = time.time()
        if now - self._last_log_time >= 2.0:
            self._last_log_time = now
            self.get_logger().info(
                f'[AprilTag empfangen] '
                f'Position: (x={p.x:+.3f}m, y={p.y:+.3f}m, z={p.z:+.3f}m) | '
                f'Entfernung: {dist:.2f}m | '
                f'Orientierung Quaternion: (x={q.x:.2f}, y={q.y:.2f}, z={q.z:.2f}, w={q.w:.2f})'
            )

    def _worldspace_tags_callback(self, msg: String) -> None:
        """
        Empfängt das aggregierte WorldSpace-Paket von WorldSpaceNode (/vision/tags).
        Trägt alle im Raum erkannten Tags direkt in den Pinocchio Collision Handler ein
        und speichert ihre WorldSpace-Koordinaten für IK-Anfahrten.
        """
        if not hasattr(self, '_worldspace_tag_poses'):
            self._worldspace_tag_poses = {}

        try:
            from vision_module import TagMessage
            packet = TagMessage.decode(msg.data)
            observations = TagMessage.to_observations(packet)
            for obs in observations:
                x, y, z = obs.pos
                self._worldspace_tag_poses[obs.id] = (float(x), float(y), float(z))

                if hasattr(self, 'ik_solver') and self.ik_solver.is_available and self.ik_solver.model:
                    self.ik_solver.collision_handler.update_dynamic_obstacle(
                        f"world_tag_{obs.id}",
                        float(x),
                        float(y),
                        float(z),
                        radius=0.20
                    )
        except Exception:
            pass

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

    def orient_to_arm_camera_tag(self) -> None:
        """
        [ARM-KAMERA / EYE-IN-HAND]
        Richtet den Kamera-Kopf in einer dynamischen Regelschleife zum AprilTag der Arm-Kamera aus.
        """
        self.get_logger().info('[ArmKamera-Ausrichtung] Starte Suche & Ausrichtung zum Arm-Kamera AprilTag...')
        start_time = time.time()
        step_count = 0

        centered = False
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
                centered = True
                self.get_logger().info(
                    f'[Ausrichtung STATUS] ERFOLGREICH ZENTRIERT! AprilTag ist genau in Bildmitte '
                    f'(Horiz={math.degrees(h_angle):+.1f}°, Vert={math.degrees(v_angle):+.1f}° <= {_DEAD_ZONE:.2f} rad).'
                )
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

        if centered:
            self.get_logger().info(f'[Ausrichtung ERFOLGREICH] Kamera zentriert nach {step_count} Schritten ({time.time() - start_time:.1f}s).')
        else:
            self.get_logger().warn(f'[Ausrichtung TIMEOUT] 30s Zeitlimit abgelaufen – fahre mit bestehender Haltung fort.')

        time.sleep(0.5)

    # Alias für Abwärtskompatibilität
    orient_to_person = orient_to_arm_camera_tag

    def orient_to_worldspace_tag(self, tag_id: int = 3) -> bool:
        """
        Richtet den Kamera-Kopf gezielt auf ein Tag aus dem WorldSpaceNode (/vision/tags) aus,
        selbst wenn es sich außerhalb des aktuellen Armkamera-Sichtfeldes befindet.
        """
        world_poses = getattr(self, '_worldspace_tag_poses', {})
        if tag_id not in world_poses:
            self.get_logger().error(f'[WorldSpace-Ausrichtung] Kein Tag mit ID {tag_id} auf /vision/tags bekannt!')
            return False

        x_base, y_base, z_base = world_poses[tag_id]
        self.get_logger().info(f'[WorldSpace-Ausrichtung] Richtet Kamera auf Tag {tag_id} bei WorldSpace ({x_base:+.3f}m, {y_base:+.3f}m, {z_base:+.3f}m) aus...')

        # Ausrichtungs-Winkel berechnen (Azimut & Elevation relative zur Roboterbasis)
        pan_angle = math.atan2(y_base, x_base)
        dist_xy = math.sqrt(x_base*x_base + y_base*y_base)
        tilt_angle = math.atan2(z_base, dist_xy)

        target_pos = list(getattr(self, '_current_oriented_position', NOD_POSITION))
        target_pos[3] = pan_angle   # joint_4 (Horizontale Ausrichtung)
        target_pos[4] = -tilt_angle # joint_5 (Vertikale Neigung)

        self._current_oriented_position = target_pos
        self.move_arm_to(target_pos, duration=2)
        return True
