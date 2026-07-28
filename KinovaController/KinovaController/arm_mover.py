import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from topic_handler.TopicList import TopicList

from .gestures import ArmGestures
from .positions import HOME_POSITION, NOD_POSITION
from .ik_solver import KinovaIKSolver, IKMovement

# ---------------------------------------------------------------------------
# Tracking-Parameter – hier anpassen wenn nötig
# ---------------------------------------------------------------------------

# Verstärkung: wie viel Gelenkwinkel (rad) pro normalisierter Bildeinheit
_SCALE_H = 0.4    # joint_1 (Basis, horizontal)
_SCALE_V = 0.25   # joint_5 (Handgelenk-Neigung, vertikal)

# Korrekturen kleiner als DEAD_ZONE werden ignoriert (Zittern vermeiden)
_DEAD_ZONE = 0.08

# Wie lange pro Schleifendurchlauf auf ein neues Tag-Signal gewartet wird
_SEARCH_TIMEOUT_S = 2.0

# Maximale Dauer der Ausrichtungs-Schleife (Sekunden)
_MAX_ALIGN_TIME_S = 8.0

# Dauer eines einzelnen Korrekturschritts (Sekunden)
_STEP_DURATION_S = 1.0


class KinovaMover(Node, ArmGestures, IKMovement):
    def __init__(self):
        super().__init__('kinova_mover')

        self.topics = TopicList()

        # Inverse Kinematics Solver (Pinocchio)
        self.ik_solver = KinovaIKSolver()

        # URDF automatisch vom ROS 2 Topic empfangen
        self.create_subscription(
            String,
            '/robot_description',
            self._robot_description_callback,
            10
        )

        self._publisher = self.create_publisher(
            JointTrajectory,
            self.topics.arm.joint_trajectory.name,
            10
        )

        # Map gesture names → methods. Add new gestures here.
        self._gestures: dict[str, callable] = {
            'nod':   self.nod,
            'shake': self.shake,
        }

        # ------------------------------------------------------------------
        # AprilTag-Tracking-Zustand
        #
        # Das VisionModule publisht auf /vision/apriltag_pose einen
        # geometry_msgs/PoseStamped mit der 3D-Pose des erkannten Tags
        # im Kamera-Koordinatensystem:
        #
        #   pose.position.x : float  Horizontale Position des Tags (Kamera-Frame)
        #                             Positiv = Tag rechts vom Kameramittelpunkt
        #                             Negativ = Tag links  vom Kameramittelpunkt
        #
        #   pose.position.y : float  Vertikale Position des Tags (Kamera-Frame)
        #                             Positiv = Tag unterhalb des Kameramittelpunkts
        #                             Negativ = Tag oberhalb des Kameramittelpunkts
        #
        #   pose.position.z : float  Tiefe – Distanz der Kamera zum Tag (Meter)
        #                             Wird für orient_to_person() nicht benötigt,
        #                             aber für späteres Greifen (IK) essenziell.
        #
        #   pose.orientation : Quaternion  Orientierung des Tags im Kamera-Frame
        #                             Wird für Greifen benötigt (Greif-Achse).
        #                             Für Tracking ignoriert.
        #
        # WICHTIG für das VisionModule:
        #   - Nur publizieren wenn der konfigurierte Tag sichtbar ist.
        #   - Kein "Tag verloren"-Message senden – einfach aufhören zu publizieren.
        #   - Ausbleibende Nachrichten = Tag nicht sichtbar.
        #   - Koordinatensystem: Standard-Kamera-Frame (Z vorwärts, X rechts, Y nach unten).
        #   - Einheiten: Meter.
        # ------------------------------------------------------------------
        self._last_position: tuple[float, float, float] | None = None
        self._last_orientation: tuple[float, float, float, float] | None = None
        self._tag_event = threading.Event()

        self.create_subscription(
            PoseStamped,
            self.topics.arm.apriltag_pose.name,
            self._tag_callback,
            10
        )

        self.create_subscription(
            String,
            self.topics.arm.gesture.name,
            self._gesture_callback,
            10
        )

        self.get_logger().info('Warte 2 Sekunden, bis das System bereit ist...')
        time.sleep(2.0)
        self.move_arm_to(HOME_POSITION, duration=5)
        time.sleep(5.0)

    def _robot_description_callback(self, msg: String) -> None:
        """Laedt die URDF aus /robot_description in den Pinocchio IK-Solver."""
        if self.ik_solver.model is None and msg.data:
            if self.ik_solver.load_urdf(msg.data):
                self.get_logger().info('URDF erfolgreich aus /robot_description geladen (Pinocchio IK bereit).')

    # ------------------------------------------------------------------
    # Core movement
    # ------------------------------------------------------------------

    def move_sequence(self, steps: list[tuple[list[float], int]]):
        """
        Publish a sequence of joint positions as a single JointTrajectory message.

        :param steps: List of (joint_positions, time_from_start_sec) tuples.
        """
        for i, (positions, _) in enumerate(steps):
            if len(positions) != 6:
                self.get_logger().error(
                    f'Step {i}: Invalid number of joints. Expected: 6, Received: {len(positions)}'
                )
                return

        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        for positions, t in steps:
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start.sec = int(t)
            msg.points.append(point)

        self.get_logger().info(f'Sending sequence of {len(steps)} point(s) to Kinova Gen3...')
        self._publisher.publish(msg)

    def move_arm_to(self, joint_positions=None, duration=10):
        """Convenience wrapper to move to a single joint position."""
        if joint_positions is None:
            joint_positions = HOME_POSITION

        self.move_sequence([(joint_positions, duration)])

    # ------------------------------------------------------------------
    # AprilTag Tracking & Log-Reaktion
    # ------------------------------------------------------------------

    def _tag_callback(self, msg: PoseStamped) -> None:
        """
        Empfängt die 3D-Pose des AprilTags vom VisionModule, loggt alle empfangenen
        Informationen und setzt das Event für die Ausrichtung / IK-Bewegung.
        """
        import math

        p = msg.pose.position
        q = msg.pose.orientation
        dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        self._last_position = (p.x, p.y, p.z)
        self._last_orientation = (q.x, q.y, q.z, q.w)
        self._tag_event.set()

        # Detailliertes Log über empfangene Nachrichten
        self.get_logger().info(
            f'[AprilTag empfangen] '
            f'Position: (x={p.x:+.3f}m, y={p.y:+.3f}m, z={p.z:+.3f}m) | '
            f'Entfernung: {dist:.2f}m | '
            f'Orientierung Quaternion: (x={q.x:.2f}, y={q.y:.2f}, z={q.z:.2f}, w={q.w:.2f})'
        )

    def orient_to_person(self) -> None:
        """
        Richtet den Arm in einer dynamischen Regelschleife zur erkannten Person aus,
        bevor eine Geste ausgeführt wird.

        Verfolgt die 3D-Position kontinuierlich und passt die Gelenke in Schleifendurchläufen an,
        bis der AprilTag zentriert ist (innerhalb _DEAD_ZONE) oder das Timeout (_MAX_ALIGN_TIME_S) erreicht ist.
        """
        import math

        self.get_logger().info('Starte dynamische Ausrichtung zur Person...')
        start_time = time.time()

        while time.time() - start_time < _MAX_ALIGN_TIME_S:
            self._tag_event.clear()
            tag_found = self._tag_event.wait(timeout=_SEARCH_TIMEOUT_S)

            if not tag_found:
                self.get_logger().warn(
                    'AprilTag nicht gefunden oder verloren. Beende Ausrichtungs-Schleife.'
                )
                break

            x, y, z = self._last_position

            h_angle = math.atan2(x, z)
            v_angle = math.atan2(y, z)

            # Prüfe, ob die Person im Bild zentriert ist
            if abs(h_angle) < _DEAD_ZONE and abs(v_angle) < _DEAD_ZONE:
                self.get_logger().info('Person erfolgreich im Kamerabild zentriert!')
                break

            # Zielposition berechnen
            target = list(NOD_POSITION)
            target[0] += h_angle * _SCALE_H   # joint_1: Basis horizontal
            target[4] += v_angle * _SCALE_V   # joint_5: Handgelenk vertikal

            self.get_logger().info(
                f'Nachregeln: h={math.degrees(h_angle):.1f}°, v={math.degrees(v_angle):.1f}° '
                f'→ Δjoint_1={h_angle * _SCALE_H:+.3f} rad, Δjoint_5={v_angle * _SCALE_V:+.3f} rad'
            )

            self.move_arm_to(target, duration=int(_STEP_DURATION_S))
            time.sleep(_STEP_DURATION_S)

        # Kurze Beruhigungszeit vor Gestenstart
        time.sleep(0.5)



    # ------------------------------------------------------------------
    # Gesture subscription & Overrides
    # ------------------------------------------------------------------

    def nod(self):
        """Richtet den Arm automatisch zur Person aus und führt dann das Nicken aus."""
        self.get_logger().info('Vorbereitung für Geste "nod": Person suchen & ausrichten...')
        self.orient_to_person()
        self.get_logger().info('Führe Geste aus: "nod"')
        super().nod()

    def shake(self):
        """Richtet den Arm automatisch zur Person aus und führt dann das Schütteln aus."""
        self.get_logger().info('Vorbereitung für Geste "shake": Person suchen & ausrichten...')
        self.orient_to_person()
        self.get_logger().info('Führe Geste aus: "shake"')
        super().shake()

    def _gesture_callback(self, msg: String) -> None:
        """
        Empfängt einen Gesten-Namen auf /arm/gesture.

        Startet die Ausführung in einem Background-Thread damit der
        ROS-Spin-Loop nicht blockiert wird (orient_to_person() schläft).
        """
        name = msg.data.strip().lower()
        gesture_fn = self._gestures.get(name)

        if gesture_fn is None:
            known = ', '.join(self._gestures.keys())
            self.get_logger().warn(f'Unknown gesture: "{name}". Known gestures: [{known}]')
            return

        thread = threading.Thread(
            target=gesture_fn,
            daemon=True
        )
        thread.start()