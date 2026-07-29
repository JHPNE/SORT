"""
KinovaMover Main Node for Kinova Gen3 Arm.

Composes modular capabilities:
- TrajectoryExecutor: Joint trajectory publishing and movement execution
- IKMovement: Inverse Kinematics solving & Cartesian target movement
- VisualTracker: AprilTag tracking & search sweeps
- ArmGestures: High-level gestures (nod, shake, search)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from topic_handler.TopicList import TopicList

from .positions import HOME_POSITION
from .trajectory_executor import TrajectoryExecutor
from .ik_solver import KinovaIKSolver, IKMovement
from .visual_tracker import VisualTracker
from .gestures import ArmGestures
from .gripper_controller import GripperController


class KinovaMover(Node, TrajectoryExecutor, IKMovement, VisualTracker, ArmGestures, GripperController):
    """
    Main ROS 2 Node for controlling the Kinova Gen3 arm.
    """

    def __init__(self):
        super().__init__('kinova_mover')

        self.topics = TopicList()

        # Publishers
        self._publisher = self.create_publisher(
            JointTrajectory,
            self.topics.arm.joint_trajectory.name,
            10
        )

        # Initialize internal visual tracker and gripper states
        self._init_visual_tracker()
        self._init_gripper()

        # Inverse Kinematics Solver (Pinocchio)
        self.ik_solver = KinovaIKSolver()

        # Subscriptions
        # URDF on /robot_description is published with TRANSIENT_LOCAL durability by robot_state_publisher
        qos_urdf = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.create_subscription(
            String,
            '/robot_description',
            self._robot_description_callback,
            qos_urdf
        )

        self.create_subscription(
            String,
            '/vision/tags',
            self._worldspace_tags_callback,
            10
        )

        self.create_subscription(
            String,
            self.topics.arm.gesture.name,
            self._gesture_callback,
            10
        )

        # Register gesture mapping
        self._gestures: dict[str, callable] = {
            'nod':    self.nod,
            'shake':  self.shake,
            'search': self.search,
            'home':   self.home,
        }

        # Background ROS 2 executor thread for real-time subscription callbacks
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info('KinovaMover erfolgreich initialisiert.')

    def destroy_node(self):
        """Cleanly shutdown executor before node destruction."""
        if hasattr(self, '_executor') and self._executor:
            try:
                self._executor.shutdown()
            except Exception:
                pass
        if hasattr(self, '_spin_thread') and self._spin_thread and self._spin_thread.is_alive():
            try:
                self._spin_thread.join(timeout=2.0)
            except Exception:
                pass
        super().destroy_node()

    def _robot_description_callback(self, msg: String) -> None:
        """Loads URDF from /robot_description into Pinocchio IK Solver."""
        if self.ik_solver.model is None and msg.data:
            if self.ik_solver.load_urdf(msg.data):
                self.get_logger().info('URDF erfolgreich aus /robot_description geladen (Pinocchio IK bereit).')

    # ------------------------------------------------------------------
    # Gesture Overrides (auto-orient before gesture)
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

    def search(self):
        """Führt den langsamen Umschau-Sweep (Suchbewegung) aus."""
        self.get_logger().info('Führe Geste aus: "search" (Umschau-Sweep)...')
        super().search()

    def goto_arm_camera_tag(self):
        """Bewegt den Arm per IK zum AprilTag (Einzelkamera am Arm)."""
        self.get_logger().info('Führe IK-Anfahrt zum AprilTag der Arm-Kamera aus...')
        self.move_to_arm_camera_tag_ik(duration=20, offset_z=0.25)

    goto_tag = goto_arm_camera_tag

    def _gesture_callback(self, msg: String) -> None:
        """Handles incoming gesture string on /arm/gesture."""
        name = msg.data.strip().lower()

        # 1. WorldSpace Anfahrt: "goto_worldspace_tag_3", "goto_world_3", "goto_tag_3"
        if name.startswith("goto_worldspace_tag_") or name.startswith("goto_world_") or name.startswith("goto_tag_"):
            try:
                tag_id = int(name.rsplit("_", 1)[1])
                self.get_logger().info(f'Empfangen: Anfahrt zu Tag {tag_id} im WorldSpace...')
                thread = threading.Thread(target=self.move_to_worldspace_tag_ik, kwargs={"tag_id": tag_id}, daemon=True)
                thread.start()
                return
            except ValueError:
                pass

        # 2. WorldSpace Ausrichtung: "orient_worldspace_tag_3", "orient_world_3"
        if name.startswith("orient_worldspace_tag_") or name.startswith("orient_world_"):
            try:
                tag_id = int(name.rsplit("_", 1)[1])
                self.get_logger().info(f'Empfangen: Kamera-Ausrichtung zu Tag {tag_id} im WorldSpace...')
                thread = threading.Thread(target=self.orient_to_worldspace_tag, kwargs={"tag_id": tag_id}, daemon=True)
                thread.start()
                return
            except ValueError:
                pass

        # 3. Arm-Kamera Anfahrt: "goto_arm_camera_tag", "goto_arm_tag", "goto_tag"
        if name in ("goto_arm_camera_tag", "goto_arm_tag", "goto_tag", "tag"):
            thread = threading.Thread(target=self.goto_arm_camera_tag, daemon=True)
            thread.start()
            return

        # 4. Arm-Kamera Ausrichtung: "orient_arm_camera_tag", "orient_arm_tag"
        if name in ("orient_arm_camera_tag", "orient_arm_tag"):
            thread = threading.Thread(target=self.orient_to_arm_camera_tag, daemon=True)
            thread.start()
            return

        # 5. Pick & Place Kommandos: "pick_tag_3", "pick_3", "open_gripper", "close_gripper"
        if name.startswith("pick_tag_") or name.startswith("pick_"):
            try:
                tag_id = int(name.rsplit("_", 1)[1])
                self.get_logger().info(f'Empfangen: Pick (Greif-Sequenz) für AprilTag {tag_id}...')
                thread = threading.Thread(target=self.pick_tag, kwargs={"tag_id": tag_id}, daemon=True)
                thread.start()
                return
            except ValueError:
                pass

        if name in ("open_gripper", "open"):
            thread = threading.Thread(target=self.open_gripper, daemon=True)
            thread.start()
            return

        if name in ("close_gripper", "close"):
            thread = threading.Thread(target=self.close_gripper, daemon=True)
            thread.start()
            return

        gesture_fn = self._gestures.get(name)

        if gesture_fn is None:
            known = ', '.join(list(self._gestures.keys()) + ['goto_arm_camera_tag', 'goto_worldspace_tag_<ID>', 'pick_tag_<ID>', 'open_gripper', 'close_gripper'])
            self.get_logger().warn(f'Unknown gesture: "{name}". Known gestures: [{known}]')
            return

        thread = threading.Thread(target=gesture_fn, daemon=True)
        thread.start()