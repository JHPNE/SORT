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


class KinovaMover(Node, TrajectoryExecutor, IKMovement, VisualTracker, ArmGestures):
    """
    Main ROS 2 Node for controlling the Kinova Gen3 arm.
    """

    def __init__(self):
        super().__init__('kinova_mover')

        self.topics = TopicList()

        # Initialize internal visual tracker states
        self._init_visual_tracker()

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

        # Publishers
        self._publisher = self.create_publisher(
            JointTrajectory,
            self.topics.arm.joint_trajectory.name,
            10
        )

        # Register gesture mapping
        self._gestures: dict[str, callable] = {
            'nod':    self.nod,
            'shake':  self.shake,
            'search': self.search,
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

    def _gesture_callback(self, msg: String) -> None:
        """Handles incoming gesture string on /arm/gesture."""
        name = msg.data.strip().lower()
        gesture_fn = self._gestures.get(name)

        if gesture_fn is None:
            known = ', '.join(self._gestures.keys())
            self.get_logger().warn(f'Unknown gesture: "{name}". Known gestures: [{known}]')
            return

        thread = threading.Thread(target=gesture_fn, daemon=True)
        thread.start()