"""
Dedicated ROS 2 Node for AprilTag IK Movement (Pinocchio-based).
"""

import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from topic_handler.TopicList import TopicList
from .ik_solver import KinovaIKSolver

HOME_POSITION = [
    -0.35875123465126624,
    -1.6124865157634325,
    -0.6201484851258137,
    -0.033775274208821315,
    -0.918114133160457,
    0.004664388289732777
]


class TagIKNode(Node):
    """
    ROS 2 Node that receives AprilTag poses and moves the Kinova Gen3 arm using Pinocchio IK.
    """

    def __init__(self):
        super().__init__('apriltag_ik_node')

        self.topics = TopicList()
        self.ik_solver = KinovaIKSolver()

        self._last_position: tuple[float, float, float] | None = None
        self._last_orientation: tuple[float, float, float, float] | None = None
        self._current_joint_position: list[float] = list(HOME_POSITION)
        self._tag_event = threading.Event()

        # URDF Topic subscription with TRANSIENT_LOCAL durability QoS
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

        # AprilTag pose subscription
        self.create_subscription(
            PoseStamped,
            self.topics.arm.apriltag_pose.name,
            self._tag_callback,
            10
        )

        # Publisher for joint trajectory controller
        self._publisher = self.create_publisher(
            JointTrajectory,
            self.topics.arm.joint_trajectory.name,
            10
        )

        # Spin executor in background thread
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info('AprilTag IK Node gestartet.')

    def destroy_node(self):
        """Clean shutdown of background executor thread."""
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
        """Loads URDF into Pinocchio IK solver."""
        if self.ik_solver.model is None and msg.data:
            if self.ik_solver.load_urdf(msg.data):
                self.get_logger().info('URDF erfolgreich geladen! Pinocchio IK-Solver bereit.')

    def _tag_callback(self, msg: PoseStamped) -> None:
        """Callback for incoming AprilTag 3D pose."""
        p = msg.pose.position
        q = msg.pose.orientation
        self._last_position = (p.x, p.y, p.z)
        self._last_orientation = (q.x, q.y, q.z, q.w)
        self._tag_event.set()

    def send_joint_trajectory(self, joint_positions: list[float], duration: int = 5) -> None:
        """Sends joint target trajectory to robot controller."""
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start.sec = int(duration)
        msg.points.append(point)

        self.get_logger().info(f'Sende Gelenktrajektorie (Dauer: {duration}s) an Roboter...')
        self._publisher.publish(msg)
        self._current_joint_position = list(joint_positions)

    def move_to_tag(self, duration: int = 5, offset_z: float = 0.2, timeout: float = 5.0) -> bool:
        """
        Waits for an AprilTag pose, solves Pinocchio IK for 3D coordinates, and moves arm.

        :param duration: Execution time in seconds.
        :param offset_z: Safety distance in front of tag in meters (default: 0.2m).
        :param timeout: Max wait time for AprilTag message in seconds.
        :return: True if IK movement sent successfully, False otherwise.
        """
        if not self.ik_solver.is_available or self.ik_solver.model is None:
            self.get_logger().error(
                'IK-Solver nicht bereit! URDF noch nicht von /robot_description empfangen.'
            )
            return False

        self._tag_event.clear()
        self.get_logger().info(f'[Tag-IK] Warte auf AprilTag-Position (max {timeout}s)...')
        found = self._tag_event.wait(timeout=timeout)

        if not found or self._last_position is None:
            self.get_logger().warn('[Tag-IK] Kein AprilTag im Sichtfeld empfangen!')
            return False

        x, y, z = self._last_position
        self.get_logger().info(
            f'[Tag-IK] Tag im Kamera-Frame erkannt: x={x:+.3f}m, y={y:+.3f}m, z={z:+.3f}m'
        )

        fk_res = self.ik_solver.get_forward_kinematics(self._current_joint_position)

        if fk_res is not None:
            p_ee, R_ee = fk_res
            # Transform tag position from camera relative vector into robot base frame
            target_cam = np.array([x, y, max(0.05, z - offset_z)])
            target_base = p_ee + R_ee @ target_cam
            target_x, target_y, target_z = float(target_base[0]), float(target_base[1]), float(target_base[2])
            self.get_logger().info(
                f'[Tag-IK] Ziel im Base-Frame: x={target_x:+.3f}m, y={target_y:+.3f}m, z={target_z:+.3f}m'
            )
        else:
            target_x, target_y, target_z = x, y, z

        # Calculate 6 joint angles using Pinocchio Inverse Kinematics
        joint_angles = self.ik_solver.solve_position(
            target_x, target_y, target_z, q_init=self._current_joint_position
        )

        if joint_angles is None:
            self.get_logger().error(f'[Tag-IK] Keine IK-Lösung für Position ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}) gefunden.')
            return False

        self.get_logger().info(f'[Tag-IK] IK-Lösung erfolgreich berechnet! Fahre Ziel an...')
        self.send_joint_trajectory(joint_angles, duration=duration)
        return True
