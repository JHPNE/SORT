import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import time

from topic_handler.TopicList import TopicList

from .gestures import ArmGestures
from .positions import HOME_POSITION


class KinovaMover(Node, ArmGestures):
    def __init__(self):
        super().__init__('kinova_mover')

        self.topics = TopicList()

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
    # Gesture subscription
    # ------------------------------------------------------------------

    def _gesture_callback(self, msg: String):
        """Receives a gesture name on /arm/gesture and dispatches to the matching method."""
        name = msg.data.strip().lower()
        gesture_fn = self._gestures.get(name)

        if gesture_fn is None:
            known = ', '.join(self._gestures.keys())
            self.get_logger().warn(f'Unknown gesture: "{name}". Known gestures: [{known}]')
            return

        self.get_logger().info(f'Executing gesture: "{name}"')
        gesture_fn()