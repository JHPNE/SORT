"""
Trajectory Execution Mixin for Kinova Gen3 Arm.

Handles building and publishing JointTrajectory messages to the ROS 2 trajectory controller topic.
"""

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .positions import HOME_POSITION

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


class TrajectoryExecutor:
    """Mixin providing trajectory execution capabilities for KinovaMover."""

    def move_sequence(self, steps: list[tuple[list[float], float | int]]) -> None:
        """
        Send a sequence of joint target waypoints with timing to the robot.

        :param steps: List of tuples (joint_positions_list, time_from_start_sec)
        """
        if not hasattr(self, '_publisher') or self._publisher is None:
            self.get_logger().error('Trajectory publisher is not initialized!')
            return

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        for positions, t in steps:
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            msg.points.append(point)

        self.get_logger().info(f'Sending sequence of {len(steps)} point(s) to Kinova Gen3...')
        self._publisher.publish(msg)

    def move_arm_to(self, joint_positions: list[float] | None = None, duration: float | int = 10) -> None:
        """Convenience wrapper to move the arm to a single joint position target."""
        if joint_positions is None:
            joint_positions = HOME_POSITION

        self.move_sequence([(joint_positions, duration)])


