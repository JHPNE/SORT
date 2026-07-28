"""
Trajectory Execution Mixin for Kinova Gen3.

Handles publishing JointTrajectory messages on the ROS 2 trajectory topic.
"""

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .positions import HOME_POSITION


class TrajectoryExecutor:
    """Mixin that handles JointTrajectory execution for KinovaMover."""

    def move_sequence(self, steps: list[tuple[list[float], int]]):
        """
        Send a sequence of joint targets with timing to the robot.
        
        :param steps: List of tuples (joint_positions_list, time_from_start_sec)
        """
        if not hasattr(self, '_publisher') or self._publisher is None:
            self.get_logger().error('Trajectory publisher is not initialized!')
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

    def move_arm_to(self, joint_positions=None, duration: int = 10):
        """Convenience wrapper to move to a single joint position."""
        if joint_positions is None:
            joint_positions = HOME_POSITION

        self.move_sequence([(joint_positions, duration)])
