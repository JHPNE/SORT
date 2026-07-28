import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

from topic_handler.TopicList import TopicList
from topic_handler.TopicHandlerPublisher import TopicHandlerPublisher

from .positions import HOME_POSITION, NOD_POSITION

class KinovaMover(Node):
    def __init__(self):
        super().__init__('kinova_mover')
        
        self.topics = TopicList()
        # self._publisher = TopicHandlerPublisher(
        #     node=self,
        #     topic_spec=self.topics.arm.joint_trajectory,
        #     qos=10
        # )

        self._publisher = self.create_publisher(
            JointTrajectory, 
            # '/joint_trajectory_controller/joint_trajectory',
            self.topics.arm.joint_trajectory.name, 
            10
        )

        self.get_logger().info('Warte 2 Sekunden, bis das System bereit ist...')
        time.sleep(2.0)
        self.move_arm_to(HOME_POSITION, duration=5)
        time.sleep(5.0)

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
            joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.move_sequence([(joint_positions, duration)])

    # def move_arm_to_home_pose(self):
    #     msg = JointTrajectory()
    #     msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
    #     point = JointTrajectoryPoint()
    #     point.positions = HOME_POSITION 
    #     point.time_from_start.sec = 5
        
    #     msg.points.append(point)
    #     self.get_logger().info('Sende home Bewegungsbefehl an den Kinova Gen3...')
    #     self._publisher.publish(msg)
    #     self.get_logger().info('Home-Befehl gesendet!')

    def nod(self):
        nod_down = list(NOD_POSITION);  nod_down[4] += 0.5   # Wrist pitch down
        nod_up   = list(NOD_POSITION);  nod_up[4]   -= 0.5   # Wrist pitch up
        nod_end  = list(HOME_POSITION); nod_end[4]  += 0.5   # Return near home

        self.move_sequence([
            (nod_down, 2),
            (nod_up,   4),
            (nod_end,  6),
        ])

def main(args=None):
    print("test")
    rclpy.init(args=args)
    node = KinovaMover()
    
    node.get_logger().info('Warte 2 Sekunden, bis das System bereit ist...')
    time.sleep(2.0)
    
    node.get_logger().info('move to home position')
    node.move_arm_to(HOME_POSITION, 5)
    node.get_logger().info('Warte 5 Sekunden, bis Home-Position erreicht ist...')
    time.sleep(5.0)
    
    node.get_logger().info('move to nod position')
    node.move_arm_to(NOD_POSITION, 5.0)
    time.sleep(5.0)

    node.get_logger().info('do nodding')
    node.nod()
    time.sleep(5.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()