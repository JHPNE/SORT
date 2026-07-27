import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

from topic_handler.TopicList import TopicList
from topic_handler.TopicHandlerPublisher import TopicHandlerPublisher

class KinovaMover(Node):
    def __init__(self):
        super().__init__('kinova_mover')
        
        self.topics = TopicList()
        arm_topics = {
            'joint_trajectory': self.topics.arm.joint_trajectory
        }

        self._publisher = TopicHandlerPublisher(
            node=self,
            topic_spec=arm_topics.joint_trajectory.spec,
            qos=10
        )

        # self.publisher_ = self.create_publisher(
        #     JointTrajectory, 
        #     '/joint_trajectory_controller/joint_trajectory', 
        #     10
        # )
        
        # self.get_logger().info('Warte 2 Sekunden, bis das System bereit ist...')
        # time.sleep(2.0)
        # self.move_arm_to_safe_pose()

    def move_arm_to(self, joint_positions, duration=10):
        if joint_positions is Null:
            joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        if len(joint_positions) != 6:
            self.get_logger().error(f'Invalid number of joints. Expected: 6, Received: {len(joint_positions)}')
            return

        msg = JointTrajectory()
        
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = duration
        
        msg.points.append(point)
        
        self.get_logger().info(f'Sending msg to Kinova Gen3: {msg}')
        self._publisher.publish(msg)

    def move_arm_to_safe_pose(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        point = JointTrajectoryPoint()
        # CHANGE POSITION HERE
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        point.time_from_start.sec = 5
        
        msg.points.append(point)
        self.get_logger().info('Sende sicheren Bewegungsbefehl an den Kinova Gen3...')
        self._publisher.publish(msg)
        self.get_logger().info('Befehl gesendet!')

def main(args=None):
    rclpy.init(args=args)
    node = KinovaMover()
    node.destroy_node()
    rclpy.shutdown()

def test_move_kinova_arm(joint_positions):
    rclpy.init(args=args)
    node = KinovaMover()
    
    node.get_logger().info('Warte 2 Sekunden, bis das System bereit ist...')
    time.sleep(2.0)
    node.move_arm_to([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # main()
    test_move_kinova_arm