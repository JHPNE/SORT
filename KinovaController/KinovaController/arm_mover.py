import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

from topic_handler.TopicList import TopicList
from topic_handler.TopicHandlerPublisher import TopicHandlerPublisher

from .positions import HOME_POSITION, SAFE_POSE

class KinovaMover(Node):
    def __init__(self):
        super().__init__('kinova_mover')
        
        self.topics = TopicList()
        self._publisher = TopicHandlerPublisher(
            node=self,
            topic_spec=self.topics.arm.joint_trajectory,
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

    def move_arm_to(self, joint_positions=None, duration=10):
        if joint_positions is None:
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
        point.positions = SAFE_POSE
        point.time_from_start.sec = 5
        
        msg.points.append(point)
        self.get_logger().info('Sende sicheren Bewegungsbefehl an den Kinova Gen3...')
        self._publisher.publish(msg)
        self.get_logger().info('Befehl gesendet!')

    def move_arm_to_home_pose(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        point = JointTrajectoryPoint()
        point.positions = HOME_POSITION 
        point.time_from_start.sec = 5
        
        msg.points.append(point)
        self.get_logger().info('Sende home Bewegungsbefehl an den Kinova Gen3...')
        self._publisher.publish(msg)
        self.get_logger().info('Home-Befehl gesendet!')

    def nod(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # Nod down
        point1 = JointTrajectoryPoint()
        point1.positions = list(HOME_POSITION)
        point1.positions[4] += 0.5
        point1.time_from_start.sec = 1
        msg.points.append(point1)

        # Nod up
        point2 = JointTrajectoryPoint()
        point2.positions = list(HOME_POSITION)
        point2.positions[4] -= 0.3
        point2.time_from_start.sec = 2
        msg.points.append(point2)
        
        # Nod down again
        point3 = JointTrajectoryPoint()
        point3.positions = list(HOME_POSITION)
        point3.positions[4] += 0.5
        point3.time_from_start.sec = 3
        msg.points.append(point3)

        # Back to Home
        point4 = JointTrajectoryPoint()
        point4.positions = list(HOME_POSITION)
        point4.time_from_start.sec = 4
        msg.points.append(point4)

        self.get_logger().info('Sende Nod-Befehl (Nicken) an den Kinova Gen3...')
        self._publisher.publish(msg)
        self.get_logger().info('Nod-Befehl gesendet!')

def main(args=None):
    rclpy.init(args=args)
    node = KinovaMover()
    node.destroy_node()
    rclpy.shutdown()

def test_move_kinova_arm(joint_positions=None, args=None):
    rclpy.init(args=args)
    node = KinovaMover()
    
    node.get_logger().info('Warte 2 Sekunden, bis das System bereit ist...')
    time.sleep(2.0)
    
    node.move_arm_to_home_pose()
    node.get_logger().info('Warte 5 Sekunden, bis Home-Position erreicht ist...')
    time.sleep(5.0)

    # node.move_arm_to(joint_positions)
    
    # time.sleep(10.0)
    
    node.get_logger().info('Führe Nick-Bewegung (Nodding) aus...')
    node.nod()
    time.sleep(5.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # main()
    test_move_kinova_arm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])