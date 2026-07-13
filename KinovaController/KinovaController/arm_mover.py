import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class KinovaMover(Node):
    def __init__(self):
        super().__init__('kinova_mover')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.get_logger().info('Warte 2 Sekunden, bis das System bereit ist...')
        time.sleep(2.0)
        self.move_arm_to_safe_pose()

    def move_arm_to_safe_pose(self):
        msg = JointTrajectory()
        
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        point = JointTrajectoryPoint()
        
        # CAREFUL: Ersetze diese Werte am besten mit deinen aktuellen Gelenkwinkeln
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

        # CAREFUL: Geschwindigkeit angeben        
        point.time_from_start.sec = 5
        
        msg.points.append(point)
        
        self.get_logger().info('Sende sicheren Bewegungsbefehl an den Kinova Gen3...')
        self.publisher_.publish(msg)
        self.get_logger().info('Befehl gesendet!')

def main(args=None):
    rclpy.init(args=args)
    node = KinovaMover()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()