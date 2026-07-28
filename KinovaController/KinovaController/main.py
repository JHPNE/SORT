import rclpy
import time

from .arm_mover import KinovaMover
from .positions import HOME_POSITION, NOD_POSITION


def main(args=None):
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
    time.sleep(6.0)

    # node.get_logger().info('do shaking')
    # node.shake()
    # time.sleep(6.0)

    # node.get_logger().info('Testing Cartesian IK Move (x=0.4m, y=0.1m, z=0.3m)...')
    # node.move_to_cartesian_position(x=0.4, y=0.1, z=0.3, duration=5)
    # time.sleep(6.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
