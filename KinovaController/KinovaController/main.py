import rclpy
import time

from .arm_mover import KinovaMover
from .positions import HOME_POSITION, NOD_POSITION


def main(args=None):
    rclpy.init(args=args)
    node = KinovaMover()

    node.get_logger().info('Warte 2 Sekunden, bis das System bereit ist...')
    time.sleep(2.0)

    node.get_logger().info('Fahre zu HOME_POSITION...')
    node.move_arm_to(HOME_POSITION, 5)
    time.sleep(5.0)

    # 1. AprilTag suchen & Kamera darauf ausrichten
    node.get_logger().info('1. Suchen und Ausrichten zum AprilTag...')
    node.orient_to_person()
    time.sleep(1.0)

    # 2. Per IK-Solver zum AprilTag hinbewegen
    node.get_logger().info('2. Per IK-Solver direkt zum AprilTag fahren...')
    node.move_to_tag_ik(duration=10)
    node.get_logger().info('Test abgeschlossen.')
    time.sleep(3.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
