import rclpy
from rclpy.executors import ExternalShutdownException
import time

from .arm_mover import KinovaMover
from .positions import HOME_POSITION, NOD_POSITION


def main(args=None):
    rclpy.init(args=args)
    node = KinovaMover()

    try:
        time.sleep(1.0)
        node.get_logger().info('Fahre zu HOME_POSITION...')
        node.move_arm_to(HOME_POSITION, 5)
        time.sleep(5.0)

        node.get_logger().info(
            'KinovaMover bereit! Lauscht aktiv auf Befehle auf /arm/gesture (nod, shake, search) '
            'und AprilTags auf /vision/apriltag_pose. (Beenden mit Strg+C)'
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
