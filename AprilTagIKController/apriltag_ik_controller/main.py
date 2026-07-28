"""
Main executable entry point for testing AprilTag IK Movement.
"""

import time
import rclpy
from .tag_ik_node import TagIKNode, HOME_POSITION


def main(args=None):
    rclpy.init(args=args)
    node = TagIKNode()

    try:
        node.get_logger().info('Warte 3 Sekunden auf URDF und AprilTag Topics...')
        time.sleep(3.0)

        # 1. Fahre zu HOME_POSITION
        node.get_logger().info('Fahre zu HOME_POSITION...')
        node.send_joint_trajectory(HOME_POSITION, duration=5)
        time.sleep(5.0)

        # 2. Per Pinocchio IK direkt zum AprilTag bewegen
        node.get_logger().info('Starte Hinbewegen zum AprilTag per Pinocchio Inverse Kinematic...')
        success = node.move_to_tag(duration=8, offset_z=0.20, timeout=10.0)

        if success:
            node.get_logger().info('IK-Test ERFOLGREICH abgeschlossen!')
        else:
            node.get_logger().warn('IK-Test konnte nicht abgeschlossen werden.')

        time.sleep(3.0)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
