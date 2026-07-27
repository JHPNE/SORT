import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

from topic_handler.TopicList import TopicList, TopicSpec
from topic_handler.TopicHandlerSubscriber import TopicHandlerSubscriber

from vision_module.AprilTagDetector import AprilTagDetector
from vision_module.CameraInstrinsics import get_intrinsics, intrinsics_from_camera_info

TAG_SIZE_M = 0.055

def _topic_name(spec) -> str:
    """TopicSpec -> topic string. Adjust if your TopicSpec names the field
    something else; this just tries the usual suspects."""
    for attr in ("name", "topic", "topic_name", "path"):
        value = getattr(spec, attr, None)
        if isinstance(value, str):
            return value
    return str(spec)

class CameraViewer(Node):
    def __init__(self, use_camera_info: bool = True):
        super().__init__("camera_viewer")

        self.bridge = CvBridge()
        self.topics = TopicList()

        self.frames: dict[str, "cv2.typing.MatLike"] = {}
        self._subs = []
        self._info_subs = []
        self._have_instrinsics: dict[str, bool] = {}

        cam_topics = {
            'k4a_rgb': self.topics.camera.k4a_rgb,
            'realsense_color': self.topics.camera.realsense_color,
            'secondary_color': self.topics.camera.secondary_color,
        }

        self.detectors: dict[str, AprilTagDetector] = {}

        for name in cam_topics:
            det = AprilTagDetector(tag_family="tag36h11", tag_size_m=TAG_SIZE_M)
            try:
                K, D = get_intrinsics(name)
                det.set_camera_info(K, D)
                self.get_logger().info(f"[{name}] using preset intrinsics")
            except KeyError:
                self.get_logger().warn(f"[{name}] no intrinsics preset, no pose")
            self.detectors[name] = det
            self._have_intrinsics[name] = False

        for name, spec in cam_topics.items():
            handler = TopicHandlerSubscriber(
                node=self,
                topic_spec=spec,
                callback=self._make_callback(name),
                qos=10,
            )

            self._subs.append(handler)

            if use_camera_info:
                info_topic = _topic_name(spec).rsplit("/", 1)[0] + "camera/info"
                self._info_subs.append(self.create_subscription(
                    CameraInfo, info_topic,
                    self._make_info_callback(name), 10))


    def _make_info_callback(self, cam_name: str):
        def callback(msg: CameraInfo):
            if self._have_intrinsics[cam_name]:
                return
            K, D = intrinsics_from_camera_info(msg)
            self.detectors[cam_name].set_camera_info(K, D)
            self._have_intrinsics[cam_name] = True
            self.get_logger().info(
                f"[{cam_name}] intrinsics from camera_info: "
                f"fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} "
                f"cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}")
        return callback

    def _make_callback(self, cam_name: str):
        def callback(msg: Image):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except CvBridgeError as e:
                self.get_logger().error(f"cv bridge error on {cam_name}: {e}")
                return
 
            detector = self.detectors[cam_name]
            detections = detector.detect(cv_image, estimate_pose=True,
                                         max_reprojection_error_px=4.0)
 
            if detections:
                for det in detections:
                    if det.has_pose:
                        x, y, z = det.position_m
                        self.get_logger().info(
                            f"[{cam_name}] tag {det.tag_id} "
                            f"xyz=({x:+.3f}, {y:+.3f}, {z:+.3f}) m "
                            f"d={det.distance_m:.3f} m "
                            f"rms={det.reprojection_error:.2f} px")
                    else:
                        self.get_logger().info(
                            f"[{cam_name}] tag {det.tag_id} (no pose, "
                            f"missing intrinsics or tag size)")
            else:
                self.get_logger().info(
                    f"[{cam_name}] searching for tags, none found yet",
                    throttle_duration_sec=2.0)
 
        return callback
    
    def _render(self):
        for name, frame in self.frames.items():
            cv2.imshow(name, frame)
        cv2.waitKey(1)
 
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()
        
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()