import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

from topic_handler.TopicList import TopicList, TopicSpec
from topic_handler.TopicHandlerSubscriber import TopicHandlerSubscriber

from vision_module.AprilTagDetector import AprilTagDetector
from vision_module.CameraInstrinsics import get_intrinsics, intrinsics_from_camera_info
from vision_module import TagMessage

TAG_SIZE_M = 0.055
DETECTION_TOPIC_NS = "/vision/tag_detections"


CAMERAS = ["k4a_rgb", "realsense_color", "secondary_color"]


def _topic_name(spec) -> str:
    """TopicSpec -> topic string. Adjust if your TopicSpec names the field
    something else; this just tries the usual suspects."""
    for attr in ("name", "topic", "topic_name", "path"):
        value = getattr(spec, attr, None)
        if isinstance(value, str):
            return value
    return str(spec)


class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")

        self.declare_parameter("tag_size_m", TAG_SIZE_M)
        self.tag_size = float(self.get_parameter("tag_size_m").value)

        self.bridge = CvBridge()
        self.topics = TopicList()

        self.frames = {}
        self._subs, self._info_subs = [], []
        self._have_instrinsics = {}
        self.detectors = {}
        self.pubs = {}
        self.frame_ids = {}

        cam_topics = {
            'k4a_rgb': self.topics.camera.k4a_rgb,
            'realsense_color': self.topics.camera.realsense_color,
            'secondary_color': self.topics.camera.secondary_color,
        }

        for name, spec in cam_topics.items():
            det = AprilTagDetector("tag36h11", tag_size_m=self.tag_size)
            try:
                K, D = get_intrinsics(name)
                det.set_camera_info(K, D)
            except KeyError:
                self.get_logger().warn(f"[{name}] no instrinsics preset")

            self.detectors[name] = det
            self._have_instrinsics[name] = False
            self.frame_ids[name] = f"{name}_optical_frame"

            self.pubs[name] = self.create_publisher(
                String, f"{DETECTION_TOPIC_NS}/{name}", 10)

            self._subs.append(TopicHandlerSubscriber(
                node=self, topic_spec=spec,
                callback=self._make_callback(name), qos=10))
 
            info_topic = _topic_name(spec).rsplit("/", 1)[0] + "/camera_info"
            self._info_subs.append(self.create_subscription(
                CameraInfo, info_topic, self._make_info_cb(name), 10))

        self.get_logger().info(
            f"publishing schema v{TagMessage.SCHEMA_VERSION} on "
            f"{DETECTION_TOPIC_NS}/<camera>")
    
    def _make_info_cb(self, name: str):
        def cb(msg: CameraInfo):
            # Adopt the driver's own frame_id rather than guessing the name.
            if msg.header.frame_id:
                self.frame_ids[name] = msg.header.frame_id
            if self._have_instrinsics[name]:
                return
            K, D = intrinsics_from_camera_info(msg)
            self.detectors[name].set_camera_info(K, D)
            self._have_instrinsics[name] = True
            self.get_logger().info(
                f"[{name}] intrinsics from camera_info, frame "
                f"'{self.frame_ids[name]}'")
        return cb
    
    def _make_callback(self, name: str):
        def cb(msg: Image):
            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except CvBridgeError as e:
                self.get_logger().error(f"cv bridge error on {name}: {e}")
                return
 
            detector = self.detectors[name]
            detections = detector.detect(img, estimate_pose=True,
                                         max_reprojection_error_px=4.0)
 
            h, w = img.shape[:2]
            # Reuse the IMAGE stamp, not now(). The fusion node syncs on this,
            # so it must mean "when light hit the sensor". Stamping at publish
            # time would fold detection latency into the sync window.
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
 
            out = String()
            out.data = TagMessage.encode(
                camera=name,
                frame_id=msg.header.frame_id or self.frame_ids[name],
                stamp=stamp, width=w, height=h,
                detections=detections,
                K=detector.camera_matrix, D=detector.dist_coeffs)
            self.pubs[name].publish(out)
 
            if detections:
                self.get_logger().info(
                    f"[{name}] tags {sorted(d.tag_id for d in detections)}",
                    throttle_duration_sec=1.0)
            else:
                self.get_logger().info(f"[{name}] no tags",
                                       throttle_duration_sec=2.0)
 
        return cb

    def _render(self):
        for name, frame in self.frames.items():
            cv2.imshow(name, frame)
            cv2.waitKey(1)
 
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
 
 
if __name__ == "__main__":
    main()
