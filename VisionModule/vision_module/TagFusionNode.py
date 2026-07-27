"""Fuse per-camera tag detections into 3D poses. Knows nothing about images.

Subscribes /vision/tag_detections/<camera> (std_msgs/String, JSON - see
tag_messages.py), triangulates, publishes PoseStamped on /vision/tag_<id> and
broadcasts TF frames tag_<id>.

Why a hand-rolled sync buffer instead of message_filters
--------------------------------------------------------
ApproximateTimeSynchronizer needs a message from EVERY registered topic before
it fires, so unplugging one camera stalls the whole pipeline. The buffer below
fuses whatever arrived recently and degrades to fewer cameras instead of
blocking. It warns when a camera falls outside the window, so degradation is
never silent.

Extrinsics:
  use_tf=True   look up reference_frame -> each packet's frame_id via tf2.
                Correct choice on the Kinova.
  use_tf=False  hardcoded EXTRINSICS, from extrinsic_calibration.py.
"""

from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from vision_module.vision_helper import rotation_matrix_to_quaternion
from vision_module.MultiViewTagFuser import (
    CameraModel, TagObservation, MultiViewTagFuser,
)
from vision_module import tag_messages

CAMERA_NAMES = ["k4a_rgb", "realsense_color", "secondary_color"]
DETECTION_TOPIC_NS = "/vision/tag_detections"

# ref_T_cam, metres. Paste output from extrinsic_calibration.py here.
EXTRINSICS: Dict[str, np.ndarray] = {
    "k4a_rgb": np.eye(4),
    "realsense_color": np.eye(4),
    "secondary_color": np.eye(4),
}


def quat_to_R(x, y, z, w) -> np.ndarray:
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


class TagFusionNode(Node):
    def __init__(self):
        super().__init__("tag_fusion")

        self.declare_parameter("camera_names", CAMERA_NAMES)
        self.declare_parameter("reference_frame", CAMERA_NAMES[0])
        self.declare_parameter("tag_size_m", 0.055)
        self.declare_parameter("use_tf", False)
        self.declare_parameter("max_sync_dt", 0.040)
        self.declare_parameter("max_age", 0.30)

        self.camera_names = list(self.get_parameter("camera_names").value)
        self.ref_frame = self.get_parameter("reference_frame").value
        self.tag_size = float(self.get_parameter("tag_size_m").value)
        self.use_tf = bool(self.get_parameter("use_tf").value)
        self.max_sync_dt = float(self.get_parameter("max_sync_dt").value)
        self.max_age = float(self.get_parameter("max_age").value)

        models = {n: CameraModel(n, np.eye(3), np.zeros((5, 1)),
                                 EXTRINSICS.get(n, np.eye(4)))
                  for n in self.camera_names}
        self.fuser = MultiViewTagFuser(models, self.tag_size)

        # camera -> (stamp, [TagObservation], frame_id)
        self._buf: Dict[str, Tuple[float, List[TagObservation], str]] = {}
        self._got_intrinsics = {n: False for n in self.camera_names}

        self._subs = [
            self.create_subscription(
                String, f"{DETECTION_TOPIC_NS}/{n}", self._make_cb(n), 10)
            for n in self.camera_names
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._pubs: Dict[int, object] = {}

        self.create_timer(1.0 / 30.0, self._tick)
        self.get_logger().info(
            f"fusing {self.camera_names} into '{self.ref_frame}', "
            f"extrinsics from {'tf2' if self.use_tf else 'hardcoded'}, "
            f"schema v{tag_messages.SCHEMA_VERSION}")

        if not self.use_tf and all(
                np.allclose(EXTRINSICS.get(n, np.eye(4)), np.eye(4))
                for n in self.camera_names):
            self.get_logger().warn(
                "all EXTRINSICS are identity, so every camera is assumed to be "
                "in the same place and triangulation is meaningless. Run "
                "extrinsic_calibration.py, or set use_tf:=true")

    # --------------------------------------------------------------- input

    def _make_cb(self, name: str):
        def cb(msg: String):
            try:
                packet = tag_messages.decode(msg.data)
            except tag_messages.SchemaMismatch as e:
                self.get_logger().error(f"[{name}] bad packet: {e}",
                                        throttle_duration_sec=5.0)
                return

            if packet.has_intrinsics and not self._got_intrinsics[name]:
                self.fuser.cameras[name].K = packet.K
                self.fuser.cameras[name].D = (
                    packet.D if packet.D is not None else np.zeros((5, 1)))
                self._got_intrinsics[name] = True
                self.get_logger().info(
                    f"[{name}] intrinsics received, fx={packet.K[0, 0]:.1f}")

            self._buf[name] = (packet.stamp,
                               tag_messages.to_observations(packet),
                               packet.frame_id)
        return cb

    # -------------------------------------------------------------- fusion

    def _refresh_tf(self) -> None:
        for name, (_, _, frame_id) in self._buf.items():
            if not frame_id:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.ref_frame, frame_id, rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(
                    f"no transform {self.ref_frame} <- {frame_id}: {e}",
                    throttle_duration_sec=5.0)
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            T = np.eye(4)
            T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
            T[:3, 3] = [t.x, t.y, t.z]
            self.fuser.set_extrinsics(name, T)

    def _tick(self):
        if not self._buf:
            return
        if self.use_tf:
            self._refresh_tf()

        now = self.get_clock().now().nanoseconds * 1e-9
        newest = max(s for s, _, _ in self._buf.values())
        if now - newest > self.max_age:
            return                              # everything is stale

        observations, skipped = [], []
        for name, (stamp, obs, _) in self._buf.items():
            if newest - stamp > self.max_sync_dt:
                skipped.append(name)
                continue
            observations.extend(obs)

        if skipped:
            self.get_logger().warn(
                f"outside {self.max_sync_dt * 1000:.0f} ms sync window: "
                f"{skipped}", throttle_duration_sec=5.0)
        if not observations:
            return

        for fused in self.fuser.fuse(observations):
            self._publish(fused, newest)
            extra = ""
            if fused.fit_rms_m is not None:
                extra = (f" fit={fused.fit_rms_m * 1000:.1f}mm"
                         f" ang={fused.max_ray_angle_deg:.0f}deg")
            x, y, z = fused.position
            self.get_logger().info(
                f"tag {fused.tag_id} [{fused.method}] "
                f"{','.join(fused.cameras)} "
                f"xyz=({x:+.3f}, {y:+.3f}, {z:+.3f}){extra}",
                throttle_duration_sec=0.5)

    def _publish(self, fused, stamp_sec: float):
        stamp = rclpy.time.Time(seconds=stamp_sec).to_msg()
        qx, qy, qz, qw = rotation_matrix_to_quaternion(fused.rotation)
        px, py, pz = (float(v) for v in fused.position)

        if fused.tag_id not in self._pubs:
            self._pubs[fused.tag_id] = self.create_publisher(
                PoseStamped, f"/vision/tag_{fused.tag_id}", 10)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.ref_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = px, py, pz
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self._pubs[fused.tag_id].publish(msg)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.ref_frame
        tf.child_frame_id = f"tag_{fused.tag_id}"
        tf.transform.translation.x = px
        tf.transform.translation.y = py
        tf.transform.translation.z = pz
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = TagFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()