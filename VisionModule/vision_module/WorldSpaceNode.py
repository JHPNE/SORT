"""Fuse per-camera tag detections into one world space.

Subscribes /vision/tag_detections/<camera>, triangulates across cameras, and
publishes the unified state two ways:

  /vision/tags        one aggregate JSON packet per cycle (TagMessages.py)
  TF frames tag_<id>  so anything - rviz, MoveIt, your own code - can ask tf2
                      "where is tag 5 relative to <any frame>"

Reference frame
---------------
Everything is expressed in `reference_frame`. On the arm, set this to the
robot base (e.g. base_link) and use_tf:=true: the arm-mounted camera's
extrinsics then update every cycle from the arm's own TF tree, so tags stay
correct while the arm moves. With use_tf:=false the hardcoded EXTRINSICS
table is used and the reference is a (static) camera.

Why a hand-rolled sync buffer instead of message_filters
--------------------------------------------------------
ApproximateTimeSynchronizer needs a message from EVERY registered topic
before it fires, so unplugging one camera stalls the whole pipeline. This
buffer fuses whatever arrived inside the sync window and degrades to fewer
cameras instead of blocking - and it warns, so degradation is never silent.
"""
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from vision_module.vision_helper import rotation_matrix_to_quaternion
from vision_module.MultiViewTagFuser import (
    CameraModel, TagObservation, MultiViewTagFuser,
)
from vision_module import TagMessage 
from vision_module.TagRegistry import tag_sizes

CAMERA_NAMES = ["arm_camera", "realsense_color", "secondary_color"]
DETECTION_TOPIC_NS = "/vision/tag_detections"

# ref_T_cam, metres. Paste output from extrinsic_calibration.py here.
# Only used when use_tf:=false.
EXTRINSICS: Dict[str, np.ndarray] = {
    "arm_camera": np.eye(4),
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


class WorldSpaceNode(Node):
    def __init__(self):
        super().__init__("world_space")

        self.declare_parameter("camera_names", CAMERA_NAMES)
        self.declare_parameter("reference_frame", "base_link")
        self.declare_parameter("tag_size_m", 0.055)
        self.declare_parameter("use_tf", True)
        self.declare_parameter("max_sync_dt", 0.040)   # s, cross-camera window
        self.declare_parameter("max_age", 0.30)        # s, staleness cutoff
        # If a driver stamps images with a clock that is not this node's
        # clock (the k4a driver can publish device time), header stamps look
        # permanently stale. false = sync on arrival time: less accurate,
        # but it runs.
        self.declare_parameter("trust_stamps", True)

        self.camera_names = list(self.get_parameter("camera_names").value)
        self.ref_frame = self.get_parameter("reference_frame").value
        self.use_tf = bool(self.get_parameter("use_tf").value)
        self.max_sync_dt = float(self.get_parameter("max_sync_dt").value)
        self.max_age = float(self.get_parameter("max_age").value)
        self.trust_stamps = bool(self.get_parameter("trust_stamps").value)

        tag_size = float(self.get_parameter("tag_size_m").value)
        models = {n: CameraModel(n, np.eye(3), np.zeros((5, 1)),
                                 EXTRINSICS.get(n, np.eye(4)))
                  for n in self.camera_names}
        self.fuser = MultiViewTagFuser(models, tag_size, tag_sizes=tag_sizes())

        # camera -> (stamp, [TagObservation], frame_id)
        self._buf: Dict[str, Tuple[float, List[TagObservation], str]] = {}
        self._got_intrinsics = {n: False for n in self.camera_names}
        self._rx = {n: 0 for n in self.camera_names}
        self._bad = {n: 0 for n in self.camera_names}

        self._subs = [
            self.create_subscription(
                String, f"{DETECTION_TOPIC_NS}/{n}", self._make_cb(n), 10)
            for n in self.camera_names
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.world_pub = self.create_publisher(String, "/vision/tags", 10)

        self.create_timer(1.0 / 30.0, self._tick)
        self.create_timer(2.0, self._status)

        self.get_logger().info(
            f"fusing {self.camera_names} into '{self.ref_frame}', "
            f"extrinsics from {'tf2' if self.use_tf else 'hardcoded table'}")
        self._sanity_check()

    def _sanity_check(self):
        if self.use_tf and self.ref_frame in self.camera_names:
            self.get_logger().error(
                f"reference_frame '{self.ref_frame}' is a CAMERA NAME, not a "
                f"TF frame - every tf2 lookup will fail. Use e.g. "
                f"reference_frame:=base_link (check: ros2 run tf2_tools "
                f"view_frames)")
        if not self.use_tf and all(
                np.allclose(EXTRINSICS.get(n, np.eye(4)), np.eye(4))
                for n in self.camera_names):
            self.get_logger().warn(
                "all EXTRINSICS are identity: every camera is assumed to sit "
                "in the same place, so triangulation is meaningless. Run "
                "extrinsic_calibration.py or set use_tf:=true")

    # --------------------------------------------------------------- input

    def _make_cb(self, name: str):
        def cb(msg: String):
            try:
                packet = TagMessage.decode(msg.data)
            except TagMessage.SchemaMismatch as e:
                self._bad[name] += 1
                self.get_logger().error(f"[{name}] bad packet: {e}",
                                        throttle_duration_sec=5.0)
                return
            self._rx[name] += 1

            if packet.has_intrinsics and not self._got_intrinsics[name]:
                cam = self.fuser.cameras[name]
                cam.K = packet.K
                cam.D = packet.D if packet.D is not None else np.zeros((5, 1))
                self._got_intrinsics[name] = True
                self.get_logger().info(
                    f"[{name}] intrinsics received, fx={packet.K[0, 0]:.1f}")

            stamp = (packet.stamp if self.trust_stamps
                     else self.get_clock().now().nanoseconds * 1e-9)
            self._buf[name] = (stamp,
                               TagMessage.to_observations(packet),
                               packet.frame_id)
        return cb

    # -------------------------------------------------------------- fusion

    def _refresh_tf(self) -> None:
        """Pull ref_T_cam for each camera from tf2. Runs every cycle, so an
        arm-mounted camera stays correct while the arm moves."""
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
            self.get_logger().warn(
                f"no packets on {DETECTION_TOPIC_NS}/<camera> - is the "
                f"detector node running?", throttle_duration_sec=5.0)
            return
        if self.use_tf:
            self._refresh_tf()

        now = self.get_clock().now().nanoseconds * 1e-9
        newest = max(s for s, _, _ in self._buf.values())
        if now - newest > self.max_age:
            self.get_logger().warn(
                f"newest packet is {now - newest:.2f}s old (max_age="
                f"{self.max_age}s). If this offset is large and constant, "
                f"image stamps and this node's clock disagree: try "
                f"trust_stamps:=false and check use_sim_time.",
                throttle_duration_sec=5.0)
            return

        observations, skipped = [], []
        for name, (stamp, obs, _) in self._buf.items():
            if newest - stamp > self.max_sync_dt:
                skipped.append(name)
            else:
                observations.extend(obs)
        if skipped:
            self.get_logger().warn(
                f"outside {self.max_sync_dt * 1000:.0f} ms sync window: "
                f"{skipped}", throttle_duration_sec=5.0)
        if not observations:
            return

        fused_tags = self.fuser.fuse(observations)

        msg = String()
        msg.data = TagMessage.encode_fused(self.ref_frame, newest, fused_tags)
        self.world_pub.publish(msg)

        for fused in fused_tags:
            self._broadcast_tf(fused, newest)
            x, y, z = fused.position
            extra = ("" if fused.fit_rms_m is None else
                     f" fit={fused.fit_rms_m * 1000:.1f}mm"
                     f" ang={fused.max_ray_angle_deg:.0f}deg")
            self.get_logger().info(
                f"tag {fused.tag_id} [{fused.method}] "
                f"{','.join(fused.cameras)} "
                f"xyz=({x:+.3f}, {y:+.3f}, {z:+.3f}){extra}",
                throttle_duration_sec=0.5)

    def _broadcast_tf(self, fused, stamp_sec: float):
        qx, qy, qz, qw = rotation_matrix_to_quaternion(fused.rotation)
        tf = TransformStamped()
        tf.header.stamp = rclpy.time.Time(seconds=stamp_sec).to_msg()
        tf.header.frame_id = self.ref_frame
        tf.child_frame_id = f"tag_{fused.tag_id}"
        (tf.transform.translation.x, tf.transform.translation.y,
         tf.transform.translation.z) = (float(v) for v in fused.position)
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = qx, qy, qz, qw
        self.tf_broadcaster.sendTransform(tf)

    # --------------------------------------------------------- diagnostics

    def _status(self):
        """Periodic state dump - this tells you which stage is stuck."""
        now = self.get_clock().now().nanoseconds * 1e-9
        parts = []
        for n in self.camera_names:
            if self._rx[n] == 0:
                bad = f" ({self._bad[n]} rejected)" if self._bad[n] else ""
                parts.append(f"{n}: NO PACKETS{bad}")
                continue
            stamp, obs, fid = self._buf.get(n, (0.0, [], ""))
            extr = self.fuser.cameras[n].ref_T_cam
            parts.append(
                f"{n}: {self._rx[n]}pkt {len(obs)}tag "
                f"age={now - stamp:+.2f}s "
                f"K={'y' if self._got_intrinsics[n] else 'N'} "
                f"extr={'identity' if np.allclose(extr, np.eye(4)) else 'set'} "
                f"frame='{fid}'")
        self.get_logger().info("STATUS  " + "  |  ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = WorldSpaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()