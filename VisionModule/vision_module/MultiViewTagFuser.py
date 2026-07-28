"""Multi-camera AprilTag fusion. Pure numpy/OpenCV, no ROS.

The pipeline, per tag id:

  1. Undistort each camera's 4 corner observations into normalized image
     coordinates. Triangulation assumes a linear pinhole; skipping this step
     silently bakes lens distortion into the result.
  2. Triangulate each of the 4 corners independently across all cameras that
     saw the tag, using iterative-reweighted DLT.
  3. Fit the known tag geometry to those 4 triangulated points (Kabsch). This
     gives position AND orientation, plus a fit residual.

What the fit residual does and does not tell you. Measured on a synthetic
3-camera rig, position error came out around 30x the residual, so it is a
proportional indicator, not an alarm. A small extrinsic error displaces all
four corners coherently, so they still form a near-perfect square: 20 mm of
extrinsic error gave 12.8 mm of position error but only 0.2 mm of residual.
Treat a large residual as proof something is badly wrong (swapped frames,
optical-vs-body frame mixup, fusing frames from different instants). Do NOT
treat a small residual as proof the answer is accurate. The only real check on
extrinsics is ground truth: put the tag at a measured position and compare.

Fallbacks:
  - only one camera sees the tag        -> that camera's PnP pose
  - viewing rays too parallel (small
    baseline, so depth is unobservable) -> weighted average of PnP poses

Frame convention: everything is in the OpenCV/ROS *optical* frame - x right,
y down, z forward along the optical axis. If you pull extrinsics from TF, use
the `..._optical_frame` links, NOT the camera body frames (which are x forward,
z up). Mixing these up gives you a 90 degree error that looks like a bug in
the triangulation.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


# --------------------------------------------------------------- containers

@dataclass
class CameraModel:
    """One camera: intrinsics plus its pose in the reference frame."""
    name: str
    K: np.ndarray                      # (3, 3)
    D: np.ndarray                      # (5, 1) or (n, 1)
    ref_T_cam: np.ndarray = field(     # (4, 4) camera pose IN reference frame
        default_factory=lambda: np.eye(4))

    def __post_init__(self):
        self.K = np.asarray(self.K, dtype=np.float64).reshape(3, 3)
        self.D = np.asarray(self.D, dtype=np.float64).reshape(-1, 1)
        self.ref_T_cam = np.asarray(self.ref_T_cam, dtype=np.float64).reshape(4, 4)

    @property
    def cam_T_ref(self) -> np.ndarray:
        R = self.ref_T_cam[:3, :3]
        t = self.ref_T_cam[:3, 3]
        out = np.eye(4)
        out[:3, :3] = R.T
        out[:3, 3] = -R.T @ t
        return out

    @property
    def P_normalized(self) -> np.ndarray:
        """(3, 4) projection for points already in normalized coords."""
        return self.cam_T_ref[:3, :4]

    @property
    def center_ref(self) -> np.ndarray:
        return self.ref_T_cam[:3, 3].copy()

    def normalize(self, pts_px: np.ndarray) -> np.ndarray:
        """(N, 2) pixels -> (N, 2) undistorted normalized image coords."""
        pts = np.asarray(pts_px, dtype=np.float64).reshape(-1, 1, 2)
        out = cv2.undistortPoints(pts, self.K, self.D)
        return out.reshape(-1, 2)


@dataclass
class TagObservation:
    """One tag seen by one camera at one instant."""
    camera: str
    tag_id: int
    corners: np.ndarray                       # (4, 2) pixels
    stamp: float = 0.0
    cam_T_tag: Optional[np.ndarray] = None    # (4, 4) from single-cam PnP
    reprojection_error: Optional[float] = None


@dataclass
class FusedTag:
    tag_id: int
    position: np.ndarray                      # (3,) in reference frame
    rotation: np.ndarray                      # (3, 3)
    method: str                               # triangulated | pnp_avg | pnp_single
    cameras: List[str] = field(default_factory=list)
    fit_rms_m: Optional[float] = None         # Kabsch residual
    edge_error_m: Optional[float] = None      # |measured edge - known edge|
    max_ray_angle_deg: Optional[float] = None # triangulation conditioning
    corners_3d: Optional[np.ndarray] = None   # (4, 3) triangulated

    @property
    def n_cameras(self) -> int:
        return len(self.cameras)

    @property
    def ref_T_tag(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.position
        return T

    def __repr__(self) -> str:
        x, y, z = self.position
        rms = "n/a" if self.fit_rms_m is None else f"{self.fit_rms_m * 1000:.1f}mm"
        return (f"<FusedTag {self.tag_id} [{self.method}, {self.n_cameras}cam] "
                f"xyz=({x:+.3f}, {y:+.3f}, {z:+.3f}) fit={rms}>")


# ------------------------------------------------------------------- math

def triangulate_nview(pts_norm: Sequence[np.ndarray],
                      projections: Sequence[np.ndarray],
                      iterations: int = 10) -> Optional[np.ndarray]:
    """Triangulate one 3D point from N normalized observations.

    Iteratively reweighted DLT (Hartley-Sturm). Plain DLT minimizes an
    algebraic error that is biased toward whichever camera happens to be
    closest; the reweighting drives it toward the geometric optimum, which
    matters once your cameras are at meaningfully different ranges.
    """
    n = len(projections)
    if n < 2:
        return None

    w = np.ones(n, dtype=np.float64)
    X = None
    for it in range(iterations + 1):
        A = np.empty((2 * n, 4), dtype=np.float64)
        for i, (p, P) in enumerate(zip(pts_norm, projections)):
            A[2 * i]     = (p[0] * P[2] - P[0]) / w[i]
            A[2 * i + 1] = (p[1] * P[2] - P[1]) / w[i]

        _, _, Vt = np.linalg.svd(A)
        Xh = Vt[-1]
        if abs(Xh[3]) < 1e-12:
            return None                     # point at infinity: rays parallel
        X = Xh[:3] / Xh[3]

        if it == iterations:
            break
        Xh1 = np.append(X, 1.0)
        w_new = np.array([P[2] @ Xh1 for P in projections])
        if np.any(np.abs(w_new) < 1e-9):
            break                            # point on a camera's focal plane
        if np.allclose(w_new, w, rtol=1e-6):
            break
        w = w_new

    # Reject points behind any camera.
    Xh1 = np.append(X, 1.0)
    for P in projections:
        if P[2] @ Xh1 <= 0:
            return None
    return X


def kabsch(src: np.ndarray, dst: np.ndarray
           ) -> Tuple[np.ndarray, np.ndarray, float]:
    """Rigid transform taking src onto dst. Returns (R, t, rms_residual)."""
    src = np.asarray(src, dtype=np.float64).reshape(-1, 3)
    dst = np.asarray(dst, dtype=np.float64).reshape(-1, 3)
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = cd - R @ cs
    resid = (src @ R.T + t) - dst
    rms = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1))))
    return R, t, rms


def average_rotations(rotations: Sequence[np.ndarray],
                      weights: Optional[Sequence[float]] = None) -> np.ndarray:
    """Chordal L2 mean of rotation matrices, via SVD projection to SO(3)."""
    if weights is None:
        weights = np.ones(len(rotations))
    w = np.asarray(weights, dtype=np.float64)
    w = w / w.sum()
    M = sum(wi * np.asarray(R, dtype=np.float64) for wi, R in zip(w, rotations))
    U, _, Vt = np.linalg.svd(M)
    d = np.sign(np.linalg.det(U @ Vt))
    return U @ np.diag([1.0, 1.0, d]) @ Vt


def max_ray_angle_deg(point: np.ndarray,
                      centers: Sequence[np.ndarray]) -> float:
    """Largest angle between viewing rays. Near zero means no usable baseline
    and depth is essentially unobservable no matter how many cameras you add."""
    dirs = []
    for c in centers:
        d = np.asarray(point, dtype=np.float64) - np.asarray(c, dtype=np.float64)
        n = np.linalg.norm(d)
        if n > 1e-9:
            dirs.append(d / n)
    best = 0.0
    for i in range(len(dirs)):
        for j in range(i + 1, len(dirs)):
            c = float(np.clip(dirs[i] @ dirs[j], -1.0, 1.0))
            best = max(best, np.degrees(np.arccos(c)))
    return best


def tag_object_points(tag_size_m: float) -> np.ndarray:
    """(4, 3) corners in the tag frame, matching cv2.aruco corner order."""
    h = float(tag_size_m) / 2.0
    return np.array([[-h,  h, 0.0],
                     [ h,  h, 0.0],
                     [ h, -h, 0.0],
                     [-h, -h, 0.0]], dtype=np.float64)


# ------------------------------------------------------------------ fuser

class MultiViewTagFuser:
    def __init__(self,
                 cameras: Dict[str, CameraModel],
                 tag_sizes: Optional[Dict[int, float]] = None,
                 min_ray_angle_deg: float = 3.0,
                 max_fit_rms_m: float = 0.005):
        """
        min_ray_angle_deg  below this the cameras are effectively co-located,
                           so we fall back to averaging PnP poses instead of
                           triangulating an ill-conditioned depth.
        max_fit_rms_m      Kabsch residual above this rejects the triangulated
                           result and falls back to pose averaging. This is a
                           GROSS error trap only - see the module docstring.
                           Expect roughly 30x this value in position error, so
                           the 5 mm default trips at around 150 mm of error.
                           It will not catch a few mm of extrinsic drift.
        """
        self.cameras = dict(cameras)
        self.min_ray_angle_deg = float(min_ray_angle_deg)
        self.max_fit_rms_m = float(max_fit_rms_m)
        self.tag_sizes = dict(tag_sizes or {})

    def set_extrinsics(self, name: str, ref_T_cam: np.ndarray) -> None:
        self.cameras[name].ref_T_cam = np.asarray(
            ref_T_cam, dtype=np.float64).reshape(4, 4)

    def _size(self, tag_id: int) -> float:
        return self.tag_sizes.get(tag_id, self.tag_size_m)

    # ---------------------------------------------------------------- api

    def fuse(self, observations: Sequence[TagObservation]) -> List[FusedTag]:
        """Group observations by tag id and fuse each group."""
        by_tag: Dict[int, List[TagObservation]] = {}
        for obs in observations:
            if obs.camera not in self.cameras:
                continue
            by_tag.setdefault(obs.tag_id, []).append(obs)

        out: List[FusedTag] = []
        for tag_id, group in sorted(by_tag.items()):
            fused = self.fuse_one(tag_id, group)
            if fused is not None:
                out.append(fused)
        return out

    def fuse_one(self, tag_id: int,
                 group: Sequence[TagObservation]) -> Optional[FusedTag]:
        # Keep one observation per camera (the best, if duplicated).
        per_cam: Dict[str, TagObservation] = {}
        obj = tag_object_points(self._size(tag_id))

        for obs in group:
            prev = per_cam.get(obs.camera)
            if prev is None:
                per_cam[obs.camera] = obs
            else:
                a = obs.reprojection_error
                b = prev.reprojection_error
                if a is not None and (b is None or a < b):
                    per_cam[obs.camera] = obs

        names = sorted(per_cam)
        if not names:
            return None
        if len(names) == 1:
            return self._from_single(tag_id, per_cam[names[0]])

        cams = [self.cameras[n] for n in names]
        Ps = [c.P_normalized for c in cams]
        centers = [c.center_ref for c in cams]

        # Corner k, triangulated across every camera that saw the tag.
        corners_3d = np.zeros((4, 3), dtype=np.float64)
        for k in range(4):
            pts = [self.cameras[n].normalize(per_cam[n].corners[k:k + 1])[0]
                   for n in names]
            X = triangulate_nview(pts, Ps)
            if X is None:
                return self._pnp_average(tag_id, per_cam, names)
            corners_3d[k] = X

        centroid = corners_3d.mean(axis=0)
        angle = max_ray_angle_deg(centroid, centers)
        if angle < self.min_ray_angle_deg:
            fused = self._pnp_average(tag_id, per_cam, names)
            if fused is not None:
                fused.max_ray_angle_deg = angle
            return fused

        R, t, rms = kabsch(obj, corners_3d)

        # Scale check: the triangulated square should have the edge length
        # you measured. This is insensitive in the same way the fit residual
        # is - 20 mm of extrinsic error moved it by only 0.2 mm - so it flags
        # a badly wrong baseline, not a slightly wrong one.
        edges = [float(np.linalg.norm(corners_3d[i] - corners_3d[(i + 1) % 4]))
                 for i in range(4)]
        edge_err = float(abs(np.mean(edges) - self._size(tag_id)))

        if rms > self.max_fit_rms_m:
            fused = self._pnp_average(tag_id, per_cam, names)
            if fused is not None:
                fused.fit_rms_m = rms
                fused.edge_error_m = edge_err
                fused.max_ray_angle_deg = angle
                fused.method = "pnp_avg(bad_fit)"
            return fused

        return FusedTag(tag_id=tag_id, position=t, rotation=R,
                        method="triangulated", cameras=names,
                        fit_rms_m=rms, edge_error_m=edge_err,
                        max_ray_angle_deg=angle, corners_3d=corners_3d)

    # ----------------------------------------------------------- fallbacks

    def _from_single(self, tag_id: int,
                     obs: TagObservation) -> Optional[FusedTag]:
        if obs.cam_T_tag is None:
            return None
        T = self.cameras[obs.camera].ref_T_cam @ obs.cam_T_tag
        return FusedTag(tag_id=tag_id, position=T[:3, 3], rotation=T[:3, :3],
                        method="pnp_single", cameras=[obs.camera])

    def _pnp_average(self, tag_id: int,
                     per_cam: Dict[str, TagObservation],
                     names: Sequence[str]) -> Optional[FusedTag]:
        """Weighted mean of each camera's PnP pose, in the reference frame.

        Weight by 1/(reproj_error * range^2): reprojection error is the
        per-view fit quality, and monocular depth error grows with the square
        of range, so a close camera deserves far more say than a far one.
        """
        Ts, weights = [], []
        for n in names:
            obs = per_cam[n]
            if obs.cam_T_tag is None:
                continue
            T = self.cameras[n].ref_T_cam @ obs.cam_T_tag
            rng = float(np.linalg.norm(obs.cam_T_tag[:3, 3]))
            err = obs.reprojection_error if obs.reprojection_error else 0.5
            weights.append(1.0 / (max(err, 1e-3) * max(rng, 1e-3) ** 2))
            Ts.append(T)

        if not Ts:
            return None
        w = np.asarray(weights, dtype=np.float64)
        w /= w.sum()
        pos = sum(wi * T[:3, 3] for wi, T in zip(w, Ts))
        rot = average_rotations([T[:3, :3] for T in Ts], w)
        return FusedTag(tag_id=tag_id, position=pos, rotation=rot,
                        method="pnp_avg", cameras=list(names))