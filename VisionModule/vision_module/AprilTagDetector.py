import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List, Sequence, Dict

from vision_module.vision_helper import (
    rotation_matrix_to_quaternion,
    normalize_family,
    get_dictionary,
    HAS_ARUCO,
)


@dataclass
class TagDetection:
    tag_id: int
    corners: np.ndarray                       # (4, 2) float32, image pixels
    backend: str = "opencv"
    rvec: Optional[np.ndarray] = None         # (3, 1) Rodrigues, camera <- tag
    tvec: Optional[np.ndarray] = None         # (3, 1) metres, camera frame
    rotation_matrix: Optional[np.ndarray] = None
    reprojection_error: Optional[float] = None  # pixels, RMS

    @property
    def center_px(self) -> Tuple[float, float]:
        c = self.corners.mean(axis=0)
        return (float(c[0]), float(c[1]))

    @property
    def perimeter_px(self) -> float:
        c = self.corners
        return float(sum(np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)))

    @property
    def has_pose(self) -> bool:
        return self.tvec is not None

    @property
    def position_m(self) -> Optional[Tuple[float, float, float]]:
        if self.tvec is None:
            return None
        t = self.tvec.reshape(3)
        return (float(t[0]), float(t[1]), float(t[2]))

    @property
    def distance_m(self) -> Optional[float]:
        return None if self.tvec is None else float(np.linalg.norm(self.tvec))

    @property
    def quaternion(self) -> Optional[Tuple[float, float, float, float]]:
        if self.rotation_matrix is None:
            return None
        return rotation_matrix_to_quaternion(self.rotation_matrix)

    @property
    def euler_deg(self) -> Optional[Tuple[float, float, float]]:
        if self.rotation_matrix is None:
            return None
        a = np.asarray(cv2.RQDecomp3x3(self.rotation_matrix)[0],
                       dtype=np.float64).reshape(-1)
        return (float(a[0]), float(a[1]), float(a[2]))

    @property
    def transform_matrix(self) -> Optional[np.ndarray]:
        """4x4 homogeneous camera_T_tag."""
        if self.rotation_matrix is None or self.tvec is None:
            return None
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = self.rotation_matrix
        T[:3, 3] = self.tvec.reshape(3)
        return T

    def __repr__(self) -> str:
        if not self.has_pose:
            return f"<Tag {self.tag_id} no pose>"
        x, y, z = self.position_m
        err = "n/a" if self.reprojection_error is None \
            else f"{self.reprojection_error:.2f}"
        return (f"<Tag {self.tag_id} xyz=({x:+.3f}, {y:+.3f}, {z:+.3f}) m "
                f"err={err} px>")


class AprilTagDetector:
    """AprilTag detection through OpenCV's ArUco module.

    tag_size_m is the side of the tag's BLACK SQUARE, including the 1-cell
    black border but NOT the white quiet zone. Measure it with calipers; a 1%
    error in this number is a 1% error in every distance you report.
    """

    def __init__(
        self,
        tag_family: str = "tag36h11",
        tag_size_m: Optional[float] = None,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        refine_corners: bool = True,
        min_marker_perimeter_rate: float = 0.02,
    ):
        self.family = normalize_family(tag_family)
        self.backend = "opencv"
        self.tag_size_m = float(tag_size_m) if tag_size_m is not None else None
        self.tag_sizes: Dict[int, float] = {}

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        if camera_matrix is not None:
            self.set_camera_info(camera_matrix, dist_coeffs)

        self._aruco_dict = get_dictionary(self.family)
        self.params = self._build_aruco_params(refine_corners,
                                               min_marker_perimeter_rate)
        self._aruco = (cv2.aruco.ArucoDetector(self._aruco_dict, self.params)
                       if HAS_ARUCO else None)

        self._obj_points: Optional[np.ndarray] = None
        self._rebuild_object_points()

    # ---------------------------------------------------------------- setup

    @staticmethod
    def _build_aruco_params(refine_corners: bool, min_perimeter_rate: float):
        p = (cv2.aruco.DetectorParameters() if HAS_ARUCO
             else cv2.aruco.DetectorParameters_create())
        p.adaptiveThreshWinSizeMin = 3
        p.adaptiveThreshWinSizeMax = 43
        p.adaptiveThreshWinSizeStep = 10
        p.adaptiveThreshConstant = 7
        # default 0.03 silently drops small / distant tags. Going much below
        # 0.02 buys you false positives on textured backgrounds.
        p.minMarkerPerimeterRate = float(min_perimeter_rate)
        p.maxMarkerPerimeterRate = 4.0
        p.polygonalApproxAccuracyRate = 0.05
        p.minCornerDistanceRate = 0.05
        p.minDistanceToBorder = 3
        p.markerBorderBits = 1           # correct for the AprilTag families
        p.maxErroneousBitsInBorderRate = 0.35
        p.errorCorrectionRate = 0.6
        p.perspectiveRemovePixelPerCell = 8
        p.perspectiveRemoveIgnoredMarginPerCell = 0.13
        p.detectInvertedMarker = False
        if refine_corners:
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            p.cornerRefinementWinSize = 5
            p.cornerRefinementMaxIterations = 50
            p.cornerRefinementMinAccuracy = 0.01
        else:
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        return p

    def set_camera_info(self, camera_matrix, dist_coeffs=None) -> None:
        K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        D = (np.zeros((5, 1), dtype=np.float64) if dist_coeffs is None
             else np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1))
        self.camera_matrix, self.dist_coeffs = K, D

    def set_tag_size(self, tag_size_m: float) -> None:
        self.tag_size_m = float(tag_size_m)
        self._rebuild_object_points()

    def set_tag_sizes(self, sizes: Dict[int, float]) -> None:
        self.tag_sizes = dict(sizes)

    def _obj_points_for(self, tag_id: int) -> np.ndarray:
        size = self.tag_sizes.get(tag_id, self.tag_size_m)
        h = size / 2.0
        return np.array([[-h,  h, 0], [ h,  h, 0],
                        [ h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    @property
    def can_estimate_pose(self) -> bool:
        return (self.camera_matrix is not None
                and self.tag_size_m is not None
                and self._obj_points is not None)

    def _rebuild_object_points(self) -> None:
        if self.tag_size_m is None:
            self._obj_points = None
            return
        h = float(self.tag_size_m) / 2.0
        # Order must match cv2.aruco corner order (TL, TR, BR, BL) and is what
        # SOLVEPNP_IPPE_SQUARE expects. Do not reorder.
        self._obj_points = np.array([[-h,  h, 0.0],
                                     [ h,  h, 0.0],
                                     [ h, -h, 0.0],
                                     [-h, -h, 0.0]], dtype=np.float64)

    @staticmethod
    def _to_gray(image: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        if gray.dtype != np.uint8:
            gray = cv2.convertScaleAbs(gray)
        return np.ascontiguousarray(gray)

    # ------------------------------------------------------------ detection

    def detect_raw(self, image: np.ndarray):
        """(corners, ids, rejected), exactly like cv2.aruco.detectMarkers."""
        if image is None or image.size == 0:
            return [], None, []
        gray = self._to_gray(image)
        if HAS_ARUCO:
            return self._aruco.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self._aruco_dict,
                                       parameters=self.params)

    def detect(
        self,
        image: np.ndarray,
        estimate_pose: bool = True,
        max_reprojection_error_px: Optional[float] = None,
        min_perimeter_px: float = 0.0,
    ) -> List[TagDetection]:
        """Detect tags and, when intrinsics allow, solve 6-DoF pose.

        Returns a list of TagDetection. Use detect_raw() if you want the plain
        (corners, ids, rejected) tuple instead.

        OpenCV's ArUco backend exposes no hamming distance or decision margin,
        so the only confidence signals you get are reprojection error and tag
        size in pixels. For anything safety critical, filter on both.
        """
        corners, ids, _ = self.detect_raw(image)
        detections: List[TagDetection] = []
        if ids is None or len(ids) == 0:
            return detections

        for c, tag_id in zip(corners, ids.flatten()):
            det = TagDetection(
                tag_id=int(tag_id),
                corners=np.asarray(c, dtype=np.float32).reshape(4, 2),
            )
            if det.perimeter_px < min_perimeter_px:
                continue
            detections.append(det)

        if estimate_pose and self.can_estimate_pose:
            for det in detections:
                self._solve_pose(det)
            if max_reprojection_error_px is not None:
                detections = [
                    d for d in detections
                    if d.reprojection_error is not None
                    and d.reprojection_error <= max_reprojection_error_px
                ]
        return detections

    def _solve_pose(self, det: TagDetection) -> None:
        """Planar PnP, two-fold ambiguity resolved by reprojection error."""
        img_pts = det.corners.astype(np.float64).reshape(4, 1, 2)
        obj = self._obj_points_for(det.tag_id)
        try:
            n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                obj, img_pts, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if n == 0 or not len(rvecs):
                return
            # errs comes back as (n, 1, 1). Iterating it yields (1, 1) arrays,
            # and float() on those is a TypeError under numpy >= 2. Flatten.
            best = 0
            if errs is not None:
                e = np.asarray(errs, dtype=np.float64).reshape(-1)
                if e.size == len(rvecs):
                    best = int(np.argmin(e))
            det.rvec, det.tvec = rvecs[best], tvecs[best]
        except cv2.error:
            ok, rvec, tvec = cv2.solvePnP(
                obj, img_pts, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return
            det.rvec, det.tvec = rvec, tvec

        det.rotation_matrix = cv2.Rodrigues(det.rvec)[0]
        proj, _ = cv2.projectPoints(obj, det.rvec, det.tvec,
                                    self.camera_matrix, self.dist_coeffs)
        det.reprojection_error = float(np.sqrt(np.mean(
            np.sum((proj.reshape(4, 2) - det.corners) ** 2, axis=1))))

    # ----------------------------------------------------------- rendering

    def draw(self, image: np.ndarray, detections: Sequence[TagDetection],
             draw_axes: bool = True, draw_text: bool = True) -> np.ndarray:
        for det in detections:
            pts = det.corners.astype(int)
            cv2.polylines(image, [pts], True, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(image, (int(pts[0][0]), int(pts[0][1])), 5,
                       (255, 0, 255), -1)  # corner 0

            cx, cy = det.center_px
            label = f"id {det.tag_id}"
            if det.has_pose:
                label += f"  {det.distance_m:.3f} m"
            cv2.putText(image, label, (int(cx) - 40, int(cy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

            if draw_text and det.has_pose:
                x, y, z = det.position_m
                cv2.putText(image, f"x{x:+.3f} y{y:+.3f} z{z:+.3f}",
                            (int(cx) - 70, int(cy) + 22), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (0, 255, 255), 1, cv2.LINE_AA)
                if det.reprojection_error is not None:
                    cv2.putText(image, f"rms {det.reprojection_error:.2f} px",
                                (int(cx) - 70, int(cy) + 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                (200, 200, 200), 1, cv2.LINE_AA)

            if draw_axes and det.has_pose and self.camera_matrix is not None:
                cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs,
                                  det.rvec, det.tvec, self.tag_size_m * 0.75, 2)
        return image