"""JSON transport between the detector node and the world-space node.

One schema, two payloads:
  encode / decode              per-camera detections (/vision/tag_detections/*)
  encode_fused / decode_fused  fused world state     (/vision/tags)

Floats round-trip through json at full double precision, so this is lossless.
"""
import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from vision_module.MultiViewTagFuser import TagObservation

SCHEMA_VERSION = 1


class SchemaMismatch(ValueError):
    pass


# ------------------------------------------------------- per-camera packets

@dataclass
class DetectionPacket:
    """One camera's view at one instant, decoded."""
    camera: str
    frame_id: str
    stamp: float                       # seconds, from the source Image header
    K: Optional[np.ndarray]            # (3, 3) or None if not yet calibrated
    D: Optional[np.ndarray]            # (n, 1)
    width: int
    height: int
    detections: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def has_intrinsics(self) -> bool:
        return self.K is not None


def encode(camera: str, frame_id: str, stamp: float, width: int, height: int,
           detections: Sequence,
           K: Optional[np.ndarray] = None,
           D: Optional[np.ndarray] = None) -> str:
    """Serialize TagDetection objects (from AprilTagDetector)."""
    items = []
    for d in detections:
        item: Dict[str, Any] = {
            "id": int(d.tag_id),
            # flattened x0,y0,...,x3,y3 in cv2.aruco corner order
            "corners": [float(v) for v in
                        np.asarray(d.corners, dtype=np.float64).reshape(8)],
            "err": (float(d.reprojection_error)
                    if d.reprojection_error is not None else None),
        }
        if d.has_pose:
            item["t"] = [float(v) for v in d.tvec.reshape(3)]
            item["R"] = [float(v) for v in
                         np.asarray(d.rotation_matrix,
                                    dtype=np.float64).reshape(9)]
        items.append(item)

    payload = {
        "v": SCHEMA_VERSION,
        "camera": camera,
        "frame_id": frame_id,
        "stamp": float(stamp),
        "width": int(width),
        "height": int(height),
        "K": ([float(v) for v in np.asarray(K, np.float64).reshape(9)]
              if K is not None else None),
        "D": ([float(v) for v in np.asarray(D, np.float64).reshape(-1)]
              if D is not None else None),
        "detections": items,
    }
    return json.dumps(payload, separators=(",", ":"))


def decode(text: str) -> DetectionPacket:
    """Parse a payload. Raises SchemaMismatch on a version or shape problem."""
    try:
        p = json.loads(text)
    except json.JSONDecodeError as e:
        raise SchemaMismatch(f"payload is not valid JSON: {e}") from e

    if p.get("v") != SCHEMA_VERSION:
        raise SchemaMismatch(
            f"schema v{p.get('v')} but this node speaks v{SCHEMA_VERSION}. "
            f"Rebuild and restart both nodes.")

    for d in p.get("detections", []):
        if len(d.get("corners", [])) != 8:
            raise SchemaMismatch(
                f"tag {d.get('id')} has {len(d.get('corners', []))} corner "
                f"values, expected 8")

    return DetectionPacket(
        camera=p["camera"],
        frame_id=p.get("frame_id", ""),
        stamp=float(p["stamp"]),
        K=(np.asarray(p["K"], np.float64).reshape(3, 3) if p.get("K") else None),
        D=(np.asarray(p["D"], np.float64).reshape(-1, 1) if p.get("D") else None),
        width=int(p.get("width", 0)),
        height=int(p.get("height", 0)),
        detections=p.get("detections", []))


def to_observations(packet: DetectionPacket) -> List[TagObservation]:
    """DetectionPacket -> [TagObservation] ready for MultiViewTagFuser."""
    out = []
    for d in packet.detections:
        cam_T_tag = None
        if "t" in d and "R" in d:
            cam_T_tag = np.eye(4)
            cam_T_tag[:3, :3] = np.asarray(d["R"], np.float64).reshape(3, 3)
            cam_T_tag[:3, 3] = np.asarray(d["t"], np.float64).reshape(3)
        out.append(TagObservation(
            camera=packet.camera,
            tag_id=int(d["id"]),
            corners=np.asarray(d["corners"], np.float64).reshape(4, 2),
            stamp=packet.stamp,
            cam_T_tag=cam_T_tag,
            reprojection_error=d.get("err")))
    return out


# ------------------------------------------------------------ fused packets

@dataclass
class FusedPacket:
    """All fused tags at one instant, in one frame."""
    frame_id: str
    stamp: float
    tags: List[Dict[str, Any]] = field(default_factory=list)


def encode_fused(frame_id: str, stamp: float, fused_tags: Sequence) -> str:
    """Serialize FusedTag objects for the aggregate /vision/tags topic."""
    items = []
    for f in fused_tags:
        items.append({
            "id": int(f.tag_id),
            "t": [float(v) for v in np.asarray(f.position).reshape(3)],
            "R": [float(v) for v in np.asarray(f.rotation).reshape(9)],
            "method": f.method,
            "cameras": list(f.cameras),
            "fit_rms_m": None if f.fit_rms_m is None else float(f.fit_rms_m),
            "ray_angle_deg": (None if f.max_ray_angle_deg is None
                              else float(f.max_ray_angle_deg)),
        })
    return json.dumps({"v": SCHEMA_VERSION, "frame_id": frame_id,
                       "stamp": float(stamp), "tags": items},
                      separators=(",", ":"))


def decode_fused(text: str) -> FusedPacket:
    try:
        p = json.loads(text)
    except json.JSONDecodeError as e:
        raise SchemaMismatch(f"payload is not valid JSON: {e}") from e
    if p.get("v") != SCHEMA_VERSION:
        raise SchemaMismatch(
            f"schema v{p.get('v')} but this node speaks v{SCHEMA_VERSION}")
    return FusedPacket(frame_id=p.get("frame_id", ""),
                       stamp=float(p["stamp"]),
                       tags=p.get("tags", []))


def pose_matrix(tag: Dict[str, Any]) -> np.ndarray:
    """One entry from FusedPacket.tags -> 4x4 homogeneous transform."""
    T = np.eye(4)
    T[:3, :3] = np.asarray(tag["R"], np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(tag["t"], np.float64).reshape(3)
    return T