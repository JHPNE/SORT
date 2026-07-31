"""
Zone

contains ZoneSpec = Basic Info over the Zone (name, Id, corners of the APRILTAG, positional arguments)
ZoneGeometry maps a geometry based on apriltag information/ZoneSpec
ZoneMap tells us which zone the cube is in? from live TagWorld state. 

"""
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Sequence

import numpy as np
from vision_module import TagRegistry as tr


@dataclass(frozen=True)
class ZoneSpec:
    """A zone as configuration: identity plus tolerances. No geometry."""
    name: str
    zone_id: int
    corner_tag_ids: Tuple[int, ...]
    inset_m: float = 0.015
    height_below_m: float = 0.03
    height_above_m: float = 0.08


ZONES: List[ZoneSpec] = [
    ZoneSpec(name=tr.zone_name(tr.ZONE_PAPIER), zone_id=tr.ZONE_PAPIER,
             corner_tag_ids=tr.corners_of(tr.ZONE_PAPIER)),
    ZoneSpec(name=tr.zone_name(tr.ZONE_RESTMUELL), zone_id=tr.ZONE_RESTMUELL,
             corner_tag_ids=tr.corners_of(tr.ZONE_RESTMUELL)),
    ZoneSpec(name=tr.zone_name(tr.ZONE_PLASTIK), zone_id=tr.ZONE_PLASTIK,
             corner_tag_ids=tr.corners_of(tr.ZONE_PLASTIK)),
]

CUBE_IDS: List[int] = tr.ids_in("trash")

# A cube pose farther than this from EVERY zone plane cannot be a cube
# sitting anywhere near the table - it is a broken detection (wrong tag
# size in the detector registry, a diverged PnP solve, a false positive).
# Such cubes are reported as 'implausible', never as 'unassigned'.
MAX_PLANE_DIST_M = 0.30


@dataclass
class ZoneGeometry:
    """Live geometry for one zone, valid for the instant it was built."""
    spec: ZoneSpec
    centroid: np.ndarray          # (3,) in the reference frame
    normal: np.ndarray            # (3,) unit, pointing away from the sheet
    u: np.ndarray                 # (3,) unit, in-plane basis
    v: np.ndarray                 # (3,) unit, in-plane basis
    corners_3d: np.ndarray        # (4, 3) ordered around the perimeter
    corners_2d: np.ndarray        # (4, 2) same, in (u, v)
    inset_2d: np.ndarray          # (4, 2) shrunk by inset_m
    visible_corners: int          # how many were actually detected
    plane_rms_m: float            # fit residual - large means bad detections

    def project(self, point: np.ndarray) -> Tuple[np.ndarray, float]:
        """(uv_in_plane, signed_height_above_plane) for a 3D point."""
        d = np.asarray(point, dtype=np.float64) - self.centroid
        return np.array([d @ self.u, d @ self.v]), float(d @ self.normal)

    def contains(self, point: np.ndarray) -> bool:
        uv, height = self.project(point)
        if not (-self.spec.height_below_m <= height
                <= self.spec.height_above_m):
            return False
        return _point_in_convex_polygon(uv, self.inset_2d)

    def pose(self) -> np.ndarray:
        """4x4 frame at the sheet centre: +z out of the sheet, +x along the
        first edge. Feed this to GraspPose like any other tag pose."""
        T = np.eye(4)
        x = self.u
        z = self.normal
        y = np.cross(z, x)
        T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
        T[:3, 3] = self.centroid
        return T

    @property
    def size_m(self) -> Tuple[float, float]:
        """Rough extent, for sanity-checking against a tape measure."""
        return (float(self.corners_2d[:, 0].ptp()),
                float(self.corners_2d[:, 1].ptp()))


def _point_in_convex_polygon(p: np.ndarray, verts: np.ndarray) -> bool:
    """All edge cross products share a sign -> inside."""
    sign = 0
    n = len(verts)
    for i in range(n):
        a, b = verts[i], verts[(i + 1) % n]
        cross = ((b[0] - a[0]) * (p[1] - a[1])
                 - (b[1] - a[1]) * (p[0] - a[0]))
        if abs(cross) < 1e-12:
            continue                      # exactly on the edge: not decisive
        s = 1 if cross > 0 else -1
        if sign == 0:
            sign = s
        elif s != sign:
            return False
    return True


def _complete_rectangle(pts: np.ndarray) -> np.ndarray:
    """Reconstruct a 4th corner from 3, assuming a parallelogram.

    The missing corner is opposite the vertex whose two edges are closest to
    perpendicular - that is the right-angle corner of the sheet, and the
    fourth point is a + c - b.
    """
    best_i, best_cos = 0, 2.0
    for i in range(3):
        b, a, c = pts[i], pts[(i + 1) % 3], pts[(i + 2) % 3]
        v1, v2 = a - b, c - b
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        cos = abs(float(v1 @ v2) / (n1 * n2))
        if cos < best_cos:
            best_cos, best_i = cos, i
    b = pts[best_i]
    a = pts[(best_i + 1) % 3]
    c = pts[(best_i + 2) % 3]
    return np.vstack([pts, a + c - b])


def build_geometry(spec: ZoneSpec, world) -> Optional[ZoneGeometry]:
    """Fit a zone from whatever of its corner tags are currently visible."""
    positions, normals = [], []
    for tid in spec.corner_tag_ids:
        p = world.position(tid)
        if p is None:
            continue
        positions.append(np.asarray(p, dtype=np.float64))
        pose = world.pose(tid)
        if pose is not None:
            normals.append(np.asarray(pose)[:3, 2])

    visible = len(positions)
    if visible < 3:
        return None

    pts = np.stack(positions)
    if visible == 3:
        pts = _complete_rectangle(pts)

    centroid = pts.mean(axis=0)
    centred = pts - centroid
    # SVD of the centred points: the smallest singular direction is the plane
    # normal, the largest two span the plane.
    _, sv, vt = np.linalg.svd(centred, full_matrices=False)
    u, v, normal = vt[0], vt[1], vt[2]
    plane_rms = float(sv[2] / np.sqrt(len(pts))) if len(sv) > 2 else 0.0

    # SVD gives an arbitrary normal sign. Orient it away from the sheet using
    # the corner tags' own +z axes - a majority vote, so one flipped tag
    # (planar PnP is two-fold ambiguous) cannot invert the zone.
    if normals:
        votes = sum(1 for z in normals if float(z @ normal) > 0.0)
        if votes * 2 < len(normals):
            normal = -normal
    if np.cross(u, v) @ normal < 0.0:
        v = -v                              # keep (u, v, normal) right-handed

    basis = np.stack([u, v])
    corners_2d = centred @ basis.T

    order = np.argsort(np.arctan2(corners_2d[:, 1], corners_2d[:, 0]))
    corners_2d = corners_2d[order]
    corners_3d = pts[order]

    # Approximate inward offset: pull each vertex toward the centroid. Exact
    # only for a square, close enough for a sheet of paper.
    radii = np.linalg.norm(corners_2d, axis=1, keepdims=True)
    scale = np.clip(1.0 - spec.inset_m / np.maximum(radii, 1e-6), 0.0, 1.0)
    inset_2d = corners_2d * scale

    return ZoneGeometry(spec, centroid, normal, u, v, corners_3d, corners_2d,
                        inset_2d, visible, plane_rms)


class ZoneMap:
    """Answers 'which zone is this cube in?' from live TagWorld state."""

    def __init__(self, world, zones: Sequence[ZoneSpec] = ZONES,
                 max_plane_dist_m: float = MAX_PLANE_DIST_M):
        self.world = world
        self.zones: Dict[int, ZoneSpec] = {z.zone_id: z for z in zones}
        self.max_plane_dist_m = max_plane_dist_m

    def geometries(self) -> Dict[int, ZoneGeometry]:
        """Fit every zone once. Reuse the result within a single tick rather
        than refitting per cube - the SVD is cheap but not free."""
        out = {}
        for zid, spec in self.zones.items():
            g = build_geometry(spec, self.world)
            if g is not None:
                out[zid] = g
        return out

    def visible_zones(self) -> List[ZoneSpec]:
        return [self.zones[zid] for zid in self.geometries()]

    def min_plane_dist(self, point: np.ndarray,
                       geos: Dict[int, ZoneGeometry]) -> Optional[float]:
        """Distance from the point to the nearest zone plane, or None if no
        zone geometry is available."""
        if not geos:
            return None
        p = np.asarray(point, dtype=np.float64)
        return min(abs(g.project(p)[1]) for g in geos.values())

    def is_plausible(self, point: np.ndarray,
                     geos: Dict[int, ZoneGeometry]) -> bool:
        """A detection is plausible if it lies within max_plane_dist_m of at
        least one zone plane. With no zones visible we cannot judge, so we
        give it the benefit of the doubt."""
        d = self.min_plane_dist(point, geos)
        return d is None or d <= self.max_plane_dist_m

    def zone_of(self, cube_tag_id: int,
                geos: Optional[Dict[int, ZoneGeometry]] = None
                ) -> Optional[ZoneSpec]:
        cube_pos = self.world.position(cube_tag_id)
        if cube_pos is None:
            return None
        for g in (geos if geos is not None else self.geometries()).values():
            if g.contains(np.asarray(cube_pos, dtype=np.float64)):
                return g.spec
        return None

    def sort_state(self, cube_ids: Sequence[int]) -> Dict[str, List[int]]:
        """zone name -> cubes in it, plus:
          'unassigned'  - visible, plausible pose, just not on any sheet
          'implausible' - visible, but the pose is nowhere near the table
                          (broken detection: wrong tag size, diverged PnP,
                          false positive). Never trust these for planning.
        """
        geos = self.geometries()
        state: Dict[str, List[int]] = {z.name: [] for z in self.zones.values()}
        state["unassigned"] = []
        state["implausible"] = []
        for cid in cube_ids:
            pos = self.world.position(cid)
            if pos is None:
                continue                    # not visible: report nothing
            p = np.asarray(pos, dtype=np.float64)
            zone = self.zone_of(cid, geos)
            if zone is not None:
                state[zone.name].append(cid)
            elif self.is_plausible(p, geos):
                state["unassigned"].append(cid)
            else:
                state["implausible"].append(cid)
        return state

    def drop_pose(self, zone_id: int,
                  hover_m: float = 0.10) -> Optional[np.ndarray]:
        """4x4 pose hover_m above the sheet centre, in the reference frame.

        Orientation is the sheet's own frame (+z out of the paper), so it
        needs the same tag-to-gripper conversion as any cube pose - see
        GraspPose.tool_pose_from_tag.
        """
        spec = self.zones.get(zone_id)
        if spec is None:
            return None
        g = build_geometry(spec, self.world)
        if g is None:
            return None
        T = g.pose()
        T[:3, 3] = g.centroid + g.normal * hover_m
        return T


    def describe(self, cube_ids: Sequence[int]) -> List[str]:
        """Human-readable dump for tag_reader."""
        lines: List[str] = []
        geos = self.geometries()

        for zid, spec in self.zones.items():
            g = geos.get(zid)
            if g is None:
                seen = [t for t in spec.corner_tag_ids
                        if self.world.position(t) is not None]
                lines.append(
                    f"  {spec.name:<14} UNAVAILABLE - {len(seen)}/4 corners "
                    f"visible {seen} (need 3)")
                continue
            w, h = g.size_m
            lines.append(
                f"  {spec.name:<14} {w:.3f} x {h:.3f} m, "
                f"{g.visible_corners}/4 corners, "
                f"plane rms {g.plane_rms_m * 1000:.1f} mm")

        for cid in cube_ids:
            pos = self.world.position(cid)
            if pos is None:
                continue
            p = np.asarray(pos, dtype=np.float64)

            if not self.is_plausible(p, geos):
                d = self.min_plane_dist(p, geos)
                lines.append(
                    f"  cube {cid}: IMPLAUSIBLE pose xyz=({p[0]:+.3f}, "
                    f"{p[1]:+.3f}, {p[2]:+.3f}) m - {d:.2f} m from the "
                    f"nearest zone plane (limit {self.max_plane_dist_m:.2f}). "
                    f"Check the tag size registered for tag {cid} and that "
                    f"the detector is running the current TagRegistry.")
                continue

            for zid, g in geos.items():
                uv, height = g.project(p)
                inside = g.contains(p)
                lines.append(
                    f"  {g.spec.name:<14} <- cube {cid}: "
                    f"uv=({uv[0]:+.3f}, {uv[1]:+.3f}) "
                    f"height={height:+.3f} m  "
                    f"{'IN' if inside else 'out'}")
        return lines