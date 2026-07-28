from dataclasses import dataclass
from typing import Dict, List, Optional
 
import numpy as np

@dataclass
class ZoneSpec:
    """One zone sheet. All offsets in metres, in the zone tag's frame."""
    name: str
    tag_id: int
    x_min: float          # left edge of zone, from tag center
    x_max: float          # right edge
    y_min: float          # bottom edge (most negative y - far end of sheet)
    y_max: float          # top edge (closest to the tag, still negative)
    z_min: float = -0.02  # small tolerance below the paper plane
    z_max: float = 0.15   # cube height + margin above the paper plane
 
    def contains_local(self, p: np.ndarray) -> bool:
        """p = (3,) point already expressed in this zone tag's frame."""
        return (self.x_min <= p[0] <= self.x_max
                and self.y_min <= p[1] <= self.y_max
                and self.z_min <= p[2] <= self.z_max)


ZONES: List[ZoneSpec] = [
    ZoneSpec(name="PapierZone",    tag_id=0,
             x_min=-0.025, x_max=0.165, y_min=-0.251, y_max=-0.035),
    ZoneSpec(name="RestmuellZone", tag_id=1,
             x_min=-0.025, x_max=0.165, y_min=-0.251, y_max=-0.035),
    ZoneSpec(name="PlastikZone",   tag_id=2,
             x_min=-0.025, x_max=0.165, y_min=-0.251, y_max=-0.035),
]

CUBE_IDS: List[int] = [3, 4, 5, 6]


class ZoneMap:
    """Answers 'which zone is this cube in?' from live TagWorld state."""
 
    def __init__(self, world, zones: List[ZoneSpec]):
        self.world = world
        self.zones = {z.tag_id: z for z in zones}
 
    def visible_zones(self) -> List[ZoneSpec]:
        ids = set(self.world.tag_ids())
        return [z for z in self.zones.values() if z.tag_id in ids]
 
    def zone_of(self, cube_tag_id: int) -> Optional[ZoneSpec]:
        """Zone containing the cube, or None (not visible / in no zone)."""
        cube_pos = self.world.position(cube_tag_id)
        if cube_pos is None:
            return None
        cube_h = np.append(cube_pos, 1.0)
 
        for zone in self.visible_zones():
            ref_T_zone = self.world.pose(zone.tag_id)
            if ref_T_zone is None:
                continue
            local = (np.linalg.inv(ref_T_zone) @ cube_h)[:3]
            if zone.contains_local(local):
                return zone
        return None
 
    def sort_state(self, cube_ids: List[int]) -> Dict[str, List[int]]:
        """Full picture in one call: zone name -> cubes in it, plus the
        visible-but-unassigned cubes under 'unassigned'."""
        state: Dict[str, List[int]] = {z.name: [] for z in self.zones.values()}
        state["unassigned"] = []
        for cid in cube_ids:
            if self.world.position(cid) is None:
                continue                       # not visible: report nothing
            zone = self.zone_of(cid)
            state[zone.name if zone else "unassigned"].append(cid)
        return state
 
    def drop_pose(self, zone_tag_id: int,
                  hover_m: float = 0.10) -> Optional[np.ndarray]:
        """4x4 pose at the zone rectangle's center, hover_m above the paper,
        in the world reference frame. Feed this to the motion node as the
        place target for that zone."""
        zone = self.zones.get(zone_tag_id)
        ref_T_zone = self.world.pose(zone_tag_id)
        if zone is None or ref_T_zone is None:
            return None
        center_local = np.array([
            (zone.x_min + zone.x_max) / 2.0,
            (zone.y_min + zone.y_max) / 2.0,
            hover_m, 1.0])
        T = ref_T_zone.copy()
        T[:3, 3] = (ref_T_zone @ center_local)[:3]
        return T