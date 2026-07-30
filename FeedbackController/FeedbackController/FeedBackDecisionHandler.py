from __future__ import annotations
 
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Sequence, Tuple
 
from vision_module.WorldClient import TagWorld
from vision_module.Zone import ZONES, ZoneMap, ZoneSpec
import vision_module.TagRegistry as tr


NON_ZONE_KEYS = ("unassigned", "implausible")


class SortState(IntEnum):
    CORRECT = 0 
    INCORRECT = 1 
    UNKNOWN = 2 


@dataclass
class Verdict:
    state: SortState
    reason: str

    sensing_ok: bool = True
    misplaced: Dict[int, Tuple[str, str]] = field(default_factory=dict)
    unassigned: List[int] = field(default_factory=list)
    unknown_cubes: List[int] = field(default_factory=list)
    implausible: List[int] = field(default_factory=list)
    visible_cubes: List[int] = field(default_factory=list)
 
    def key(self) -> tuple:
        return (self.state,
                tuple(sorted(self.misplaced.items())),
                tuple(sorted(self.unassigned)),
                tuple(sorted(self.unknown_cubes)))


class FeedBackDecisionHandler():
    def __init__(self, world: TagWorld, zones=ZONES, category: str = "trash"):

        self.world = world
        self.zones = ZoneMap(world, zones)
        self.cube_ids = tr.ids_in(category)


    def evaluate(self) -> Verdict:
        if not self.world.fresh:
            return Verdict(SortState.UNKNOWN, "No Fresh Data", sensing_ok=False)

        visible = [c for c in self.cube_ids
                   if self.world.position(c) is not None]

        if not visible:
            return Verdict(SortState.UNKNOWN, "no cubes visible", sensing_ok=False)
 
        if not self.zones.visible_zones():
            return Verdict(SortState.UNKNOWN, "no zone tags visible", sensing_ok=False,
                           visible_cubes=visible)

        state = self.zones.sort_state(self.cube_ids)
        implausible = list(state.get("implausible", []))

        unassigned = [c for c in state.get("unassigned", [])
                      if not tr.is_unknown_trash(c)]

        misplaced: Dict[int, Tuple[str, str]] = {}
        unknown_cubes: List[int] = []

        for zone_name, cubes in state.items():
            if zone_name in NON_ZONE_KEYS:
                continue

            for cube_id in cubes:
                if tr.info(cube_id) is None:
                    return Verdict(
                        SortState.UNKNOWN,
                        f"cube {cube_id} is not in TagRegistry",
                        visible_cubes=visible, implausible=implausible)
 
                # Cube Tag ID 6
                if tr.is_unknown_trash(cube_id):
                    unknown_cubes.append(cube_id)
                    continue

                target_zone_id = tr.target_zone(cube_id)
                target = self.zones.zones.get(target_zone_id)

                if target is None:
                    return Verdict(
                        SortState.UNKNOWN,
                        f"target zone {target_zone_id} for cube {cube_id} "
                        f"is not in ZONES",
                        visible_cubes=visible, implausible=implausible)
 
                if target.name != zone_name:
                    misplaced[cube_id] = (zone_name, target.name)
 
            common = dict(misplaced=misplaced, unassigned=unassigned,
                      unknown_cubes=unknown_cubes, implausible=implausible,
                      visible_cubes=visible)

        if misplaced:
            return Verdict(SortState.INCORRECT,
                           f"{len(misplaced)} cube(s) in the wrong zone",
                           **common)
        if unassigned:
            return Verdict(SortState.INCORRECT,
                           f"{len(unassigned)} cube(s) in no zone", **common)
 
        if unknown_cubes:
            return Verdict(SortState.UNKNOWN,
                           f"cube(s) {unknown_cubes} are unknown trash - "
                           f"no correct zone to check against", **common)
 
        return Verdict(SortState.CORRECT,
                       f"all {len(visible)} visible cube(s) correct", **common)


def parse_state(raw: str) -> SortState | None:
    """SortState from a string on /sorting/result: "CORRECT", "correct",
    or the raw enum value "0". None if it means nothing."""
    key = str(raw).strip().upper()
    if key in SortState.__members__:
        return SortState[key]
    if key.isdigit() and int(key) in set(s.value for s in SortState):
        return SortState(int(key))
    return None 
