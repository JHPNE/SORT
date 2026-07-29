from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional

from vision_module.WorldClient import TagWorld
from vision_module.Zone import ZONES, ZoneMap
import vision_module.TagRegistry as tr

class SortState(IntEnum):
    CORRECT = 0 
    INCORRECT = 1 
    UNKNOWN = 2 

@dataclass
class Verdict:
    state: SortState
    reason: str
    zone: int = 2

    misplaced: Dict[int, tuple] = field(default_factory=dict)
    unassigned: List[int] = field(default_factory=list)
    visible_cubes: List[int] = field(default_factory=list)
 
class FeedbackType(ABC):
    @abstractmethod
    def message(self, verdict: Verdict) -> int:
        ...

class FeedbackPositive(FeedbackType):
    def message(self) -> int:
        return SortState.CORRECT
 
 
class FeedbackNegative(FeedbackType):
    def message(self, verdict: Verdict) -> str:
        return SortState.INCORRECT 
 
 
class FeedbackSilent(FeedbackType):
    def message(self) -> str:
        return SortState.UNKNOWN 


class FeedBackDecisionHandler():
    def __init__(self, world: TagWorld, zones=ZONES, category: str = "trash"):

        self.world = world
        self.zones = ZoneMap(world, ZONES)
        self.cube_ids = tr.ids_in(category)


    def evaluate(self) -> Verdict:
        if not self.world.fresh:
            return Verdict(SortState.UNKNOWN, "No Fresh Data", 2)

        visible = [c for c in self.cube_ids
                   if self.world.position(c) is not None]

        if not visible:
            return Verdict(SortState.UNKNOWN, "no cubes visible", 2)
 
        if not self.zones.visible_zones():
            return Verdict(SortState.UNKNOWN, "no zone tags visible", 2,
                           visible_cubes=visible)

        state = self.zones.sort_state(self.cube_ids)

        unassigned = list(state.get("unassigned", []))
        misplaced: Dict[int, tuple] = {}
        for zone_name, cubes in state.items():
            if zone_name == "unassigned":
                continue

            for cube_id in cubes:
                target_tag = tr.target_zone(cube_id)

                if target_tag is None:
                    return Verdict(
                        SortState.UNKNOWN,
                        f"cube {cube_id} has no target zone in TagRegistry", 2,
                        visible_cubes=visible)

                target = self.zones.zones.get(target_tag)
                if target is None:
                    return Verdict(
                        SortState.UNKNOWN,
                        f"target zone tag {target_tag} for cube {cube_id} ", 2,
                        f"is not in ZONES",
                        visible_cubes=visible)
                if target.name != zone_name:
                    misplaced[cube_id] = (zone_name, target.name)

        if unassigned:
            return Verdict(SortState.INCORRECT,
                           f"{len(unassigned)} cube(s) in no zone", 1,
                           misplaced, unassigned, visible)
        if misplaced:
            return Verdict(SortState.INCORRECT,
                           f"{len(misplaced)} cube(s) in the wrong zone", 1,
                           misplaced, unassigned, visible)
 
        return Verdict(SortState.CORRECT,
                       f"all {len(visible)} visible cube(s) correct", 2,
                       visible_cubes=visible)


    def feedback_for(self, verdict: Verdict) -> FeedbackType:
        return {
            SortState.CORRECT: FeedbackPositive(),
            SortState.INCORRECT: FeedbackNegative(),
            SortState.UNKNOWN: FeedbackSilent(),
        }[verdict.state]
