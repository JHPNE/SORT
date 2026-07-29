from VisionModule.vision_module.WorldClient import TagWorld
from VisionModule.vision_module.Zone import ZONES, ZoneMap
import VisionModule.vision_module.TagRegistry as tr

from dataclasses import dataclass
from abc import ABC, abstractmethod

class FeedbackType(ABC):
    @abstractmethod
    def handle_feedback(self):
        pass

class FeedBackNegative(FeedbackType):
    def handle_feedback(self):
        return ""

class FeedBackPositive(FeedbackType):
    def handle_feedback(self):
        return ""


class FeedBackDecisionHandler():
    def __init__(self, world: TagWorld):
        self.world = world
        self.feedback_type = self._pick_feedback(world)
        self.zones = ZoneMap(self.world, ZONES)
        self.cube_ids = tr.ids_in("trash")


    def _pick_feedback(self, world: TagWorld) -> FeedbackType:
        if self._has_correct_alignment(world):
            return FeedBackPositive

        return FeedBackNegative


    def _has_correct_alignment(self, world: TagWorld) -> bool:
        zone_state = self.zones.sort_state(self.cube_ids)

        for zone_name, cubes in zone_state.items():
            if zone_name == "unassigned":
                if cubes:
                    return False
                continue

            for cube_id in cubes:
                expected_zone_id = tr.target_zone(cube_id)
                if expected_zone_id is None:
                    return False

                expected_zone = self.zones.zones.get(expected_zone_id)
                if expected_zone is None or expected_zone.name != zone_name:
                    return False

        return True

    def get_feedback(self):
        return self.feedback_type
