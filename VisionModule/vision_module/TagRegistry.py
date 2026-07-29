from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
 
DEFAULT_TAG_SIZE_M = 0.05
CORNER_TAG_SIZE_M = 0.045
 

@dataclass(frozen=True)
class TagInfo:
    tag_id: int
    name: str
    category: str                    # "zone_corner" | "trash" | "human"
    size_m: float = DEFAULT_TAG_SIZE_M
    sorts_to: Optional[int] = None   # trash only: logical zone id
    zone_id: Optional[int] = None    # zone_corner only: which zone it bounds

ZONE_PAPIER = 0
ZONE_RESTMUELL = 1
ZONE_PLASTIK = 2
 
ZONE_NAMES: Dict[int, str] = {
    ZONE_PAPIER: "PapierZone",
    ZONE_RESTMUELL: "RestmuellZone",
    ZONE_PLASTIK: "PlastikZone",
}
 
# Corner tag ids per zone. Order here is irrelevant - ZoneMap sorts corners
# geometrically, so any tag may go in any corner of its own sheet.
ZONE_CORNERS: Dict[int, Tuple[int, ...]] = {
    ZONE_PAPIER:    (100, 101, 102, 103),
    ZONE_PLASTIK:   (104, 105, 106, 107),
    ZONE_RESTMUELL: (108, 109, 110, 111),
} 

def _corner_tags() -> Dict[int, TagInfo]:
    out: Dict[int, TagInfo] = {}
    for zone_id, ids in ZONE_CORNERS.items():
        for n, tid in enumerate(ids):
            out[tid] = TagInfo(
                tid, f"{ZONE_NAMES[zone_id]}_c{n}", "zone_corner",
                size_m=CORNER_TAG_SIZE_M, zone_id=zone_id)
    return out
 
TAGS: Dict[int, TagInfo] = {
    **_corner_tags(),                                    # <-- missing
    3: TagInfo(3, "Papier",  "trash", sorts_to=ZONE_PAPIER),
    4: TagInfo(4, "Plastik", "trash", sorts_to=ZONE_PLASTIK),
    5: TagInfo(5, "Organic", "trash", sorts_to=ZONE_RESTMUELL),
    6: TagInfo(6, "Unknown", "trash", sorts_to=ZONE_RESTMUELL),
    67: TagInfo(67, "Human", "human"),
}
 
def info(tag_id: int) -> Optional[TagInfo]:
    return TAGS.get(tag_id)
 
def size_of(tag_id: int) -> float:
    t = TAGS.get(tag_id)
    return t.size_m if t else DEFAULT_TAG_SIZE_M
 
def tag_sizes() -> Dict[int, float]:
    """{tag_id: size_m} for detector / fuser construction."""
    return {tid: t.size_m for tid, t in TAGS.items()}
 
def ids_in(category: str) -> List[int]:
    return sorted(tid for tid, t in TAGS.items() if t.category == category)
 
def target_zone(trash_tag_id: int) -> Optional[int]:
    """Where does this piece of trash belong? None if unknown tag."""
    t = TAGS.get(trash_tag_id)
    return t.sorts_to if t else None

 
def corners_of(zone_id: int) -> Tuple[int, ...]:
    return ZONE_CORNERS.get(zone_id, ())
 
 
def zone_of_corner(tag_id: int) -> Optional[int]:
    t = TAGS.get(tag_id)
    return t.zone_id if t else None
 
 
def zone_name(zone_id: int) -> str:
    return ZONE_NAMES.get(zone_id, f"Zone{zone_id}") 