from dataclasses import dataclass
from typing import Dict, List, Optional
 
DEFAULT_TAG_SIZE_M = 0.05
 
 
@dataclass(frozen=True)
class TagInfo:
    tag_id: int
    name: str
    category: str                    # "zone" | "trash" | "human"
    size_m: float = DEFAULT_TAG_SIZE_M
    sorts_to: Optional[int] = None   # trash only: tag_id of its target zone
 
 
TAGS: Dict[int, TagInfo] = {
    # Zones (A4 sheets)
    0: TagInfo(0, "PapierZone",   "zone"),
    1: TagInfo(1, "RestmuellZone", "zone"),
    2: TagInfo(2, "PlastikZone",  "zone"),
    # Trash cubes
    3: TagInfo(3, "Papier",   "trash", sorts_to=0),
    4: TagInfo(4, "Plastik",  "trash", sorts_to=2),
    5: TagInfo(5, "Organic",  "trash", sorts_to=1),
    6: TagInfo(6, "Unknown",  "trash", sorts_to=1),
    # Human marker (e.g. wristband) - motion node should treat as keep-out
    7: TagInfo(7, "Human", "human"),
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
 