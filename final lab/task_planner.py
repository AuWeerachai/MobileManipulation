from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class MoveTask:
    label: str
    source_region: str
    goal_region: str
    pick_pose: List[float]
    place_pose: List[float]


def assign_region(
    xy: Tuple[float, float], region_bounds: Dict[str, Dict[str, List[float]]]
) -> str:
    """Return region name for a 2D map coordinate."""
    x, y = xy
    for region_name, bounds in region_bounds.items():
        x_min, x_max = bounds["x"]
        y_min, y_max = bounds["y"]
        if x_min <= x <= x_max and y_min <= y <= y_max:
            return region_name
    return "UNKNOWN"


def build_rearrangement_plan(
    object_map: Dict[str, Dict[str, List[float]]],
    region_bounds: Dict[str, Dict[str, List[float]]],
    goal_regions: Dict[str, str],
    region_centers: Dict[str, List[float]],
) -> List[MoveTask]:
    """
    Greedy/rule-based planner:
      - detect each object's current region from map coordinate
      - if current != goal, add a move task to place at goal region center
    """
    plan: List[MoveTask] = []
    for label, info in object_map.items():
        map_pose = info["map_pose"]
        source = assign_region((map_pose[0], map_pose[1]), region_bounds)
        goal = goal_regions.get(label, source)
        if source == "UNKNOWN" or goal not in region_centers or source == goal:
            continue
        plan.append(
            MoveTask(
                label=label,
                source_region=source,
                goal_region=goal,
                pick_pose=map_pose,
                place_pose=region_centers[goal],
            )
        )
    return plan
