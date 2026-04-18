"""
Rule-based task planner for the Option B semi-autonomous pipeline.

Given map-frame object positions, region bounds, region centers, and goal region
assignments, emit a list of MoveTask entries. Each MoveTask carries enough
information for the pipeline to:
  1. navigate to a stand-off "approach pose" near the object,
  2. use Lab3 IK to grasp the object at its known map position,
  3. navigate to an "approach pose" near the goal region center,
  4. blind-drop in that region.

The approach pose is placed `approach_offset_m` meters in -X of the target and
faces +X (yaw = 0). Lab3 IK's virtual base rotation/translation joints will
refine the final alignment during grasping, so a rough stand-off is sufficient.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class MoveTask:
    label: str
    source_region: str
    goal_region: str
    object_map_xyz: List[float]          # [x, y, z] of object in map frame
    pick_approach_pose: List[float]      # Nav2 pose [x, y, z, qx, qy, qz, qw]
    place_approach_pose: List[float]     # Nav2 pose near region center


def assign_region(
    xy: Tuple[float, float],
    region_bounds: Dict[str, Dict[str, List[float]]],
) -> str:
    """Return the region name whose axis-aligned box contains xy, else 'UNKNOWN'."""
    x, y = xy
    for region_name, bounds in region_bounds.items():
        x_min, x_max = bounds["x"]
        y_min, y_max = bounds["y"]
        if x_min <= x <= x_max and y_min <= y <= y_max:
            return region_name
    return "UNKNOWN"


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Return (qx, qy, qz, qw) for a yaw-only rotation about Z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def compute_approach_pose(
    target_xy: Tuple[float, float],
    approach_offset_m: float,
    yaw: float = 0.0,
) -> List[float]:
    """
    Park the robot `approach_offset_m` meters away from target, facing +X by default.

    The base lands at (tx - offset*cos(yaw), ty - offset*sin(yaw)) so that driving
    forward along the robot's +X brings it toward the target. Lab3 IK handles the
    final reach via the virtual base rotation + translation joints.
    """
    tx, ty = target_xy
    rx = tx - approach_offset_m * math.cos(yaw)
    ry = ty - approach_offset_m * math.sin(yaw)
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    return [rx, ry, 0.0, qx, qy, qz, qw]


def build_rearrangement_plan(
    object_map: Dict[str, Dict[str, List[float]]],
    region_bounds: Dict[str, Dict[str, List[float]]],
    goal_regions: Dict[str, str],
    region_centers: Dict[str, List[float]],
    approach_offset_m: float = 0.5,
) -> List[MoveTask]:
    """
    Greedy / rule-based planner:
      - detect each object's current region from its map coordinate
      - if current != goal, emit a MoveTask with pick + place approach poses
    """
    plan: List[MoveTask] = []
    for label, info in object_map.items():
        map_pose = info["map_pose"]  # [x, y, z, qx, qy, qz, qw] in map frame
        obj_xy = (map_pose[0], map_pose[1])

        source = assign_region(obj_xy, region_bounds)
        goal = goal_regions.get(label, source)
        if source == "UNKNOWN" or goal not in region_centers or source == goal:
            continue

        goal_center = region_centers[goal]
        goal_xy = (goal_center[0], goal_center[1])

        plan.append(
            MoveTask(
                label=label,
                source_region=source,
                goal_region=goal,
                object_map_xyz=[float(map_pose[0]), float(map_pose[1]), float(map_pose[2])],
                pick_approach_pose=compute_approach_pose(obj_xy, approach_offset_m),
                place_approach_pose=compute_approach_pose(goal_xy, approach_offset_m),
            )
        )
    return plan
