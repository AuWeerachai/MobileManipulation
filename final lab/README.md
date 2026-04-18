# Final Lab: Object Rearrangement

This folder implements two project modes from the proposal.

## 1) Semi-autonomous mode (Lab1-Lab4 resources only)

Script: `semi_autonomous_pipeline.py`

What it reuses:
- `Lab4` navigation pattern (`stretch_nav2.robot_navigator.BasicNavigator`)
- `Lab3` manipulation style (`HelloNode`, ready poses, IK helper import)
- `Lab3`/`Lab2` pick-place command conventions (`move_to_pose` joint dictionaries)
- `Lab3` perception stack for **live** localization (`live_object_localizer.py`: YOLO-E, `message_filters`, `detection_utils`, `tf2` → `map`)
- Rule-based planning for region assignment and rearrangement

Current assumptions:
- A map already exists and Nav2 stack is running.
- With `live_detection.enabled: true` in `config/regions.yaml`, the pipeline runs a short **collection phase** after Nav2 is active: it estimates each object’s **pick pose in `map`** from the gripper or head camera (same topics as Lab3), then averages samples for stability. Any object still missing falls back to `object_map` in the YAML.
- Region boundaries and goals are predefined in the same file.

Run:

```bash
python "final lab/semi_autonomous_pipeline.py" --config "final lab/config/regions.yaml"
```

YAML poses only (skip live detector):

```bash
python "final lab/semi_autonomous_pipeline.py" --config "final lab/config/regions.yaml" --static-only
```

## 2) Fully autonomous mode (separate script with extra libraries)

Script: `fully_autonomous_pipeline.py`

Adds external packages beyond Lab1-Lab4:
- `slam_toolbox` (2D SLAM)
- `nav2_wavefront_frontier_exploration` (frontier exploration)
- `nav2_map_server` (map saving)

Flow:
1. Starts SLAM Toolbox in background.
2. Starts frontier exploration in background.
3. Waits for user confirmation when exploration is sufficient.
4. Saves map.
5. Runs the semi-autonomous rearrangement script.

Run:

```bash
python "final lab/fully_autonomous_pipeline.py" --config "final lab/config/regions.yaml"
```

Skip exploration and only run rearrangement:

```bash
python "final lab/fully_autonomous_pipeline.py" --skip-exploration
```

## Planning helper

- `task_planner.py`: region assignment + greedy/rule-based rearrangement planner.
- `live_object_localizer.py`: multi-object map-frame localization (Lab3 patterns).

## Notes

- `config/regions.yaml` contains example coordinates; tune these for your environment.
- This folder is intentionally script-first and mirrors the style of Lab1-Lab4.
