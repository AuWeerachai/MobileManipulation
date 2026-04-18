# Final Lab: Object Rearrangement (Option B)

Semi-autonomous mode only. Fully-autonomous exploration + SLAM is intentionally
deferred so we can get a working end-to-end pick-and-place pipeline first.

## What's here

- `semi_autonomous_pipeline.py` — main runnable. Navigates, grasps with Lab3 IK,
  blind-drops, stows, repeats.
- `task_planner.py` — converts object map positions + goal regions + region
  bounds into a list of `MoveTask` records (with pick / place approach poses).
- `live_object_localizer.py` — Lab3-style YOLO-E + TF localizer. Dormant under
  Option B; kept in the repo for when we wire perception back in.
- `config/regions.yaml` — scenario config (start pose, regions, objects, goals).

Fully-autonomous exploration + SLAM is deferred to a later iteration and not
present in this folder.

## What the pipeline reuses

| Component                          | Source                              |
| ---------------------------------- | ----------------------------------- |
| `stow_the_robot`, `move_to_pose`   | Lab1 `stretch_ros.py`               |
| `READY_POSE_P2`, IK chain, helpers | Lab3 `ik_ros_utils.py`              |
| TF buffer / listener pattern       | Lab3 `grasp_objects.py`             |
| `BasicNavigator` nav flow          | Lab4 `nav2_stretch.py`              |

No new external ROS packages are required beyond what Labs 1-4 already use.

## Assumptions

- A saved map is available and you are running Nav2 + localization (e.g.,
  `map_server` + AMCL) against it. Lab4 already produced `asangium.pgm` /
  `asangium.yaml`; reuse those or replace with your own.
- The object coordinates in `config/regions.yaml` are measured in that same
  map frame. Under Option B they are authoritative — no perception in the loop.
- Your Stretch has YOLO-E and the usual Lab3 dependencies installed (only
  needed if you turn live detection back on later).

## Running it

1. Bring up the Stretch driver and cameras (separate terminals):

   ```bash
   ros2 launch stretch_core stretch_driver.launch.py
   ros2 launch stretch_core d435i_low_resolution.launch.py   # head cam (for future use)
   ```

2. Launch Nav2 against your map (e.g., the Lab4 map). Whatever command you used
   in Lab4 for `nav2_stretch.py` is the right one here — this script expects
   `waitUntilNav2Active()` to succeed.

3. Run the pipeline:

   ```bash
   python "final lab/semi_autonomous_pipeline.py" --config "final lab/config/regions.yaml"
   ```

## Per-task flow

For each `MoveTask`:

1. Stow the arm.
2. Nav2 to the pick approach pose (`approach_offset_m` meters in -X of the
   object, facing +X).
3. Move to `READY_POSE_P2` (arm up, wrist yaw aligned, head pointed at arm).
4. TF-transform the known object position from `map` into `base_link`.
5. Lab3 IK grasp: open gripper, `ik.get_current_configuration` →
   `ik.get_grasp_goal` → `ik.move_to_configuration`, close gripper, retract to
   `READY_POSE_P2`.
6. Stow, Nav2 to the place approach pose (same offset from the goal region
   center).
7. Blind drop: extend arm, open gripper, retract.
8. Stow.

## Tuning knobs in `config/regions.yaml`

- `start_pose` — robot's starting pose in the map frame.
- `approach_offset_m` — stand-off distance for pick + place approach poses.
- `region_bounds` — axis-aligned boxes in the map frame. Used to compute each
  object's current region.
- `region_centers` — reference pose for each region; only the x, y are used to
  derive the place approach pose.
- `object_map` — map-frame pose per object. Edit these to match the real scene.
- `goal_regions` — desired region per object.
- `live_detection` — leave `enabled: false` for Option B.

## What's intentionally simple (and worth upgrading later)

- **Blind place**. We extend the arm to a fixed preset and open the gripper.
  Good enough for "drop somewhere in region X"; replace with IK to a specific
  place pose if you need precision.
- **No perception in the loop**. `live_object_localizer.py` is ready to be
  wired in (either as an initial head-scan discovery pass or as a re-detection
  step right before grasp), but Option B defers it so the first runs are
  deterministic.
- **Fixed yaw for approach poses**. The planner parks at `yaw = 0` relative to
  each target. If certain objects need a different facing (obstacles, walls),
  extend `task_planner.compute_approach_pose` to take a per-object yaw override.

## When you're ready to go fully-autonomous

Write a new script in this folder that:

1. Runs SLAM Toolbox to build a 2D map while driving the robot.
2. Uses a real exploration package (e.g., `m-explore-ros2`) or a custom
   head-scan / rotate-in-place explorer built on Lab1/Lab4 primitives.
3. Saves the map with `nav2_map_server`.
4. Re-launches Nav2 + localization (`map_server` + AMCL) against the saved map
   before invoking `semi_autonomous_pipeline.py`.
5. Optionally localizes the three target objects during exploration using the
   dormant `live_object_localizer.py`, then writes them into the YAML before
   rearrangement.
