"""
Semi-autonomous object rearrangement pipeline (Option B).

Reuses Lab1-Lab4 patterns:
  - Lab4 nav2_stretch.py:   BasicNavigator, setInitialPose / waitUntilNav2Active /
                            goToPose / isTaskComplete / TaskResult.
  - Lab3 grasp_objects.py:  HelloNode, tf2 buffer/listener, IK-driven grasp using
                            ik_ros_utils (READY_POSE_P2, chain, get_current_configuration,
                            get_grasp_goal, move_to_configuration).
  - Lab1 stretch_ros.py:    stow_the_robot / move_to_pose conventions.

Assumptions (Option B):
  - A map is already saved (e.g., Lab4 asangium.yaml/pgm).
  - Nav2 + localization is already running against that map.
  - Object positions in config/regions.yaml are authoritative (no live detection).

Per-task flow:
  stow -> nav to pick approach -> ready pose -> TF(map->base_link) for the object
  position -> Lab3 IK grasp -> close gripper -> retract -> stow -> nav to place
  approach -> ready pose -> blind drop -> stow.
"""

import argparse
import os
import sys
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
import yaml
import ikpy.utils.geometry
import tf2_ros
from geometry_msgs.msg import PoseStamped
from hello_helpers.hello_misc import HelloNode
from rclpy.duration import Duration
from rclpy.node import Node
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from tf2_geometry_msgs import do_transform_pose_stamped

from task_planner import build_rearrangement_plan

# Reuse Lab3 IK helper (chain, READY_POSE_P2, get_current_configuration,
# get_grasp_goal, move_to_configuration, print_q).
LAB3_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Lab3"))
if LAB3_PATH not in sys.path:
    sys.path.append(LAB3_PATH)

import ik_ros_utils as ik  # noqa: E402


# Five joints the Lab3 IK chain needs to read from the current JointState.
IK_JOINT_NAMES = [
    "joint_lift",
    "joint_arm_l0",
    "joint_wrist_yaw",
    "joint_wrist_pitch",
    "joint_wrist_roll",
]


def set_pose(
    pose: List[float], pose_msg: PoseStamped, frame_id: str, stamp_node: Node
) -> None:
    """Fill a PoseStamped from a 7-vector [x, y, z, qx, qy, qz, qw]. Matches Lab4."""
    pose_msg.header.frame_id = frame_id
    pose_msg.header.stamp = stamp_node.get_clock().now().to_msg()
    pose_msg.pose.position.x = pose[0]
    pose_msg.pose.position.y = pose[1]
    pose_msg.pose.position.z = pose[2]
    pose_msg.pose.orientation.x = pose[3]
    pose_msg.pose.orientation.y = pose[4]
    pose_msg.pose.orientation.z = pose[5]
    pose_msg.pose.orientation.w = pose[6]


class SemiAutoManipulator(HelloNode):
    """HelloNode-based manipulator. Reuses Lab3 IK helper for real grasp moves."""

    def __init__(self) -> None:
        super().__init__()
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None

    def setup(self) -> None:
        """Start the HelloNode executor thread, add TF, stow, then move to READY_POSE_P2."""
        HelloNode.main(
            self,
            "semi_auto_manipulator",
            "semi_auto_manipulator",
            wait_for_first_pointcloud=False,
        )
        # Lab3 pattern: buffer + listener for TF lookups.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.stow_the_robot()
        self.move_to_pose(ik.READY_POSE_P2, blocking=True)

    def ready_pose(self) -> None:
        """Return to the Lab3 ready pose used for grasping."""
        self.move_to_pose(ik.READY_POSE_P2, blocking=True)

    def stow(self) -> None:
        """Stow the arm. Always call before base navigation."""
        self.stow_the_robot()

    def _joint_state_dict(self) -> Optional[Dict[str, float]]:
        """Extract the 5 joints needed by ik_ros_utils from HelloNode's joint_state msg."""
        if self.joint_state is None:
            return None
        out: Dict[str, float] = {}
        for n in IK_JOINT_NAMES:
            if n not in self.joint_state.name:
                return None
            i = self.joint_state.name.index(n)
            out[n] = self.joint_state.position[i]
        return out

    def transform_map_point_to_base(
        self, map_xyz: List[float], timeout_s: float = 2.0
    ) -> Optional[np.ndarray]:
        """TF-transform a map-frame 3D point into base_link. Returns None on failure."""
        if self.tf_buffer is None:
            return None

        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(map_xyz[0])
        ps.pose.position.y = float(map_xyz[1])
        ps.pose.position.z = float(map_xyz[2])
        ps.pose.orientation.w = 1.0
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link",
                "map",
                rclpy.time.Time(),
                timeout=Duration(seconds=timeout_s),
            )
            ps_base = do_transform_pose_stamped(ps, tf)
            return np.array(
                [ps_base.pose.position.x, ps_base.pose.position.y, ps_base.pose.position.z]
            )
        except Exception as e:  # tf2 can raise several distinct exceptions
            self.get_logger().warning(f"TF map->base_link lookup failed: {e}")
            return None

    def open_gripper(self) -> None:
        self.move_to_pose(
            {
                "joint_gripper_finger_left": 0.84,
                "joint_gripper_finger_right": 0.84,
            },
            blocking=True,
        )

    def close_gripper(self) -> None:
        self.move_to_pose(
            {
                "joint_gripper_finger_left": 0.0,
                "joint_gripper_finger_right": 0.0,
            },
            blocking=True,
        )

    def grasp_at_base_xyz(self, target_xyz_base: np.ndarray) -> bool:
        """Lab3-style IK grasp: open gripper, IK to target, close, retract to ready."""
        js = self._joint_state_dict()
        if js is None:
            self.get_logger().warning("Joint state not populated yet; cannot run IK.")
            return False

        # Identity rotation — we only care about position for picking (see Lab3).
        orient = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)
        q_init = ik.get_current_configuration(js)
        q_soln = ik.get_grasp_goal(target_xyz_base, orient, q_init)
        ik.print_q(q_soln)
        if q_soln is None:
            self.get_logger().warning("IK found no valid solution for this target.")
            return False

        self.open_gripper()
        ik.move_to_configuration(self, q_soln)
        self.close_gripper()
        # Retract to ready so the base can move safely afterwards.
        self.move_to_pose(ik.READY_POSE_P2, blocking=True)
        return True

    def place_blind(self) -> None:
        """
        Simple blind drop used in Option B: extend the arm, open the gripper,
        retract to ready. Good enough for "put it somewhere inside the goal region".
        Replace with an IK-based place pose later if you need precision.
        """
        self.move_to_pose(
            {
                "joint_arm": 0.35,
                "joint_lift": 0.78,
                "joint_wrist_pitch": -0.4,
            },
            blocking=True,
        )
        self.open_gripper()
        self.move_to_pose(ik.READY_POSE_P2, blocking=True)


class SemiAutonomousPipeline:
    def __init__(self, config_path: str) -> None:
        self.config_path = config_path
        with open(self.config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

        # Nav2 helper (Lab4 style). BasicNavigator is itself a Node.
        self.navigator = BasicNavigator()
        # HelloNode-based manipulator. Spins its own executor thread after setup().
        self.manipulator = SemiAutoManipulator()

    def nav_to_pose(self, pose: List[float], timeout_s: float = 180.0) -> bool:
        """Lab4-style blocking navigation to a single map-frame pose."""
        goal = PoseStamped()
        set_pose(pose, goal, "map", self.navigator)
        self.navigator.goToPose(goal)
        start = time.time()
        while not self.navigator.isTaskComplete():
            if time.time() - start > timeout_s:
                self.navigator.cancelTask()
                return False
            time.sleep(0.1)
        return self.navigator.getResult() == TaskResult.SUCCEEDED

    def run(self) -> None:
        # Nav2 initial pose + wait for bringup (Lab4 pattern).
        initial_pose = PoseStamped()
        set_pose(self.config["start_pose"], initial_pose, "map", self.navigator)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

        # Bring up the manipulator AFTER Nav2 so TF(map->base_link) is available.
        self.manipulator.setup()

        approach_offset_m = float(self.config.get("approach_offset_m", 0.5))
        object_map = self.config.get("object_map") or {}
        goal_regions = self.config["goal_regions"]

        plan = build_rearrangement_plan(
            object_map=object_map,
            region_bounds=self.config["region_bounds"],
            goal_regions=goal_regions,
            region_centers=self.config["region_centers"],
            approach_offset_m=approach_offset_m,
        )

        logger = self.navigator.get_logger()
        logger.info(f"Generated {len(plan)} move tasks")
        if not plan:
            logger.warning(
                "No rearrangement tasks were produced. Either every object is already in "
                "its goal region, region_bounds don't contain the object positions "
                "(UNKNOWN source), or object_map is missing entries."
            )

        for task in plan:
            logger.info(f"Task: {task.label} {task.source_region} -> {task.goal_region}")

            # 1) Stow before base motion.
            self.manipulator.stow()

            # 2) Nav to pick approach pose.
            if not self.nav_to_pose(task.pick_approach_pose):
                logger.warning(f"Failed to reach pick approach pose for {task.label}")
                continue

            # 3) Ready pose + tiny settle so TF is fresh.
            self.manipulator.ready_pose()
            time.sleep(0.5)

            # 4) Turn the known map-frame object position into a base_link target.
            target_xyz = self.manipulator.transform_map_point_to_base(task.object_map_xyz)
            if target_xyz is None:
                logger.warning(f"TF map->base_link failed for {task.label}; skipping.")
                continue

            # 5) Lab3 IK-based grasp (open, IK, close, retract).
            if not self.manipulator.grasp_at_base_xyz(target_xyz):
                logger.warning(f"Grasp failed for {task.label}; skipping to next task.")
                continue

            # 6) Stow + nav to place approach.
            self.manipulator.stow()
            if not self.nav_to_pose(task.place_approach_pose):
                logger.warning(f"Failed to reach place approach pose for {task.label}")
                continue

            # 7) Blind place in goal region.
            self.manipulator.ready_pose()
            self.manipulator.place_blind()

            # 8) Stow after placing.
            self.manipulator.stow()

        logger.info("Semi-autonomous pipeline complete.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Semi-autonomous object rearrangement (Option B)"
    )
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config", "regions.yaml"),
        help="Path to YAML scenario config",
    )
    args = parser.parse_args()

    rclpy.init()
    pipeline = SemiAutonomousPipeline(args.config)
    try:
        pipeline.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
