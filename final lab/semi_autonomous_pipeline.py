import argparse
import os
import sys
import time
from typing import Dict, List

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from hello_helpers.hello_misc import HelloNode
from rclpy.node import Node
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult

from task_planner import build_rearrangement_plan

from live_object_localizer import collect_live_object_map, merge_object_maps


LAB3_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Lab3"))
if LAB3_PATH not in sys.path:
    sys.path.append(LAB3_PATH)

import ik_ros_utils as ik  # noqa: E402


def set_pose(pose: List[float], pose_msg: PoseStamped, frame_id: str, stamp_node: Node) -> None:
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
    """Reuse Lab3 IK helper patterns for ready/pick/place moves."""

    def __init__(self) -> None:
        super().__init__()

    def setup(self) -> None:
        HelloNode.main(self, "semi_auto_manipulator", "semi_auto_manipulator", wait_for_first_pointcloud=False)
        self.stow_the_robot()
        self.move_to_pose(ik.READY_POSE_P2, blocking=True)

    def pick_object(self, label: str) -> None:
        self.get_logger().info(f"[Pick] Attempting pickup for {label}")
        # Keep manip action style consistent with Lab3 examples.
        self.move_to_pose({"joint_gripper_finger_left": 0.84, "joint_gripper_finger_right": 0.84}, blocking=True)
        self.move_to_pose({"joint_arm": 0.35, "joint_lift": 0.75, "joint_wrist_pitch": -0.4}, blocking=True)
        self.move_to_pose({"joint_gripper_finger_left": 0.0, "joint_gripper_finger_right": 0.0}, blocking=True)
        self.move_to_pose(ik.READY_POSE_P2, blocking=True)

    def place_object(self, label: str) -> None:
        self.get_logger().info(f"[Place] Attempting placement for {label}")
        self.move_to_pose({"joint_arm": 0.35, "joint_lift": 0.78, "joint_wrist_pitch": -0.4}, blocking=True)
        self.move_to_pose({"joint_gripper_finger_left": 0.84, "joint_gripper_finger_right": 0.84}, blocking=True)
        self.move_to_pose(ik.READY_POSE_P2, blocking=True)


class SemiAutonomousPipeline:
    def __init__(self, config_path: str, static_only: bool = False) -> None:
        self.config_path = config_path
        self._static_only = static_only
        with open(self.config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)
        self.navigator = BasicNavigator()
        self.manipulator = SemiAutoManipulator()

    def nav_to_pose(self, pose: List[float], timeout_s: float = 180.0) -> bool:
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
        # Nav2 init
        initial_pose = PoseStamped()
        set_pose(self.config["start_pose"], initial_pose, "map", self.navigator)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

        goal_regions: Dict[str, str] = self.config["goal_regions"]
        yaml_object_map: Dict[str, Dict[str, List[float]]] = self.config.get("object_map") or {}
        live_cfg = self.config.get("live_detection") or {}

        if live_cfg.get("enabled", False) and not self._static_only:
            self.navigator.get_logger().info("Live detection: collecting object poses in map frame…")
            live_map = collect_live_object_map(
                live_cfg,
                goal_regions,
                logger=self.navigator.get_logger(),
            )
            object_map = merge_object_maps(yaml_object_map, live_map, goal_regions)
            missing = [k for k in goal_regions if k not in object_map]
            if missing:
                self.navigator.get_logger().warning(
                    f"Live localization missing map poses for: {missing} (check prompts, TF map->camera, timeout)"
                )
        else:
            object_map = merge_object_maps(yaml_object_map, {}, goal_regions)

        # Manip init
        self.manipulator.setup()

        plan = build_rearrangement_plan(
            object_map=object_map,
            region_bounds=self.config["region_bounds"],
            goal_regions=goal_regions,
            region_centers=self.config["region_centers"],
        )
        self.navigator.get_logger().info(f"Generated {len(plan)} move tasks")
        if len(plan) == 0:
            self.navigator.get_logger().warning(
                "No rearrangement tasks: every object may already be in its goal region, "
                "or map poses/regions are wrong (UNKNOWN source), or object_map is empty "
                "for all goal_regions keys. Check live_detection, YAML object_map, and region_bounds."
            )

        for task in plan:
            self.navigator.get_logger().info(
                f"Task: {task.label} {task.source_region} -> {task.goal_region}"
            )
            if not self.nav_to_pose(task.pick_pose):
                self.navigator.get_logger().warning(f"Failed to reach pick pose for {task.label}")
                continue
            self.manipulator.pick_object(task.label)
            if not self.nav_to_pose(task.place_pose):
                self.navigator.get_logger().warning(f"Failed to reach place pose for {task.label}")
                continue
            self.manipulator.place_object(task.label)

        self.navigator.get_logger().info("Semi-autonomous pipeline complete.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Semi-autonomous object rearrangement")
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config", "regions.yaml"),
        help="Path to YAML scenario config",
    )
    parser.add_argument(
        "--static-only",
        action="store_true",
        help="Use only object_map from YAML (skip live YOLO + TF localization)",
    )
    args = parser.parse_args()

    rclpy.init()
    pipeline = SemiAutonomousPipeline(args.config, static_only=args.static_only)
    try:
        pipeline.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
