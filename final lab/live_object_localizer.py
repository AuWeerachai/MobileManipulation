"""
Live multi-object localization for the final lab pipeline.

Mirrors Lab3 `object_detector.py` / `object_detector_pcd.py` patterns:
  - message_filters ApproximateTimeSynchronizer for color + depth + camera_info
  - Ultralytics YOLO-E + `detection_utils.parse_results`
  - `detection_utils.pixel_to_3d` + `get_pose_msg`
  - tf2: camera optical frame -> map

Accumulates map-frame positions per logical object key until timeout or all keys seen.
"""

from __future__ import annotations

import os
import os.path as osp
import sys
from collections import defaultdict
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
import message_filters
from ultralytics import YOLO

LAB3_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "Lab3"))

if LAB3_PATH not in sys.path:
    sys.path.append(LAB3_PATH)

import detection_utils  # noqa: E402


def _pose_to_map_list(pose: PoseStamped) -> List[float]:
    p = pose.pose.position
    o = pose.pose.orientation
    return [float(p.x), float(p.y), float(p.z), float(o.x), float(o.y), float(o.z), float(o.w)]


def _match_key_for_detection(
    label: str, prompts_and_keys: List[Dict[str, str]]
) -> Optional[str]:
    lab = label.strip().casefold()
    for item in prompts_and_keys:
        if lab == item["prompt"].strip().casefold():
            return item["key"]
    return None


def _centroid_depth_goal(
    detection: Dict[str, Any],
    latest_depth: np.ndarray,
    latest_cam_info: CameraInfo,
    stamp,
    frame_id: str,
) -> Optional[PoseStamped]:
    centroid = detection["centroid"]
    depth_val = latest_depth[centroid[1], centroid[0]]
    if depth_val == 0 or np.isnan(depth_val):
        return None
    xyz = detection_utils.pixel_to_3d(centroid, depth_val, latest_cam_info)
    return detection_utils.get_pose_msg(stamp, frame_id, xyz)


def _mask_centroid_goal(
    detection: Dict[str, Any],
    latest_depth: np.ndarray,
    latest_cam_info: CameraInfo,
    stamp,
    frame_id: str,
    color_shape,
) -> Optional[PoseStamped]:
    mask = detection["mask"]
    blank = np.zeros(color_shape[:2], dtype=np.uint8)
    cv2.fillPoly(blank, [mask], 1)
    mask_pix = np.argwhere(blank > 0)
    points_3d: List[np.ndarray] = []
    for y, x in mask_pix:
        depth_val = latest_depth[y, x]
        if depth_val == 0 or np.isnan(depth_val):
            continue
        xyz = detection_utils.pixel_to_3d((int(x), int(y)), depth_val, latest_cam_info)
        points_3d.append(xyz)
    if not points_3d:
        return None
    centroid_xyz = np.mean(points_3d, axis=0)
    return detection_utils.get_pose_msg(stamp, frame_id, centroid_xyz)


class LiveObjectLocalizer(Node):
    def __init__(
        self,
        prompts_and_keys: List[Dict[str, str]],
        camera_mode: str,
        model_dir: str,
        model_name: str,
        required_keys: List[str],
        min_samples_per_object: int,
        visualize: bool = False,
        depth_visualization_part: int = 1,
    ) -> None:
        super().__init__("live_object_localizer")
        self.prompts_and_keys = prompts_and_keys
        self.required_keys = set(required_keys)
        self.min_samples = max(1, int(min_samples_per_object))
        self.visualize = visualize
        self.depth_visualization_part = depth_visualization_part

        prompts = [p["prompt"] for p in prompts_and_keys]
        if not prompts:
            raise ValueError("live_detection.prompts_and_keys must be non-empty")

        self.camera_mode = camera_mode.strip().lower()
        if self.camera_mode == "gripper":
            color_topic = "/gripper_camera/color/image_rect_raw"
            depth_topic = "/gripper_camera/aligned_depth_to_color/image_raw"
            info_topic = "/gripper_camera/color/camera_info"
            self.optical_frame = "gripper_camera_color_optical_frame"
            self.rotate = False
            self.goal_builder = _centroid_depth_goal
        elif self.camera_mode == "head":
            color_topic = "/camera/color/image_rect_raw"
            depth_topic = "/camera/aligned_depth_to_color/image_raw"
            info_topic = "/camera/color/camera_info"
            self.optical_frame = "camera_color_optical_frame"
            self.rotate = True
            self.goal_builder = _mask_centroid_goal
        else:
            raise ValueError("camera_mode must be 'gripper' or 'head'")

        self.color_sub = message_filters.Subscriber(self, Image, color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, info_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.01,
        )
        self.sync.registerCallback(self._image_callback)

        self.bridge = CvBridge()
        self.latest_color: Optional[np.ndarray] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_cam_info: Optional[CameraInfo] = None

        model_path = osp.join(model_dir, model_name)
        self.model = YOLO(model_path)
        self.model.set_classes(prompts)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._positions: Dict[str, List[np.ndarray]] = defaultdict(list)

        self.create_timer(0.5, self._timer_callback)

    def _image_callback(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warning(f"cv_bridge error: {e}")
            return
        if self.rotate:
            color = cv2.rotate(color, cv2.ROTATE_90_CLOCKWISE)
            depth = cv2.rotate(depth, cv2.ROTATE_90_CLOCKWISE)
        self.latest_color = color
        self.latest_depth = depth
        self.latest_cam_info = info_msg

    def _transform_to_map(self, pose_cam: PoseStamped) -> Optional[PoseStamped]:
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                pose_cam.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0),
            )
            return do_transform_pose_stamped(pose_cam, tf)
        except Exception as e:
            self.get_logger().warning(f"TF map transform failed: {e}")
            return None

    def _timer_callback(self) -> None:
        if self.latest_color is None or self.latest_depth is None or self.latest_cam_info is None:
            return

        results = self.model(self.latest_color)
        detections = detection_utils.parse_results(results)

        if self.visualize and detections is not None:
            detection_utils.visualize_detections_masks(
                part=self.depth_visualization_part,
                detections=detections,
                rgb_image=self.latest_color,
                depth_image=self.latest_depth,
            )

        if not detections:
            return

        stamp = self.get_clock().now().to_msg()

        for det in detections:
            key = _match_key_for_detection(det["label"], self.prompts_and_keys)
            if key is None or key not in self.required_keys:
                continue

            if self.goal_builder is _mask_centroid_goal:
                pose_cam = self.goal_builder(
                    det,
                    self.latest_depth,
                    self.latest_cam_info,
                    stamp,
                    self.optical_frame,
                    self.latest_color.shape,
                )
            else:
                pose_cam = self.goal_builder(
                    det,
                    self.latest_depth,
                    self.latest_cam_info,
                    stamp,
                    self.optical_frame,
                )
            if pose_cam is None:
                continue
            pose_map = self._transform_to_map(pose_cam)
            if pose_map is None:
                continue
            p = pose_map.pose.position
            self._positions[key].append(np.array([p.x, p.y, p.z], dtype=np.float64))
            self.get_logger().info(
                f"Localized {key} at map ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"[samples={len(self._positions[key])}]"
            )

    def all_keys_satisfied(self) -> bool:
        for k in self.required_keys:
            if len(self._positions.get(k, ())) < self.min_samples:
                return False
        return True

    def get_object_map(self) -> Dict[str, Dict[str, List[float]]]:
        out: Dict[str, Dict[str, List[float]]] = {}
        for k, samples in self._positions.items():
            if not samples:
                continue
            mean = np.mean(np.stack(samples, axis=0), axis=0)
            out[k] = {
                "map_pose": [
                    float(mean[0]),
                    float(mean[1]),
                    float(mean[2]),
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ]
            }
        return out


def collect_live_object_map(
    live_cfg: Dict[str, Any],
    goal_regions: Dict[str, str],
    logger=None,
) -> Dict[str, Dict[str, List[float]]]:
    """
    Spin a temporary node to fill map poses for all keys in goal_regions.
    """
    log = logger.info if logger else print

    prompts_and_keys: List[Dict[str, str]] = live_cfg.get("prompts_and_keys") or []
    if not prompts_and_keys:
        raise ValueError("live_detection.prompts_and_keys is required when live mode is on")

    camera = live_cfg.get("camera", "gripper")
    model_dir = live_cfg.get("yolo_model_dir", "/home/hello-robot/models")
    model_name = live_cfg.get("yolo_model_name", "yoloe-26s-seg.pt")
    timeout_s = float(live_cfg.get("collect_timeout_s", 60.0))
    min_samples = int(live_cfg.get("min_samples_per_object", 2))
    visualize = bool(live_cfg.get("visualize", False))
    depth_part = int(live_cfg.get("depth_visualization_part", 1))

    required_keys = list(goal_regions.keys())
    node = LiveObjectLocalizer(
        prompts_and_keys=prompts_and_keys,
        camera_mode=str(camera),
        model_dir=model_dir,
        model_name=model_name,
        required_keys=required_keys,
        min_samples_per_object=min_samples,
        visualize=visualize,
        depth_visualization_part=depth_part,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    start = node.get_clock().now()
    while True:
        executor.spin_once(timeout_sec=0.05)
        if node.all_keys_satisfied():
            log("Live localization: all required objects sampled.")
            break
        elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
        if elapsed >= timeout_s:
            log(
                f"Live localization: timeout after {timeout_s}s "
                f"(have keys {list(node.get_object_map().keys())})"
            )
            break
    result = node.get_object_map()
    executor.remove_node(node)
    node.destroy_node()
    return result


def merge_object_maps(
    yaml_map: Dict[str, Any],
    live_map: Dict[str, Dict[str, List[float]]],
    goal_regions: Dict[str, str],
) -> Dict[str, Dict[str, List[float]]]:
    """YAML poses as fallback; live estimates override when present."""
    merged: Dict[str, Dict[str, List[float]]] = {}
    for label in goal_regions:
        if label in live_map:
            merged[label] = live_map[label]
        elif label in yaml_map:
            merged[label] = yaml_map[label]
    return merged
