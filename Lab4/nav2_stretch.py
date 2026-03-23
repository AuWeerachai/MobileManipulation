from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


def set_pose(pose, pose_msg: PoseStamped, frame_id: str, stamp_node: Node) -> None:
    pose_msg.header.frame_id = frame_id
    pose_msg.header.stamp = stamp_node.get_clock().now().to_msg()
    pose_msg.pose.position.x = pose[0]
    pose_msg.pose.position.y = pose[1]
    pose_msg.pose.position.z = pose[2]
    pose_msg.pose.orientation.x = pose[3]
    pose_msg.pose.orientation.y = pose[4]
    pose_msg.pose.orientation.z = pose[5]
    pose_msg.pose.orientation.w = pose[6]


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Robot starting pose in the map
    start_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    # Go to 3 different points, then come back to start
    route = [
        [4.12, 0.00692, -0.00534, 0.0, 0.0, 0.7071, 0.7071],         # point 1
        [1.34, -3.45, -0.00134, 0.0, 0.0, 0.7071, 0.7071],     # point 2
        [2.99, -14.5, -0.00134, 0.0, 0.0, 0.7071, 0.7071],           # point 3
        [7.39, -9.82, -0.00134, 0.0, 0.0, 0.7071, 0.7071],                   # point4
    ]

    initial_pose = PoseStamped()
    set_pose(start_pose, initial_pose, "map", navigator)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    route_poses = []
    for pt in route:
        pose = PoseStamped()
        set_pose(pt, pose, "map", navigator)
        route_poses.append(pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(route_poses)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            navigator.get_logger().info(
                f"Executing waypoint: {feedback.current_waypoint + 1}/{len(route_poses)}"
            )

            if navigator.get_clock().now() - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info("Route complete!")
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info("Mission cancelled. Mission duration exceeded 10 mins.")
    elif result == TaskResult.FAILED:
        navigator.get_logger().info("Mission failed.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()