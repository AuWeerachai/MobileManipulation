import rclpy, time
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from hello_helpers.hello_misc import HelloNode
import moveit2_utils
import pdb


# stretch_robot_home.py
# Make sure to run `ros2 launch stretch_core stretch_driver.launch.py`

class MoveMe(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        self.main('move_me', 'move_me', wait_for_first_pointcloud=False)
        self.stow_the_robot()

        self.planning_group = 'mobile_base_arm'
        self.moveit, self.moveit_plan, self.planning_params = moveit2_utils.setup_moveit(self.planning_group)

        self.goal_state = RobotState(self.moveit.get_robot_model())
        self.stow_positions = [0.0, 0.0, 0.0, 
                    self.get_joint_pos('joint_lift'), self.get_joint_pos('joint_arm_l3'), 
                    self.get_joint_pos('joint_arm_l2'), self.get_joint_pos('joint_arm_l1'), self.get_joint_pos('joint_arm_l0'), 
                    self.get_joint_pos('joint_wrist_yaw'), self.get_joint_pos('joint_wrist_pitch'), self.get_joint_pos('joint_wrist_roll')]
        self.goal_state.set_joint_group_positions(self.planning_group, self.stow_positions)

        self.i = 1


    def moveme(self):
        while  self.i <= 4:
            print(f'--- Planning Step {self.i} ---')

            # Example goal state: demonstrates commanding the robot to move forward by 0.3 meters
            # We pass in the current joint position as the goal for each joint in the arm + wrist since we don't want them to move
            # Ordering: [x, y, theta, lift, arm/4, arm/4, arm/4, arm/4, yaw, pitch, roll]
            # For driving the base: the positive x-axis is pointing out of the front of the robot (the flat side of the base). 
            # Positive y-axis is on the left of the robot (opposite direction the arm is facing).
            
            self.moveit_plan.set_start_state_to_current_state()
            current_pos = self.goal_state.get_joint_group_positions(self.planning_group)
            # breakpoint()

            if self.i == 1:
                target_pos = np.array(current_pos) + np.array([0.2, 0.2, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                # target_pos = np.array(current_pos) + np.array([0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            elif self.i == 2:
                target_pos = np.array(current_pos) + np.array([0.0, -0.2, -np.radians(90), 
                                                                0.0, 0.1, 0.00, 0.00, 0.00,
                                                                0.0, 0.0, 0.0])
            elif self.i == 3:
                target_pos = np.array(current_pos) + np.array([-0.1, -0.4, -np.radians(90), 
                                                                0.0, 0.0, 0.0, 0.0, 0.0,
                                                                np.radians(45), np.radians(45), np.radians(45)])
            elif self.i == 4:
                # target_pos = self.stow_positions
                target_pos = np.array(current_pos) + np.array([-0.2, 0.0, -np.radians(180), 
                                                                0.0, 0.0, 0.0, 0.0, 0.0,
                                                                0.0, 0.0, 0.0])
            
            # TODO: Your code will likely go here. Note that I gave you a for loop already, which you can edit and use.
            # For base motion (x, y, theta) each goal state should be defined according to the robot's current position (or its previous goal).
            # Reminder: You can use the RViz GUI for MoveIt 2 to get a better intuition for what these goal positions should be.

            self.goal_state.set_joint_group_positions(self.planning_group, target_pos)

            # self.moveit_plan.set_goal_state(robot_state=self.goal_state, goal_position_tolerance=0.05, goal_orientation_tolerance=0.1)
            self.moveit_plan.set_goal_state(robot_state=self.goal_state)
            
            plan = self.moveit_plan.plan(parameters=self.planning_params)
            # if plan.trajectory == None:
            #     break
            print(plan.trajectory.get_robot_trajectory_msg())
    
            self.execute_plan(plan)
            time.sleep(3)

            self.i += 1

    def execute_plan(self, plan):
        # NOTE: You don't need to edit this function
        processor = moveit2_utils.TrajectoryProcessor()
        segments = processor.process_trajectory(plan, self.joint_state)

        for i, goal_traj in enumerate(segments):
            # print(goal_traj)
            # time.sleep(2.0)
            self.get_logger().info(f"Executing segment {i+1}/{len(segments)} (Mode: {self._detect_mode(goal_traj)})")
            
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = goal_traj
            
            future = self.trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error(f"Segment {i+1} rejected!")
                break
            
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            res = res_future.result()
            
            if res.result.error_code != res.result.SUCCESSFUL:
                self.get_logger().error(f"Segment {i+1} failed! Code: {res.result.error_code}")
                break

    def get_joint_pos(self, joint_name):
        return self.joint_state.position[self.joint_state.name.index(joint_name)]
        
    def _detect_mode(self, traj):
        if 'translate_mobile_base' in traj.joint_names: return 'TRANSLATE'
        if 'rotate_mobile_base' in traj.joint_names: return 'ROTATE'
        return 'ARM_ONLY'

if __name__ == '__main__':
    m = MoveMe()
    time.sleep(3)
    # breakpoint()
    m.moveme()