"""
Stretch Robot Inverse Kinematics (IK) Module.

This module provides IK computation and control for the Hello Robot Stretch.
It builds a kinematic chain from URDF with virtual base joints for mobile base control.

Launch command:
    python ik_ros.py

Dependencies:
    python3 -m pip install --upgrade ikpy graphviz urchin networkx
"""

import os
import importlib.resources as importlib_resources

import numpy as np
import ikpy.chain
import ikpy.urdf.utils
import ikpy.utils.geometry
import urchin as urdfpy

import hello_helpers.hello_misc as hm

# Constants
MOBILE_BASE_EFFORT_LIMIT = 100.0
MOBILE_BASE_VELOCITY_LIMIT = 1.0
MOBILE_BASE_TRANSLATION_LIMIT = 1.0
IK_POSITION_TOLERANCE = 1e-2
DEFAULT_JOINT_MOVE_DURATION = 3.0
DEFAULT_BASE_MOVE_DURATION = 4.0
TEMP_URDF_DIR = '/tmp/iktutorial'
MODIFIED_URDF_PATH = '/tmp/iktutorial/stretch.urdf'


class StretchIKDemo(hm.HelloNode):
    """
    Stretch robot IK demonstration node.
    
    This class extends HelloNode to provide inverse kinematics functionality
    for the Stretch robot, allowing end-effector positioning through IK solving.
    """
    
    def __init__(self):
        """Initialize the IK demo node."""
        hm.HelloNode.__init__(self)
        self.chain = None

    def _clamp_joint_value(self, joint_name, value):
        """
        Clamp a joint value to its defined limits.
        
        Args:
            joint_name (str): Name of the joint in the kinematic chain.
            value (float): Desired joint value.
            
        Returns:
            float: Clamped value within joint bounds.
        """
        link_names = [link.name for link in self.chain.links]
        link_index = link_names.index(joint_name)
        lower_bound, upper_bound = self.chain.links[link_index].bounds
        return max(lower_bound, min(value, upper_bound))

    def setup_ik_chain(self):
        """
        Set up the inverse kinematics chain from URDF.
        
        Loads the Stretch robot URDF, removes unnecessary links/joints,
        and adds virtual base joints to enable mobile base positioning in IK.
        
        Returns:
            ikpy.chain.Chain: Configured kinematic chain with active links marked.
        """
        # Load URDF file
        package_path = str(importlib_resources.files('stretch_urdf'))
        urdf_file_path = package_path + '/SE3/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.urdf'

        # Load and copy URDF for modification
        original_urdf = urdfpy.URDF.load(urdf_file_path)
        modified_urdf = original_urdf.copy()

        # Remove unnecessary links (cameras, sensors, wheels, etc.)
        # Kept links: base_link, link_mast, link_lift, link_arm_l*, link_wrist_*, link_gripper_s3_body, link_grasp_center
        link_names_to_remove = [
            'link_right_wheel', 'link_left_wheel', 'caster_link', 'link_head', 'link_head_pan', 
            'link_head_tilt', 'link_aruco_right_base', 'link_aruco_left_base', 'link_aruco_shoulder', 
            'link_aruco_top_wrist', 'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 
            'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 'camera_infra1_optical_frame', 
            'camera_infra2_frame', 'camera_infra2_optical_frame', 'camera_color_frame', 'camera_color_optical_frame', 
            'camera_accel_frame', 'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 
            'gripper_camera_bottom_screw_frame', 'gripper_camera_link', 'gripper_camera_depth_frame', 
            'gripper_camera_depth_optical_frame', 'gripper_camera_infra1_frame', 'gripper_camera_infra1_optical_frame', 
            'gripper_camera_infra2_frame', 'gripper_camera_infra2_optical_frame', 'gripper_camera_color_frame', 
            'gripper_camera_color_optical_frame', 'laser', 'base_imu', 'respeaker_base', 'link_wrist_quick_connect', 
            'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_aruco_fingertip_right', 
            'link_gripper_finger_left', 'link_gripper_fingertip_left', 'link_aruco_fingertip_left', 
            'link_aruco_d405', 'link_head_nav_cam'
        ]
        links_to_remove = [link for link in modified_urdf._links if link.name in link_names_to_remove]
        for link in links_to_remove:
            modified_urdf._links.remove(link)

        # Remove unnecessary joints corresponding to removed links
        # Kept joints: joint_mast, joint_lift, joint_arm_l*, joint_wrist_*, joint_gripper_s3_body, joint_grasp_center
        joint_names_to_remove = [
            'joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_head', 'joint_head_pan', 
            'joint_head_tilt', 'joint_aruco_right_base', 'joint_aruco_left_base', 'joint_aruco_shoulder', 
            'joint_aruco_top_wrist', 'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 
            'camera_depth_joint', 'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 
            'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 'camera_color_optical_joint', 
            'camera_accel_joint', 'camera_accel_optical_joint', 'camera_gyro_joint', 'camera_gyro_optical_joint', 
            'gripper_camera_joint', 'gripper_camera_link_joint', 'gripper_camera_depth_joint', 
            'gripper_camera_depth_optical_joint', 'gripper_camera_infra1_joint', 'gripper_camera_infra1_optical_joint', 
            'gripper_camera_infra2_joint', 'gripper_camera_infra2_optical_joint', 'gripper_camera_color_joint', 
            'gripper_camera_color_optical_joint', 'joint_laser', 'joint_base_imu', 'joint_respeaker', 
            'joint_wrist_quick_connect', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 
            'joint_aruco_fingertip_right', 'joint_gripper_finger_left', 'joint_gripper_fingertip_left', 
            'joint_aruco_fingertip_left', 'joint_aruco_d405', 'joint_head_nav_cam'
        ]
        joints_to_remove = [joint for joint in modified_urdf._joints if joint.name in joint_names_to_remove]
        for joint in joints_to_remove:
            modified_urdf._joints.remove(joint)

        # Add virtual base joints for mobile base control
        # Joint 1: Base rotation around Z-axis (yaw)
        joint_base_rotation = urdfpy.Joint(
            name='joint_base_rotation',
            parent='base_link',
            child='link_base_rotation',
            joint_type='revolute',
            axis=np.array([0.0, 0.0, 1.0]),
            origin=np.eye(4, dtype=np.float64),
            limit=urdfpy.JointLimit(
                effort=MOBILE_BASE_EFFORT_LIMIT,
                velocity=MOBILE_BASE_VELOCITY_LIMIT,
                lower=-np.pi,
                upper=np.pi
            )
        )
        modified_urdf._joints.append(joint_base_rotation)

        link_base_rotation = urdfpy.Link(
            name='link_base_rotation',
            inertial=None,
            visuals=None,
            collisions=None
        )
        modified_urdf._links.append(link_base_rotation)

        # Joint 2: Base translation along X-axis (forward/backward)
        joint_base_translation = urdfpy.Joint(
            name='joint_base_translation',
            parent='link_base_rotation',
            child='link_base_translation',
            joint_type='prismatic',
            axis=np.array([1.0, 0.0, 0.0]),
            origin=np.eye(4, dtype=np.float64),
            limit=urdfpy.JointLimit(
                effort=MOBILE_BASE_EFFORT_LIMIT,
                velocity=MOBILE_BASE_VELOCITY_LIMIT,
                lower=-MOBILE_BASE_TRANSLATION_LIMIT,
                upper=MOBILE_BASE_TRANSLATION_LIMIT
            )
        )
        modified_urdf._joints.append(joint_base_translation)
        
        link_base_translation = urdfpy.Link(
            name='link_base_translation',
            inertial=None,
            visuals=None,
            collisions=None
        )
        modified_urdf._links.append(link_base_translation)


        # Update kinematic chain: connect mast to virtual base translation link
        for joint in modified_urdf._joints:
            if joint.name == 'joint_mast':
                joint.parent = 'link_base_translation'

        # Save modified URDF to temporary file
        os.makedirs(TEMP_URDF_DIR, exist_ok=True)
        modified_urdf.save(MODIFIED_URDF_PATH)
        
        # Define which joints are active in IK (True) vs fixed (False)
        active_links_mask = [
            False,  # 0: base_link (fixed)
            True,   # 1: joint_base_rotation (revolute - mobile base yaw)
            True,   # 2: joint_base_translation (prismatic - mobile base x)
            False,  # 3: joint_mast (fixed)
            True,   # 4: joint_lift (prismatic - vertical lift)
            False,  # 5: joint_arm_l4 (fixed)
            True,   # 6: joint_arm_l3 (prismatic - arm extension segment 3)
            True,   # 7: joint_arm_l2 (prismatic - arm extension segment 2)
            True,   # 8: joint_arm_l1 (prismatic - arm extension segment 1)
            True,   # 9: joint_arm_l0 (prismatic - arm extension segment 0)
            True,   # 10: joint_wrist_yaw (revolute - wrist yaw)
            False,  # 11: joint_wrist_yaw_bottom (fixed)
            True,   # 12: joint_wrist_pitch (revolute - wrist pitch)
            True,   # 13: joint_wrist_roll (revolute - wrist roll)
            False,  # 14: joint_gripper_s3_body (fixed)
            False,  # 15: joint_grasp_center (fixed - end effector)
        ]
        
        # Build kinematic chain from modified URDF
        chain = ikpy.chain.Chain.from_urdf_file(MODIFIED_URDF_PATH, active_links_mask=active_links_mask)
        
        # Print chain information for debugging
        print("Kinematic chain links:")
        for link in chain.links:
            print(f"  * {link.name} (Type: {link.joint_type})")
        
        return chain

    def get_current_configuration(self):
        """
        Get current robot joint configuration from ROS state.
        
        Returns:
            list: Configuration vector matching kinematic chain structure.
                  Contains positions for all 16 links (active and fixed).
        """
        # Extract joint positions from ROS joint state message
        joint_positions = self.joint_state.position
        joint_names = self.joint_state.name
        
        def get_joint_position(name):
            """Helper to get position of a named joint, returns 0.0 if not found."""
            if name in joint_names:
                index = list(joint_names).index(name)
                return joint_positions[index]
            return 0.0

        # Virtual base joints (not actual hardware joints)
        base_rotation = 0.0
        base_translation = 0.0
        
        # Hardware joint positions (clamped to limits)
        lift_position = self._clamp_joint_value('joint_lift', get_joint_position('joint_lift'))
        arm_extension = self._clamp_joint_value('joint_arm_l0', get_joint_position('joint_arm_l0'))
        wrist_yaw = self._clamp_joint_value('joint_wrist_yaw', get_joint_position('joint_wrist_yaw'))
        wrist_pitch = self._clamp_joint_value('joint_wrist_pitch', get_joint_position('joint_wrist_pitch'))
        wrist_roll = self._clamp_joint_value('joint_wrist_roll', get_joint_position('joint_wrist_roll'))
        
        # Build configuration vector for all 16 links in chain
        # Order: base_link, link_base_rotation, link_base_translation, link_mast, link_lift,
        #        link_arm_l4, link_arm_l3, link_arm_l2, link_arm_l1, link_arm_l0,
        #        link_wrist_yaw, link_wrist_yaw_bottom, link_wrist_pitch, link_wrist_roll,
        #        link_gripper_s3_body, link_grasp_center
        return [
            0.0,              # 0: base_link (fixed origin)
            base_rotation,    # 1: virtual base rotation
            base_translation, # 2: virtual base translation
            0.0,              # 3: mast (fixed)
            lift_position,    # 4: lift joint
            0.0,              # 5: arm_l4 (fixed)
            arm_extension,    # 6-9: arm extension (4 prismatic segments with same value)
            arm_extension,
            arm_extension,
            arm_extension,
            wrist_yaw,        # 10: wrist yaw
            0.0,              # 11: wrist_yaw_bottom (fixed)
            wrist_pitch,      # 12: wrist pitch
            wrist_roll,       # 13: wrist roll
            0.0,              # 14: gripper body (fixed)
            0.0,              # 15: grasp center (fixed)
        ]
    
    def move_to_configuration(self, configuration):
        """
        Move robot to a specified joint configuration.
        
        Args:
            configuration (list): Joint configuration vector from IK solution.
        """
        # Extract joint values from configuration vector
        base_rotation = configuration[1]
        base_translation = configuration[2]
        lift_position = configuration[4]
        arm_extension = configuration[6] + configuration[7] + configuration[8] + configuration[9]
        wrist_yaw = configuration[10]
        wrist_pitch = configuration[12]
        wrist_roll = configuration[13]
        
        # Move arm and wrist joints
        self.move_to_pose(
            {
                'joint_lift': lift_position,
                'joint_arm': arm_extension,
                'joint_wrist_yaw': wrist_yaw,
                'joint_wrist_pitch': wrist_pitch,
                'joint_wrist_roll': wrist_roll,
            },
            blocking=True,
            duration=DEFAULT_JOINT_MOVE_DURATION
        )
        
        # Move mobile base (rotation then translation)
        self.move_to_pose(
            {'rotate_mobile_base': base_rotation},
            blocking=True,
            duration=DEFAULT_BASE_MOVE_DURATION
        )
        self.move_to_pose(
            {'translate_mobile_base': base_translation},
            blocking=True,
            duration=DEFAULT_BASE_MOVE_DURATION
        )


    def move_to_grasp_goal(self, target_position, target_orientation):
        """
        Compute inverse kinematics and move to target grasp pose.
        
        Args:
            target_position (array-like): Target end-effector position [x, y, z].
            target_orientation (np.ndarray): Target orientation matrix (3x3 or 4x4).
            
        Returns:
            list or None: Joint configuration if successful, None if IK fails.
        """
        # Get current configuration as initial guess for IK
        initial_configuration = self.get_current_configuration()
        
        # Solve inverse kinematics
        solution_configuration = self.chain.inverse_kinematics(
            target_position,
            target_orientation,
            orientation_mode='all',
            initial_position=initial_configuration
        )
        print(f"IK Solution: {solution_configuration}")

        # Verify solution accuracy
        end_effector_pose = self.chain.forward_kinematics(solution_configuration)
        position_error = np.linalg.norm(end_effector_pose[:3, 3] - target_position)
        
        if not np.isclose(position_error, 0.0, atol=IK_POSITION_TOLERANCE):
            print(f"IK solution failed: position error = {position_error:.4f}m")
            return None
        
        # Move robot to solution configuration
        self.move_to_configuration(solution_configuration)
        return solution_configuration

    def get_current_grasp_pose(self):
        """
        Get the current end-effector pose using forward kinematics.
        
        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix of end-effector.
        """
        current_configuration = self.get_current_configuration()
        return self.chain.forward_kinematics(current_configuration)

    def main(self):
        """
        Main execution function for the IK demo.
        
        Initializes the ROS node, sets up the kinematic chain,
        and executes demonstration motions.
        """
        # Initialize ROS node
        hm.HelloNode.main(
            self,
            node_name='stretch_ik_demo',
            node_topic_namespace='stretch_ik_demo',
            wait_for_first_pointcloud=False
        )
        
        # Stow robot to known configuration
        self.stow_the_robot()
        
        # Set up kinematic chain for IK
        self.chain = self.setup_ik_chain()

        # Demo: Move to target pose
        print("------ Part 1.1: Single Target Pose ------")
        target_poses = [
            ([0.6, 0.0, 0.3], ikpy.utils.geometry.rpy_matrix(0, 0, 0)),
        ]
        for pose_index, (position, orientation) in enumerate(target_poses):
            print(f"Moving to pose {pose_index + 1}: position={position}")
            self.move_to_grasp_goal(position, orientation)

        # Demo: Multiple poses (uncomment to enable)
        # print("------ Part 1.2: Multiple Target Poses ------")
        # target_poses = [
        #     ([0.6, 0.0, 0.3], ikpy.utils.geometry.rpy_matrix(0, 0, 0)),
        #     ([0.0, 0.6, 0.3], ikpy.utils.geometry.rpy_matrix(0, 0, 0)),
        #     ([0.0, -0.6, 0.3], ikpy.utils.geometry.rpy_matrix(0, 0, 0)),
        #     ([0.6, 0.0, 0.3], ikpy.utils.geometry.rpy_matrix(0, 0, 0)),
        # ]
        # for pose_index, (position, orientation) in enumerate(target_poses):
        #     print(f"Moving to pose {pose_index + 1}: position={position}")
        #     self.move_to_grasp_goal(position, orientation)

        # Stop robot motion
        self.stop_the_robot()


if __name__ == '__main__':
    node = StretchIKDemo()
    node.main()