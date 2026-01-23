#!/usr/bin/env python3
import math
import hello_helpers.hello_misc as hm
import time


class SangStretchNode(hm.HelloNode):
    
    def __init__(self):
        super().__init__()

    def main(self):
        
        hm.HelloNode.main(self, 'my_stretch_node', 'my_stretch_node', wait_for_first_pointcloud=False)

        self.stow_the_robot()
        self.move_to_pose({'joint_arm': 0.5, 'joint_lift': 1.1}, blocking=True)
        
        self.move_to_pose({'joint_wrist_yaw': math.radians(45)}, blocking=True)
        self.move_to_pose({'joint_wrist_roll':  math.radians(45)}, blocking=True)
        self.move_to_pose({'joint_wrist_pitch': math.radians(45)}, blocking=True)
        

        self.move_to_pose({'joint_gripper_finger_right': math.radians(45)}, blocking=True)
        self.move_to_pose({'joint_gripper_finger_right': math.radians(0)}, blocking=True)


        # self.move_to_pose({'joint_head_tilt': math.radians(120)}, blocking=True)
        # self.move_to_pose({'joint_head_pan':  math.radians(-90)}, blocking=True)
        self.move_to_pose({'joint_head_pan': math.radians(120)}, blocking=True)
        self.move_to_pose({'joint_head_tilt': math.radians(-90)}, blocking=True)
        
   
        self.stow_the_robot()

        self.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)
        self.move_to_pose({'rotate_mobile_base': math.radians(180)}, blocking=True)
        self.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)


        self.stop_the_robot()  # convenience stop service :contentReference[oaicite:6]{index=6}


if __name__ == '__main__':
    node = SangStretchNode()
    node.main()
