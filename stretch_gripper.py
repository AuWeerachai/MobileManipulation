#!/usr/bin/env python3
import math
import hello_helpers.hello_misc as hm
import time


class SangStretchNode(hm.HelloNode):
    
    def __init__(self):
        super().__init__()

    def main(self):
        
        hm.HelloNode.main(self, 'my_stretch_node', 'my_stretch_node', wait_for_first_pointcloud=False)

        self.move_to_pose({'joint_arm': 0.5, 'joint_lift': 1.1}, blocking=True)


  
        self.move_to_pose({'joint_gripper_finger_left': 50.0, 'joint_gripper_finger_right': 50.0}, blocking=True)



        self.stop_the_robot()  # convenience stop service :contentReference[oaicite:6]{index=6}


if __name__ == '__main__':
    node = SangStretchNode()
    node.main()
