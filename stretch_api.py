import time
import stretch_body.robot
import numpy as np

#start and reset
robot = stretch_body.robot.Robot()
robot.startup()
robot.stow()
robot.push_command()
robot.wait_command()


#move lift/arm
robot.arm.move_to(0.5) # Move telescoping arm
robot.lift.move_to(1.1) # Move lift
robot.push_command()
robot.wait_command()

#move wrist
robot.end_of_arm.move_to('wrist_roll', np.radians(30))
robot.push_command()
robot.wait_command()
robot.end_of_arm.move_to('wrist_pitch', np.radians(30))
robot.push_command()
robot.wait_command()
robot.end_of_arm.move_to('wrist_yaw', np.radians(30))
robot.push_command()
robot.wait_command()
robot.end_of_arm.move_to('stretch_gripper', 50)
robot.push_command()
robot.wait_command()
robot.end_of_arm.move_to('stretch_gripper', 0)
robot.push_command()
robot.wait_command()

#move camera
robot.head.move_by('head_pan', np.radians(45))
robot.push_command()
robot.wait_command()
robot.head.move_by('head_tilt', np.radians(45))
robot.push_command()
robot.wait_command()

#robot stow
robot.stow()
robot.push_command()
robot.wait_command()

#move base
robot.base.translate_by(0.5) # Move robot base 0.5 meters forward
robot.push_command()
robot.wait_command()
robot.base.rotate_by(np.radians(180))
robot.push_command()
robot.wait_command()
robot.base.translate_by(0.5) # Move robot base 0.5 meters forward
robot.push_command()
robot.wait_command()

robot.stop()
