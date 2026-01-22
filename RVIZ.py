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
robot.arm.move_to(0.2) # Move telescoping arm
robot.lift.move_to(0.6) # Move lift
robot.push_command()
robot.wait_command()

#move wrist
robot.end_of_arm.move_to('wrist_pitch', np.radians(0))
robot.push_command()
robot.wait_command()
robot.end_of_arm.move_to('wrist_yaw', np.radians(90))
robot.push_command()
robot.wait_command()



