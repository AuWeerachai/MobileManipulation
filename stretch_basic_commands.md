• Username: hello-robot
• Password: 16762cmu!
• stretch-se3-3159:
• SSH: ssh -X hello-robot@stretch3159.wi .local.cmu.edu
• Anydesk: 1943010008
• stretch-se3-3160:
• SSH: ssh -X hello-robot@stret

# Important Commands
stretch_robot_home.py
stretch_robot_stow.py

ros2 launch stretch_core stretch_driver.launch.py

Stretch API:
robot.stow()
ROS 2 API:
https://docs.hello-robot.com/0.3/ros2/intro_to_hellonode/
node.stow_the_robot()
node.move_to_pose({'joint_arm': 0.7}, blocking=True)
node.move_to_pose({‘joint_wrist_yaw':
node.joint_state.position[node.joint_state.name.index(‘joint_wrist_yaw')] +
np.radians(45)}, blocking=True)