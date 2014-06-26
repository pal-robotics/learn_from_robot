learn_from_robot
================

A package to activate and deactivate current on joints to create poses of the robot

For using this package the robot must be running:
```
roslaunch reem_controller_configuration current_limit_controller.launch
```

Which needs the params:
```
rosparam load `rospack find reem_controller_configuration`/config/current_limit_controller.yaml
```

Launch this package with:
```
roslaunch learn_from_robot activation_manager.launch
```

This will launch:

activation_manager.py

Which listens to messages from the topic '/active_joints' that contain the name of the joint (or joint group) and wether to 
give current to it or not.

joint_error_states.py

Which acts like '/joint_states' topic but publishes the current position error in each joint.

actual_pose_sender.py

Which sends the robot joints to where they are being moved so when the activation of the current
of a joint happens it will stay there (and not jump fast to the last position). It reads
joint_error_states to know when to send commands to the motors.

Once everything is launched you can use

poses_grabber.py

To get play_motion motions of each pose you want.

