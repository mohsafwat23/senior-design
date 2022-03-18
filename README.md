# ME 352 senior-design, the University of Wisoncin-Madison
Landing a quadrotor on a moving platform

# Special thanks:
To Clyde McQueen and Peter Mullen for making the Ros2 Tello Driver:
https://github.com/clydemcqueen/tello_ros

# My contribution:
## PID Landing algorithm:
tello_ros_ws/src/fiducial_vlam/fiducial_vlam/scripts/landing_pid.py
## Aruco Marker pose estimation:
tello_ros_ws/src/fiducial_vlam/fiducial_vlam/scripts/aruco_marker_pose_estimation_tf.py
## Keyboard teleop:
tello_ros_ws/src/tello_ros/tello_driver/scripts/tello_keyboard.py
## Gazebo Models:
tello_ros_ws/src/tello_ros/tello_gazebo/models/moving_platform_with_aruco
tello_ros_ws/src/tello_ros/tello_gazebo/models/stationary_platform_with_aruco
tello_ros_ws/src/tello_ros/tello_gazebo/models/me439_robot
## Launch files for simulation:
tello_ros_ws/src/tello_ros/tello_gazebo/launch/drone_robot_launch.py
tello_ros_ws/src/tello_ros/tello_gazebo/launch/drone_launch.py
## Mobile Robot movement:
tello_ros_ws/src/tello_ros/tello_gazebo/src/mobrob_node.cpp








