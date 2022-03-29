from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    return LaunchDescription([
        # Node(package='joy', executable='joy_node', output='screen'),
        # Node(package='tello_driver', executable='tello_joy_main', output='screen'),
        # Node(
        #     package='tello_driver', 
        #     executable='tello_keyboard.py', output='screen', 
        #     remappings=[
        #         ('/drone1/cmd_vel', '/cmd_vel'),
        #         ('/drone1/tello_action', '/tello_action'),
        #         ] 
        #     ),
        # Node(
        #     package='fiducial_vlam', 
        #     executable='landing_pid.py', output='screen', 
        #     ),
        Node(
            package='fiducial_vlam', 
            executable='aruco_marker_pose_estimation_tf.py', output='screen', 
            remappings=[
                ("/drone1/image_raw", '/image_raw'),
                ] 
            ),
        Node(package='tello_driver', executable='tello_driver_main', output='screen'),
    ])
