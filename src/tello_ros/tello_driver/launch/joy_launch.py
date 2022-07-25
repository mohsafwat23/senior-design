from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fiducial_vlam', 
            executable='camera_pose.py', output='screen',
            remappings=[
                ("/drone1/image_raw", '/image_raw'),
                ] 
        ),

        Node(package='tello_driver', executable='tello_driver_main', output='screen'),
        # Node(
        #     package='fiducial_vlam', 
        #     executable='basic_image_publisher.py', output='screen', 
        # ),
    ])
