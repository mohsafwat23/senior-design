"""Simulate a Tello drone"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ns = 'drone1'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')
    robot_sdf_path = os.path.join(get_package_share_directory('tello_gazebo'), 'models','moving_platform_with_aruco', 'model.sdf')

    return LaunchDescription([
        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # Spawn tello.urdf
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', '1', '0.7']),
        
        # Spawn tello.urdf
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
             arguments=[robot_sdf_path, '1', '1', '1', '4']),
        
        # Publish mobrob movement
        Node(package='tello_gazebo', executable='mobrob_node', output='screen'),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),
        
        Node(package='fiducial_vlam', executable='sim_pid2.py'),

     # #    Node(package='fiducial_vlam', executable='drone_ekf.py'),

         Node(package='fiducial_vlam', executable='aruco_marker_pose_estimation_tf.py'),
          #    Node(package='fiducial_vlam', executable='prevSemester.py', remappings=[
          #       ("/cmd_vel", '/drone1/cmd_vel'),
          #       ("/tello_action", '/drone1/tello_action'),
          #       ],
          #        parameters = [{
          #           "aruco_dictionary_name_1": "DICT_ARUCO_ORIGINAL",
          #           "aruco_dictionary_name_2": "DICT_6X6_50",
          #           "aruco_marker_side_length_1": 0.296,
          #           "aruco_marker_side_length_2": 0.1355,
          #           "camera_calibration_parameters": "Simulation",
          #           "image_topic": "/drone1/image_raw"
          #        }
          #        ],
          #   ),

        # Node(package='fiducial_vlam', executable='drone_ekf.py'),

     #    # Joystick driver, generates /namespace/joy messages
     #    Node(package='joy', executable='joy_node', output='screen',
     #         namespace=ns),

     #    # Joystick controller, generates /namespace/cmd_vel messages
     #    Node(package='tello_driver', executable='tello_joy_main', output='screen',
     #         namespace=ns),
    ])
