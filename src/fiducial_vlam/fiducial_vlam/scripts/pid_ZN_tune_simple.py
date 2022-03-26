#!/usr/bin/env python3
...
"""
author: Mohamed Safwat
email: mohamedmohabsafwat@gmail.com
"""
import time
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Twist # Handles TransformStamped message
from nav_msgs.msg import Path, Odometry
import numpy as np
from tf2_ros import TFMessage
from tello_msgs.srv import TelloAction
import math
import csv



class DroneSimpleTune(Node):
    """
    Create an LandingNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('simple_tune')

        ### Stationary Platform Location ###
        self.platform_x = 2.423940
        self.platform_y = 3.560530
        self.platform_z = 2.0

        ### Drone Location ###
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0

        self.time_data = 0.0

        self.x_desired = 0.0 #positive value
        self.integralX = 0
        self.integralY = 0
        self.integralZ = 0
        self.kp_xyz = np.array([0.5, 0.5, 0.5])
        self.kd_xyz = np.array([0.2, 0.2, 0.2])
        self.ki_xyz = np.array([0.0, 0.0, 0.0])
        self.error_prevX = 0
        self.error_prevY = 0
        self.error_prevZ = 0

        self.landing_command_pub = self.create_publisher(Twist,'/drone1/cmd_vel', 1)
        self.drone_action_client = self.create_client(TelloAction, '/drone1/tello_action')

        self.t_old = time.time()
        print("Landing node initialized")

        self.drone_odom_subscription = self.create_subscription(
        Odometry, 
        "/drone/p3d_odom", 
        self.drone_pose_subscriber, 
        1)
        self.drone_odom_subscription # prevent unused variable warning

        self.timer_period = 0.025  # seconds

        # Create the timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        ### data plotter ###
        self.f = open("Data/kp{}_kd{}_ki{}.csv".format(self.kp_xyz[0], self.kd_xyz[0],self.ki_xyz[0]), "a+")
        self.writer = csv.writer(self.f, delimiter=',')
    def timer_callback(self):
        """
        Callback function.
        This function gets called every time a message is received.
        """

        #dt = t_stamp - self.t_old
         
        dt = self.timer_period
        self.time_data = self.time_data + dt
        distance = np.sqrt((self.drone_x - self.platform_x)**2 + (self.drone_y - self.platform_y)**2 + 
        (self.drone_z - self.platform_z)**2)
        print(self.drone_x, self.drone_y, self.drone_z)
        """
        The pose distances are defined as follows:
        x: left/right
        y: up/down
        z: forward
        relative to the AruCo marker
        """
        #The X sign is flipped because the drone is facing the opposite direction of the camera
        errorX = (self.platform_x -self.x_desired) - self.drone_x
        #The Y sign is flipped because the drone is facing the opposite direction 
        errorY = self.platform_y - self.drone_y 
        errorZ = self.platform_z - self.drone_z
        
        integralX = self.integralX + (errorX * dt)
        integralY = self.integralY + (errorY * dt)
        integralZ = self.integralZ + (errorZ * dt)
        derivativeX = (errorX - self.error_prevX)/dt
        derivativeY = (errorY - self.error_prevY)/dt
        derivativeZ = (errorZ - self.error_prevZ)/dt
        speed_FB = self.kp_xyz[0] * errorX + self.kd_xyz[0] * derivativeX  + self.ki_xyz[0] * integralX
        speed_LR = self.kp_xyz[1] * errorY + self.kd_xyz[1] * derivativeY  + self.ki_xyz[1] * integralY
        speed_UD = self.kp_xyz[2] * errorZ + self.kd_xyz[2] * derivativeZ  + self.ki_xyz[2] * integralZ
        speed_LR = float(np.clip(speed_LR, -0.4, 0.4))
        speed_UD = float(np.clip(speed_UD, -0.4, 0.4))
        speed_FB = float(np.clip(speed_FB, -0.80, 0.80))
        """
        The speed values are defined as follows:
        twist.linear.x: forward/backward speed of the drone (m/s) 
        twist.linear.y: left/right speed of the drone (m/s)
        twist.linear.z: up/down speed of the drone (m/s)
        """
        if errorX == 0.0:
            speed_FB = 0.0
        if errorY == 0.0:
            speed_LR = 0.0
        if errorZ == 0.0:
            speed_UD = 0.0
        twist = Twist()
        twist.linear.x = speed_FB
        twist.linear.y = speed_LR
        twist.linear.z = speed_UD
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.landing_command_pub.publish(twist)
        # if distance > 0.05 and distance < 0.4:
        #     request = TelloAction.Request()
        #     request.cmd = "land"
        #     self.drone_action_client.call_async(request)
        #     distance = 1.0
        #self.t_old = t_stamp
        self.error_prevX = errorX
        self.error_prevY = errorY
        self.error_prevZ = errorZ
        self.integralX = integralX
        self.integralY = integralY
        self.integralZ = integralZ

        self.writer.writerow([self.time_data, errorX])
        self.f.flush()


    def drone_pose_subscriber(self,msg):
        """This callback function gets the position of the drone"""
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z


def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  simple_tune_node = DroneSimpleTune()
   
  # Spin the node so the callback function is called.
  rclpy.spin(simple_tune_node)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  simple_tune_node.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
    main()