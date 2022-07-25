#!/usr/bin/env python3
...
"""
author: Mohamed Safwat
email: mohamedmohabsafwat@gmail.com
"""
import time
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Twist, Vector3 # Handles TransformStamped message
from nav_msgs.msg import Path, Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from tello_msgs.srv import TelloAction
import math
import csv
from std_msgs.msg import Bool, Int16MultiArray
from sensor_msgs.msg import Imu
from tello_msgs.msg import FlightData
from transforms3d.euler import euler2quat, quat2euler

class DroneControl(Node):
    """
    Create an LandingNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('drone_control')

        self.t_old = time.time()
        self.vel_pub = self.create_publisher(Twist,'/drone1/cmd_vel', 1)
        self.twist = Twist()
        timer_period = 0.0025  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.drone_commander)

        self.action = ''

        self.joy = [0.0,0.0,0.0,0.0]

        self.vel = [0.0,0.0,0.0]

        self.angVelZ = 0.0

        self.drone_action_client = self.create_client(TelloAction, '/drone1/tello_action')



        self.gyr_sub = self.create_subscription(
        Vector3, 
        '/gyr_data', 
        self.gyr_subscriber, 
        1)

        self.gyr_subscriber # prevent unused variable warning


        self.joy_sub = self.create_subscription(
        Int16MultiArray, 
        '/joy', 
        self.joy_subscriber, 
        1)




    def drone_commander(self):
        """This function gets the drone's pose from the EKF"""
        # twist = Twist()
        # v_yaw = 0.2
        # twist.linear.x = 0.0
        # twist.linear.y = 0.0
        # twist.linear.z = 0.0
        # twist.angular.x = 0.0
        # twist.angular.y = 0.0
        # twist.angular.z = v_yaw
        if self.joy[3] == -1:
            request = TelloAction.Request()
            request.cmd = 'takeoff'
            self.drone_action_client.call_async(request)
        elif self.joy[3] == 1:
            request = TelloAction.Request()
            request.cmd = 'land'
            self.drone_action_client.call_async(request)

        self.twist.linear.x = self.vel[0]
        self.twist.linear.y = self.vel[1]
        self.twist.linear.z = float(self.vel[2])
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = self.angVelZ#/np.pi
        self.vel_pub.publish(self.twist)


    def gyr_subscriber(self,msg):
        angVelX = msg.x
        angVelY = msg.y
        self.angVelZ = msg.z#/np.pi


    def joy_subscriber(self,msg):
        """This callback function gets the imu data of the drone"""
        joyX = msg.data[0]
        joyY = -msg.data[1]
        joyZ = msg.data[2]
        joyTL = msg.data[3]
        self.joy = [joyX,joyY,joyZ, joyTL]
        self.vel = [0.0,0.0,0.0]
        self.vel[2] = self.joy[2]#*2.0

        for i in range(2):
            if abs(self.joy[i]) < 15:
                self.vel[i] = 0.0
            else:
                self.vel[i] = self.joy[i]*0.003
        
    


def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  drone_control_node = DroneControl()
   
  # Spin the node so the callback function is called.
  rclpy.spin(drone_control_node)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  drone_control_node.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
    main()