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
from tf2_ros import TransformBroadcaster
from tello_msgs.srv import TelloAction
import math
import csv
from std_msgs.msg import Bool
from tello_msgs.msg import FlightData
from transforms3d.euler import euler2quat

class DroneEKF(Node):
    """
    Create an LandingNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('drone_ekf')

        self.t_old = time.time()
        print("Landing node initialized")
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel', 1)
        timer_period = 0.05  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.ekf_drone)

        self.cam_pose_sub = self.create_subscription(
        Pose, 
        '/drone/pose', 
        self.camera_pose_subscriber, 
        1)
        self.cam_pose_sub # prevent unused variable warning

        self.imu_sub = self.create_subscription(
        FlightData, 
        '/flight_data', 
        self.imu_subscriber, 
        1)
        
        self.tfbroadcaster = TransformBroadcaster(self)

        self.dt = 0.0
        self.A = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,0,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1],
        ])#np.eye(6)      #state transition matrix
        self.x_hat_k_1 = np.array([[0,0,0,0,0,0]]).T #initial state      


        self.P_k_1 = np.eye(6)     #initialize covariance matrix
        self.Q = 0.02*np.eye(6)  #process noise covariance 
        
        self.R_cam = 0.4*np.eye(3)  #measurement noise covariance
        self.R_imu = 0.4*np.eye(3)  #measurement noise covariance

        self.r2d = np.pi/180

        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0

        self.drone_roll = 0.0
        self.drone_pitch = 0.0
        self.drone_yaw = 0.0

        self.H_cam = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
        ])  #state to measurement matrix     

        self.H_imu = np.array([
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,1,0,0,1],
        ])

    def quat_to_euler(self, qx, qy, qz, qw):
        t0 = +2.0 * (qw * qx + qy * qz)
        t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
        roll_x = math.atan2(t0, t1)#*(180/np.pi) #in degrees
    
        t2 = +2.0 * (qw * qy - qz * qx)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)#*(180/np.pi) #in degrees
    
        t3 = +2.0 * (qw * qz + qx * qy)
        t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_z = math.atan2(t3, t4)#*(180/np.pi) #in degrees

        euler = np.array([roll_x, pitch_y, yaw_z])
        return euler


    def ekf_drone(self):
        """This function gets the drone's pose from the EKF"""
        twist = Twist()
        v_yaw = 0.2
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = v_yaw

        u_k_1 = np.array([[0,0,0,v_yaw]]).T
        psi = self.x_hat_k_1[5][0]
        phi = self.x_hat_k_1[3][0]
        theta = self.x_hat_k_1[4][0]

        yaw = np.array([      #input matrix
            [-np.sin(psi), np.cos(psi), 0],
            [        0,        0, 1],
            [-np.cos(psi), np.sin(psi), 0],
            ])
        
        roll = np.array([      #input matrix
            [        0, -np.cos(phi), -np.sin(phi)],
            [        0, -np.sin(phi),  np.cos(phi)],
            [        1,         0,         0],
            ])

        pitch = np.array([      #input matrix
            [        0,         1,         0],
            [ np.sin(theta),         0,  np.cos(theta)],
            [-np.cos(theta),         0,  np.sin(theta)],
            ])

        rot = yaw @ roll @ pitch
        
        B = np.array([
            [1,1,1,      0],
            [1,1,1,      0],
            [1,1,1,      0],
            [0,0,0,      0],
            [0,0,0,      0],
            [0,0,0,      1],
            ])
        B[0:3, 0:3] = rot
        B = self.dt*B
        
        x_hat_k = self.A@self.x_hat_k_1 +  B@u_k_1

        P_k = self.A@self.P_k_1@self.A.T +self.Q

        K1 = P_k@self.H_cam.T@np.linalg.inv(self.H_cam@P_k@self.H_cam.T + self.R_cam)

        P_k_new = (np.identity(6) - K1@self.H_cam)@P_k

        if self.tx == 0.0 and self.ty == 0.0 and self.tz == 0.0:
            z_k_1_cam = np.array([[self.tx, self.ty, self.tz]]).T
            x_k_new = x_hat_k + K1@(z_k_1_cam - self.H_cam@x_hat_k)
        else:
            x_k_new = x_hat_k
        K2 = P_k@self.H_imu.T@np.linalg.inv(self.H_imu@P_k@self.H_imu.T + self.R_imu)
        z_k_1_imu = np.array([[self.drone_roll, self.drone_pitch, self.drone_yaw]]).T

        x_k_new = x_k_new + K2@(z_k_1_imu - self.H_imu@x_k_new)
        P_k_new = (np.identity(6) - K2@self.H_imu)@P_k_new
        self.x_hat_k_1 = x_k_new
        self.P_k_1 = P_k_new
        self.vel_pub.publish(twist)
        #print("x_k_new:",np.shape(x_k_new),"B:",np.shape(B),"u_k_1:",np.shape(u_k_1),"z_k_1_cam:",np.shape(z_k_1_imu))
        print(x_k_new[3][0], x_k_new[4][0], x_k_new[5][0])
        quat = euler2quat(x_k_new[3][0], x_k_new[4][0], x_k_new[5][0])
        quat = euler2quat(x_k_new[3][0]*self.r2d, x_k_new[4][0]*self.r2d, x_k_new[5][0]*self.r2d)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_depth_frame'
        t.child_frame_id = 'tello'
        t.transform.translation.x = x_k_new[0][0]
        t.transform.translation.y = x_k_new[1][0]
        t.transform.translation.z = x_k_new[2][0]
        t.transform.rotation.w = quat[0]
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]

        self.tfbroadcaster.sendTransform(t)

        


    def camera_pose_subscriber(self,msg):
        """
        Callback function.
        This function gets called every time a message is received.
        """
        #dt = 1.0/50.0
        self.tx = msg.position.x
        self.ty = msg.position.y
        self.tz = msg.position.z
        # self.qx = msg.transforms[0].transform.rotation.x
        # self.qy = msg.transforms[0].transform.rotation.y
        # self.qz = msg.transforms[0].transform.rotation.z
        # self.qw = msg.transforms[0].transform.rotation.w

    def imu_subscriber(self,msg):
        """This callback function gets the imu data of the drone"""
        t_stamp_sec = msg.header.stamp.sec
        t_stamp_nsec = msg.header.stamp.nanosec
        t_stamp = float(t_stamp_sec + t_stamp_nsec/(10**9))
        self.dt = t_stamp - self.t_old #This time difference is about 0.025 s and doesnt work well
        self.drone_roll = msg.roll
        self.drone_pitch = msg.pitch
        self.drone_yaw = msg.yaw
        #self.z_barom = msg.
        self.t_old = t_stamp



def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  ekf_node = DroneEKF()
   
  # Spin the node so the callback function is called.
  rclpy.spin(ekf_node)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  ekf_node.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
    main()