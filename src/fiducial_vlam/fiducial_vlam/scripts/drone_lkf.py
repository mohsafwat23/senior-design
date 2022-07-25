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
from sensor_msgs.msg import Imu
from tello_msgs.msg import FlightData
from transforms3d.euler import euler2quat, quat2euler

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
        timer_period = 0.05  # seconds

        self.pose_pub = self.create_publisher(Pose, '/drone/EKFpose', 1)
        self.pos = Pose()

        # Create the timer
        self.timer = self.create_timer(timer_period, self.ekf_drone)

        self.cam_pose_sub = self.create_subscription(
        Pose, 
        '/drone/poseBad', 
        self.camera_pose_subscriber, 
        1)
        self.cam_pose_sub # prevent unused variable warning

        # self.imu_sub = self.create_subscription(
        # FlightData, 
        # '/flight_data', 
        # self.imu_subscriber, 
        # 1)
        

        self.imu_sub = self.create_subscription(
        Imu, 
        '/drone1/imu', 
        self.imu_subscriber, 
        1)


        self.vel_sub = self.create_subscription(
        Twist, 
        '/drone1/cmd_vel', 
        self.vel_subscriber, 
        1)

        self.tfbroadcaster = TransformBroadcaster(self)




        self.drone_vx,self.drone_vy,self.drone_vz = 0,0,0

        self.A = np.eye(3)  
        # np.array([
        #     [1,0,0,0,0,0],
        #     [0,1,0,0,0,0],
        #     [0,0,1,0,0,0],
        # ])#np.eye(6)      #state transition matrix

        self.H_cam = np.array([
            [1.0,0.0,0.0],
            [0.0,1.0,0.0],
            [0.0,0.0,1.0],
        ])  #state to measurement matrix  
        
        #x,y,z
        self.x_hat_k_1 = np.array([[0.074,0.544,8.8]]).T #initial state 
             


        self.P_k_1 = np.eye(3)     #initialize covariance matrix
        self.Q = 0.02*np.eye(3)  #process noise covariance 
        
        self.R_cam = 0.4*np.eye(3)  #measurement noise covariance

        self.r2d = 180/np.pi


        self.dt = 1/20
        

        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0

        self.drone_roll = 0.0
        self.drone_pitch = 0.0
        self.drone_yaw = 0.0

        self.angles = (0,0,0)

   

        self.t_now = time.time()

        self.quat_new = np.array([0.5,0.5,0.5,0.5])

        self.no_marker = 0

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
    
    def quaternion_multiply(self,quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


    def ekf_drone(self):
        """This function gets the drone's pose from the EKF"""
        # twist = Twist()
        # v_yaw = 0.2
        # twist.linear.x = 0.0
        # twist.linear.y = 0.0
        # twist.linear.z = 0.0
        # twist.angular.x = 0.0
        # twist.angular.y = 0.0
        # twist.angular.z = v_yaw
        # t_now = time.time()
        # self.dt = t_now - self.t_old

        
        self.t_now = time.time()
        u_k_1 = np.array([[self.drone_vx,self.drone_vy,self.drone_vz]]).T
        R_u_fix = np.array([
            [0.0,-1.0,0.0],
            [0.0,0.0,-1.0],
            [1.0,0.0,0.0],
            ])
        # R_u_fix = np.array([
        #     [1.0,0.0,0.0],
        #     [0.0,0.0,1.0],
        #     [0.0,1.0,0.0],
        #     ])
        #the contol inputs are rotated 
        u_k_1 = R_u_fix@u_k_1 #== np.array([[self.drone_vy,self.drone_vz,-self.drone_vx,self.drone_omega]]).T
        #print(theta,psi,phi)


        yaw = np.array([      #input matrix
            [np.cos(self.drone_yaw), 0, np.sin(self.drone_yaw)],
            [        0,        -1., 0],
            [np.sin(self.drone_yaw),0, -np.cos(self.drone_yaw)],
            ])
        

        roll = np.array([      #input matrix
            [np.cos(self.drone_roll), np.sin(self.drone_roll),0],
            [np.sin(self.drone_roll),  -np.cos(self.drone_roll),0],
            [        0,         0,         -1.],
            ])


        pitch = np.array([      #input matrix
            [        1.,         0,         0],
            [0, -np.cos(self.drone_pitch),  np.sin(self.drone_pitch)],
            [0,-np.sin(self.drone_pitch),  -np.cos(self.drone_pitch)],
            ])

        rot = roll @ yaw @ pitch#yaw @ roll @ pitch #pitch @ roll @ yaw#
        

        B = self.dt*rot
        #print(x_hat_k)

        #print(B)
        #print(B)
        #print(1/self.dt)
        x_hat_k = self.A@self.x_hat_k_1 +  B@u_k_1
        #print(x_hat_k)

        P_k = self.A@self.P_k_1@self.A.T +self.Q

        K1 = P_k@self.H_cam.T@np.linalg.inv(self.H_cam@P_k@self.H_cam.T + self.R_cam)

        P_k_new = (np.identity(3) - K1@self.H_cam)@P_k


        if not (self.tx == 0.0 and self.ty == 0.0 and self.tz == 0.0):
            z_k_1_cam = np.array([[-self.tx, -self.ty, -self.tz]]).T
            #print("a",z_k_1_cam)
            z_k_1_cam = rot@z_k_1_cam
            print("b",z_k_1_cam)
            #print("meas",x_hat_k, "obs",self.H_cam@x_hat_k)
            x_k_new = x_hat_k + K1@(z_k_1_cam - self.H_cam@x_hat_k)
            #print("x1:",x_k_new[3][0], x_k_new[4][0], x_k_new[5][0])
            self.no_marker = 0

        else:
            x_k_new = x_hat_k
            self.no_marker+=1
            #if the marker is not visible for a long period of time, the drone will slow down
            if self.no_marker > 15:
                x_k_new = x_hat_k*0.01

        self.x_hat_k_1 = x_k_new
        self.P_k_1 = P_k_new
        #self.vel_pub.publish(twist)
        #print("x2:",x_k_new[3][0]*self.r2d, x_k_new[4][0]*self.r2d, x_k_new[5][0]*self.r2d)
        #print(x_k_new)
        #print("x_k_new:",np.shape(x_k_new),"B:",np.shape(B),"u_k_1:",np.shape(u_k_1),"z_k_1_cam:",np.shape(z_k_1_imu))
        #change to tuple
        #self.quat_new = euler2quat(x_k_new[4][0], x_k_new[5][0], x_k_new[3][0])
        quat2 = np.array([0.0,1.0,0.0,0.0])
        quat3 = self.quaternion_multiply(self.quat_new,quat2)
        #quat3 = self.quaternion_multiply(self.quat_new,quat2)
        #self.quat_new = euler2quat(self.drone_pitch,self.drone_yaw,self.drone_roll)
        #quat = euler2quat(x_k_new[3][0]*self.r2d, x_k_new[4][0]*self.r2d, x_k_new[5][0]*self.r2d)
        ############################################
        #self.x_hat_k_1 = x_hat_k
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_depth_frame'
        t.child_frame_id = 'tello'
        # t.transform.translation.x = x_k_new[0][0]
        # t.transform.translation.y = x_k_new[1][0]
        # t.transform.translation.z = x_k_new[2][0]
        # t.transform.rotation.w = quat3[0]#self.drone_w#quat[0]
        # t.transform.rotation.x = quat3[1]#self.drone_x#quat[1]
        # t.transform.rotation.y = quat3[2]#self.drone_y#quat[2]
        # t.transform.rotation.z = quat3[3]#self.drone_z#quat[3]


        # quat2 = np.array([0.0,0.0,1.0,0.0])
        # quat3 = self.quaternion_multiply(self.quat_new,quat2)
        # self.quat_new = euler2quat(self.drone_pitch,self.drone_yaw,self.drone_roll)

        t.transform.translation.x = self.x_hat_k_1[0][0]
        t.transform.translation.y = self.x_hat_k_1[1][0]
        t.transform.translation.z = self.x_hat_k_1[2][0]
        t.transform.rotation.w = quat3[0]#self.drone_w#quat[0]
        t.transform.rotation.x = quat3[1]#self.drone_x#quat[1]
        t.transform.rotation.y = quat3[2]#self.drone_y#quat[2]
        t.transform.rotation.z = quat3[3]#self.drone_z#quat[3]


        self.pos.position.x = self.x_hat_k_1[0][0]#x_k_new[0][0]
        self.pos.position.y = self.x_hat_k_1[0][0]#x_k_new[1][0]
        self.pos.position.z = self.x_hat_k_1[0][0]#x_k_new[2][0]
        self.pos.orientation.x = quat3[1] 
        self.pos.orientation.y = quat3[2]
        self.pos.orientation.z = quat3[3] 
        self.pos.orientation.w = quat3[0]  

        self.tfbroadcaster.sendTransform(t)
        #self.t_old = t_now

        self.pose_pub.publish(self.pos)


        


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

    # def imu_subscriber(self,msg):
    #     """This callback function gets the imu data of the drone"""
    #     t_stamp_sec = msg.header.stamp.sec
    #     t_stamp_nsec = msg.header.stamp.nanosec
    #     t_stamp = float(t_stamp_sec + t_stamp_nsec/(10**9))
    #     self.dt = t_stamp - self.t_old #This time difference is about 0.025 s and doesnt work well
    #     self.drone_roll = msg.roll
    #     self.drone_pitch = msg.pitch
    #     self.drone_yaw = msg.yaw
    #     #self.z_barom = msg.
    #     self.t_old = t_stamp

    def imu_subscriber(self,msg):
        """This callback function gets the imu data of the drone"""
        self.drone_w = msg.orientation.w
        self.drone_x = msg.orientation.x
        self.drone_y = msg.orientation.y
        self.drone_z = msg.orientation.z
        quat1 = np.array([self.drone_w,self.drone_x,self.drone_y,self.drone_z])
        quat2 = np.array([0.0,0.0,0.0,1.0])
        # quat3 = np.array([0.0,0.0,-1.0,0.0])
        #quat3 = self.quaternion_multiply(quat1,quat2)
        # self.quat_new = self.quaternion_multiply(self.quaternion_multiply(quat2,quat1),quat3)
        #self.quat_new = self.quaternion_multiply(quat_new2,quat3)
        self.angles = quat2euler(quat1)
        self.drone_roll = self.angles[0]
        self.drone_pitch = self.angles[1]
        self.drone_yaw = self.angles[2]
        #print(self.drone_roll,self.drone_pitch, self.drone_yaw )
        self.quat_new = euler2quat(self.drone_pitch,self.drone_yaw,self.drone_roll)
    
    def vel_subscriber(self,msg):
        """This callback function gets the velocity data of the drone"""
        self.dt = 1/20#self.t_now - self.t_old
        self.drone_vx = msg.linear.x
        self.drone_vy = msg.linear.y
        self.drone_vz = msg.linear.z
        self.drone_omega = msg.angular.z
        self.t_old = self.t_now

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