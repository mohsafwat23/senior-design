#!/usr/bin/env python3
...
"""
author: Mohamed Safwat
email: mohamedmohabsafwat@gmail.com
"""
import time
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Vector3, PoseStamped, TransformStamped, Twist # Handles TransformStamped message
from nav_msgs.msg import Path
import numpy as np
from tf2_ros import TFMessage
from tello_msgs.srv import TelloAction
import math



class LandingNode(Node):
    """
    Create an LandingNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('landing_node')

        self.z_desired = 0.1 #positive value
        self.y_desired = 0.0 #negative value
        self.integralX = 0
        self.integralY = 0
        self.integralZ = 0
        self.integralYaw = 0
        self.kp_xyz = np.array([0.4, 0.4, 0.4, 0.4])
        self.kd_xyz = np.array([0.1, 0.1, 0.1, 0.1])
        self.ki_xyz = np.array([0.0, 0.0, 0.0, 0.0])
        self.error_prevX = 0
        self.error_prevY = 0
        self.error_prevZ = 0
        self.error_prevYaw = 0

        # distance properties
        self.camera_FOV = 82.6 #degrees FOV of the drone camera
        self.angle = (180.0 - self.camera_FOV)/2.0
        self.platform_width = 0.55 #meters
        self.distance_min = self.platform_width/2 * math.tan(math.radians(self.angle))
        self.distance_des = 8*self.distance_min #meters

        self.landing_command_pub = self.create_publisher(Twist,'/cmd_vel', 1)
        self.drone_angle = self.create_publisher(Vector3,'/angle_drone', 1)
        self.drone_action_client = self.create_client(TelloAction, '/tello_action')
        
        self.t_old = time.time()
        print("Landing node initialized")
        self.subscription = self.create_subscription(
        TFMessage, 
        '/tf', 
        self.listener_callback, 
        1)
        self.subscription # prevent unused variable warning

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

    def listener_callback(self, msg):
        """
        Callback function.
        This function gets called every time a message is received.
        """
        t_stamp_sec = msg.transforms[0].header.stamp.sec
        t_stamp_nsec = msg.transforms[0].header.stamp.nanosec
        t_stamp = float(t_stamp_sec + t_stamp_nsec/(10**9))
        dt = t_stamp - self.t_old #This time difference is about 0.025 s and doesnt work well
        dt = 1.0/50.0
        tx = msg.transforms[0].transform.translation.x
        ty = msg.transforms[0].transform.translation.y
        tz = msg.transforms[0].transform.translation.z
        qx = msg.transforms[0].transform.rotation.x
        qy = msg.transforms[0].transform.rotation.y
        qz = msg.transforms[0].transform.rotation.z
        qw = msg.transforms[0].transform.rotation.w
        angle = self.quat_to_euler(qx, qy, qz, qw)
        distance = np.sqrt(tx**2 + ty**2 + tz**2)
        """
        The pose distances are defined as follows:
        x: left/right
        y: up/down
        z: forward
        relative to the AruCo marker
        """
        #The X sign is flipped because the drone is facing the opposite direction of the camera
        errorX = -tx
        #The Y sign is flipped because the drone is facing the opposite direction 
        errorY = -ty + self.y_desired
        errorZ = tz - self.z_desired
        """This doesn't work because of pose estimation ambiguity"""
        errorYaw = -angle[1] #pitch angle becuse the y-direction is facing up on the aruco marker
        #errorYaw = -tx

        integralYaw = self.integralYaw + (errorYaw * dt)
        derivativeYaw = (errorYaw - self.error_prevYaw)/dt
        speed_YAW = self.kp_xyz[3] * errorYaw + self.kd_xyz[3] * derivativeYaw + self.ki_xyz[3] * integralYaw
        if distance < 0.8:
            self.z_desired = 0.1
            self.y_desired = -0.1
            speed_YAW = float(np.clip(speed_YAW, -0.4, 0.4))
            self.kp_xyz[1] = 2.0
        else:
            speed_YAW = 0.0



        
        integralX = self.integralX + (errorX * dt)
        integralY = self.integralY + (errorY * dt)
        integralZ = self.integralZ + (errorZ * dt)
        derivativeX = (errorX - self.error_prevX)/dt
        derivativeY = (errorY - self.error_prevY)/dt
        derivativeZ = (errorZ - self.error_prevZ)/dt
        speed_LR = self.kp_xyz[0] * errorX + self.kd_xyz[0] * derivativeX  + self.ki_xyz[0] * integralX
        speed_UD = self.kp_xyz[1] * errorY + self.kd_xyz[1] * derivativeY  + self.ki_xyz[1] * integralY
        speed_FB = self.kp_xyz[2] * errorZ + self.kd_xyz[2] * derivativeZ  + self.ki_xyz[2] * integralZ
        speed_LR = float(np.clip(speed_LR, -0.4, 0.4))
        speed_UD = float(np.clip(speed_UD, -0.6, 0.6))
        speed_FB = float(np.clip(speed_FB, -0.50, 0.50))
        """
        The speed values are defined as follows:
        twist.linear.x: forward/backward speed of the drone (m/s) 
        twist.linear.y: left/right speed of the drone (m/s)
        twist.linear.z: up/down speed of the drone (m/s)
        """
        if tx == 0.0:
            speed_LR = 0.0
        if ty == 0.0:
            speed_UD = 0.0
        if tz == 0.0:
            speed_FB = 0.0
        if angle[1] == 0.0:
            speed_YAW = 0.0
        twist = Twist()
        twist.linear.x = speed_FB
        twist.linear.y = speed_LR
        twist.linear.z = speed_UD
        print(speed_UD)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = speed_YAW#0.0
        self.landing_command_pub.publish(twist)

        angle_vector = Vector3()
        angle_vector.x = angle[0]
        angle_vector.y = angle[1]
        angle_vector.z = angle[2]
        self.drone_angle.publish(angle_vector)
        if distance > 0.05 and distance < 0.3:
            request = TelloAction.Request()
            request.cmd = "land"
            self.drone_action_client.call_async(request)
        self.t_old = t_stamp
        self.error_prevX = errorX
        self.error_prevY = errorY
        self.error_prevZ = errorZ
        self.error_prevYaw = errorYaw
        self.integralX = integralX
        self.integralY = integralY
        self.integralZ = integralZ
        self.integralYaw = integralYaw

def main(args=None):
   
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    landing_node = LandingNode()

    # Spin the node so the callback function is called.
    rclpy.spin(landing_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    landing_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    time.sleep(5)
    main()