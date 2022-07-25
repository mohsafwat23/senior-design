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
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
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

        self.not_detected_counter = 0
        self.episode_count = 0
        self.reward = 0
        self.actual_distance = 0
        self.z_desired = 0.3 #positive value
        self.integralX = 0
        self.integralY = 0
        self.integralZ = 0
        self.kp_xyz = np.array([0.4, 0.4, 0.4])
        self.kd_xyz = np.array([0.01, 0.01, 0.01])
        self.ki_xyz = np.array([0.0, 0.0, 0.0])
        self.error_prevX = 0
        self.error_prevY = 0
        self.error_prevZ = 0

        self.landing_command_pub = self.create_publisher(Twist,'/drone1/cmd_vel', 1)
        self.drone_action_client = self.create_client(TelloAction, '/drone1/tello_action')

        self.t_old = time.time()
        print("Landing node initialized")
        self.subscription = self.create_subscription(
        TFMessage, 
        '/tf', 
        self.listener_callback, 
        1)
        self.subscription # prevent unused variable warning

    # def quat_to_euler(self, qx, qy, qz, qw):
    #     t0 = +2.0 * (qw * qx + qy * qz)
    #     t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    #     roll_x = math.atan2(t0, t1)*(180/np.pi) #in degrees
    
    #     t2 = +2.0 * (qw * qy - qz * qx)
    #     t2 = +1.0 if t2 > +1.0 else t2
    #     t2 = -1.0 if t2 < -1.0 else t2
    #     pitch_y = math.asin(t2)*(180/np.pi) #in degrees
    
    #     t3 = +2.0 * (qw * qz + qx * qy)
    #     t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    #     yaw_z = math.atan2(t3, t4)*(180/np.pi) #in degrees

    #     euler = np.array([roll_x, pitch_y, yaw_z])
    #     return euler

    def listener_callback(self, msg):
        """
        Callback function.
        This function gets called every time a message is received.
        """
        t_stamp_sec = msg.transforms[0].header.stamp.sec
        t_stamp_nsec = msg.transforms[0].header.stamp.nanosec
        t_stamp = float(t_stamp_sec + t_stamp_nsec/(10**9))
        dt = t_stamp - self.t_old
        print(t_stamp)
        tx = msg.transforms[0].transform.translation.x
        ty = msg.transforms[0].transform.translation.y
        tz = msg.transforms[0].transform.translation.z
        qx = msg.transforms[0].transform.rotation.x
        qy = msg.transforms[0].transform.rotation.y
        qz = msg.transforms[0].transform.rotation.z
        qw = msg.transforms[0].transform.rotation.w
        
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
        errorY = -ty 
        errorZ = tz - self.z_desired

        # This checks if the drone isn't detecting the marker for a certain period of time
        if tx == 0.0 and ty == 0.0 and tz == 0.0:
            print("No marker detected")
            
            if self.not_detected_counter > 10:
                print("End of Episode")
                self.reward = -10
                self.not_detected_counter = 0
                # call service to reset the simulation
                pass
        else:
            distance = np.sqrt(tx**2 + ty**2 + tz**2)
            
        
        if distance < 0.1:
            print("Reached the desired height")
            self.reward = +10
            self.not_detected_counter = 0
            # call service to reset the simulation
            pass


        speed_LR = float(np.clip(speed_LR, -0.4, 0.4))
        speed_UD = float(np.clip(speed_UD, -0.4, 0.4))
        speed_FB = float(np.clip(speed_FB, -0.80, 0.80))
        """
        The speed values are defined as follows:
        twist.linear.x: forward/backward speed of the drone (m/s) 
        twist.linear.y: left/right speed of the drone (m/s)
        twist.linear.z: up/down speed of the drone (m/s)
        """

        twist = Twist()
        twist.linear.x = speed_FB
        twist.linear.y = speed_LR
        twist.linear.z = speed_UD
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.landing_command_pub.publish(twist)
        if distance > 0.05 and distance < 0.4:
            request = TelloAction.Request()
            request.cmd = "land"
            self.drone_action_client.call_async(request)
            distance = 1.0
        self.t_old = t_stamp

    
    def reset(self):
        """
        Reset the episode by resetting the simulation.
        """
        self.episode_count += 1
        print("Episode: ", self.episode_count)

        return "episode_over"
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
    main()