#!/usr/bin/env python3
...
"""
author: Mohamed Safwat
email: mohamedmohabsafwat@gmail.com
"""
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import time
from geometry_msgs.msg import Twist, Vector3
from djitellopy import tello
from scipy.spatial.transform import Rotation as R
import numpy as np

  
class DroneImagePublisher(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class. In other words, it is a ROS 2 node.
    It inherits all of the methods of the Node class. 
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('drone_image_publisher')
        
        self.me = tello.Tello()
        self.me.connect()
        self.w, self.h = 640, 480    
        self.t_prev = time.time()
        self.me.streamon()

        # Create the publisher. This publisher will publish an Image
        self.publisher_ = self.create_publisher(Image, 'drone_video_frames', 5)
        # self.subscription = self.create_subscription(
        # Twist, 
        # 'cmd_vel', 
        # self.timer_callback, 
        # 10)
        # self.move = Twist()  
        # We will publish a message every 0.02 seconds
        timer_period = 0.025  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
                
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
    
    def timer_callback(self):
        img = self.me.get_frame_read().frame
        img = cv2.resize(img,(self.w,self.h))
        tim = time.time()
        f = 1/(tim - self.t_prev)

        #r_acc = rot.apply(acc)        
        #self.me.send_rc_control(self.move.linear.x, self.move.linear.y, self.move.linear.z, self.move.angular.z)
        #print(msg)

        self.publisher_.publish(self.br.cv2_to_imgmsg(img))
        
        self.t_prev = tim
        # Display the message on the console
        #self.get_logger().info('Publishing video frame')
   
def main(args=None):
   
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_publisher = DroneImagePublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()