#!/usr/bin/env python3
...
# Publishes a coordinate transformation between an ArUco marker and a camera
"""
author: Mohamed Safwat
email: mohamedmohabsafwat@gmail.com
"""
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary ROS 2 libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import TransformStamped, Twist,Pose # Handles TransformStamped message
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from tello_msgs.srv import TelloAction
import os
import math
import time
import sys
import csv
from transforms3d.euler import quat2euler


# Import Python libraries
import cv2 # OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R

# The different ArUco dictionaries built into the OpenCV library.
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class ArucoNode(Node):
  """
  Create an ArucoNode class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('aruco_node')

    # Declare parameters
    self.declare_parameter("aruco_dictionary_name_1", "DICT_ARUCO_ORIGINAL")
    self.declare_parameter("aruco_dictionary_name_2", "DICT_ARUCO_ORIGINAL")
    self.declare_parameter("aruco_marker_side_length_1", 0.181)
    self.declare_parameter("aruco_marker_side_length_2", 0.296)
    self.declare_parameter("camera_calibration_parameters", "Simulation")
    self.declare_parameter("image_topic", "/image_raw")
    self.declare_parameter("aruco_marker_name", "aruco_marker")

    # Read parameters
    self.aruco_dictionary_name_1 = self.get_parameter("aruco_dictionary_name_1").get_parameter_value().string_value
    self.aruco_dictionary_name_2 = self.get_parameter("aruco_dictionary_name_2").get_parameter_value().string_value
    
    self.aruco_marker_side_length_1 = self.get_parameter("aruco_marker_side_length_1").get_parameter_value().double_value
    self.aruco_marker_side_length_2 = self.get_parameter("aruco_marker_side_length_2").get_parameter_value().double_value

    self.camera_calibration_parameters = self.get_parameter(
      "camera_calibration_parameters").get_parameter_value().string_value
    image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
    self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value

    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(self.aruco_dictionary_name_1, None) is None:
      self.get_logger().info("[INFO] ArUCo tag of '{}' is not supported".format(
        args["type"]))


    # Load the camera parameters from the saved file
    # cv_file = cv2.FileStorage(
    #   self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)

    #CHARUCO BOARD CALIBRATION (COMPUTER)
    # self.mtx = np.array([
    # [743.41116567, 0.          , 479.16745299],
    #  [0.          , 742.16273303, 269.83681487],
    #  [0.          , 0.          , 1.          ]])
    # self.dst = np.array([[ 0.24915784, -0.60878258,  0.00273825,  0.0115768,   0.52518434]])
    if self.camera_calibration_parameters == "Simulation":
      #####################Simulation distortions#############################
      self.mtx = np.array([
      [921.9938565545156, 0.               , 480.5],
      [0.               , 921.9938565545156, 360.5],
      [0.               , 0.               , 1.   ]
      ])
      self.dst = np.array([[ 0., 0.,  0.,  0., 0.]])
    else:
      ######################TELLO DRONE CALIBRATION####################
      self.mtx = np.array([
        [1.41751417e+03, 0.00000000e+00, 5.73407595e+02],
        [0.00000000e+00, 1.42339298e+03, 3.92504178e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

      self.dst = np.array([[ 1.00585204e+00, -3.01089540e+01,  9.82743988e-03, -1.41835250e-02,
                  2.87673404e+02]])


    self.camera_FOV = 82.6 #degrees FOV of the drone camera
    self.angle = (180.0 - self.camera_FOV)/2.0
    self.platform_width = 0.55 #meters
    self.distance_min = self.platform_width/2 * math.tan(math.radians(self.angle))
    self.distance_des = 8#12*self.distance_min


    #cv_file.release()

    self.t_prev = float(time.time())

    # Load the ArUco dictionary
    self.get_logger().info("[INFO] detecting '{}' markers...".format(
      self.aruco_dictionary_name_1))
    self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_dictionary_name_1])
    self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()

    self.drone_action_client = self.create_client(TelloAction, '/tello_action')


  #============================================================================
  #    Send drone desired velocity
  #============================================================================
    self.drone_vel_pub = self.create_publisher(Twist,'/cmd_vel', 1)
    self.drone_vel_msg = Twist()


  #============================================================================
  #    Check which marker is being detected
  #============================================================================
    self.bool_publisher = self.create_publisher(Bool, "/large_aruco", 1)

    self.large_marker = Bool()

    self.subscription = self.create_subscription(
      Image,
      image_topic,
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    # Initialize the transform broadcaster
    self.tfbroadcaster = TransformBroadcaster(self)

    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()

    #distance between camera and marker
    self.dist_target = 0.1
    #self.w, self.h = 640, 480
    self.w, self.h = 960,720 
    self.pidX = [0.18, 0.1,0]  #lr
    self.pidY = [0.18, 0.05,0]  #ud
    self.pidZ = [0.2, 0.1,0]    #fb
    self.pidYaw = [0.15, 0.1, 0]
    self.pErrorYaw = 0.0
    self.pErrorX = 0.0
    self.pErrorY = 0.0
    self.pErrorZ = 0.0
    self.dt = 1/30
    self.integralX, self.integralY, self.integralZ = 0.0, 0.0, 0.0

    self.distance = 10

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    #self.get_logger().info('Receiving video frame')

    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data)
    if self.distance > self.distance_des:
      self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_dictionary_name_1])
      self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
      #self.aruco_marker_side_length = 0.1778
      self.aruco_marker_side_length = self.aruco_marker_side_length_1
      #self.aruco_marker_side_length = 0.296
      self.large_marker.data = True
    else:# self.distance < self.distance_des:# and self.area > 4000.0:
      #self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT['DICT_6X6_50'])
      self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_dictionary_name_2])
      self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
      #self.aruco_marker_side_length = 0.1778
      self.aruco_marker_side_length = self.aruco_marker_side_length_2
      #self.aruco_marker_side_length = 0.1355
      self.large_marker.data = False
    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
      current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters,
      cameraMatrix=self.mtx, distCoeff=self.dst)

    self.bool_publisher.publish(self.large_marker)
    t_now = float(time.time())
    self.dt = (t_now - self.t_prev)
    print(self.dt)
    # Check that at least one ArUco marker was detected
    if marker_ids is not None:

      # Draw a square around detected markers in the video frame
      cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)

      # Get the rotation and translation vectors
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        self.aruco_marker_side_length,
        self.mtx,
        self.dst)

      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder,
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
      #https://github.com/opencv/opencv/issues/8813
      for i, marker_id in enumerate(marker_ids):

        center = np.mean(corners[0][0], axis=0)
        self.x_centerPixel = center[0]
        self.y_centerPixel = center[1]
        self.dist = np.linalg.norm(tvecs)

        errorX = (-self.x_centerPixel + self.w/2)/100
        errorY = (-self.y_centerPixel + self.h/2)/100
        errorZ = (self.dist - self.dist_target)
        errorYaw = errorX
        # print("errorX: ", errorX, "errorY: ", errorY, "errorZ: ", errorZ)
        self.integralX = self.integralX + (errorX * self.dt)
        self.integralY = self.integralY + (errorY * self.dt)
        self.integralZ = self.integralZ + (errorZ * self.dt)
        speed_lr = self.pidX[0]*errorX + self.pidX[1]*(errorX-self.pErrorX)/self.dt + self.pidX[2]*self.integralX
        speed_ud = self.pidY[0]*errorY + self.pidY[1]*(errorY-self.pErrorY)/self.dt + self.pidY[2]*self.integralY
        speed_fb = self.pidZ[0]*errorZ + self.pidZ[1]*(errorZ-self.pErrorZ)/self.dt + self.pidZ[2]*self.integralZ
        speed_yaw = self.pidYaw[0]*errorYaw + self.pidYaw[1]*(errorYaw-self.pErrorYaw)/self.dt
        speed_fb = np.clip(speed_fb, -0.80, 0.80)
        speed_lr = np.clip(speed_lr, -0.80, 0.80)
        speed_ud = np.clip(speed_ud, -0.80, 0.80)
        speed_yaw = np.clip(speed_yaw, -0.5, 0.5)
        if self.x_centerPixel == 0:
            speed_lr = 0.0
            speed_yaw = 0.0
            errorX = 0.0
            errorYaw = 0.0
        if self.y_centerPixel == 0:
            speed_ud = 0.0
            errorY = 0.0
        if self.dist == 0:
            speed_fb = 0.0
            errorZ = 0.0
        #print(speed_fb, speed_lr, speed_ud)

        self.drone_vel_msg.linear.x = speed_fb
        self.drone_vel_msg.linear.y = speed_lr
        self.drone_vel_msg.linear.z = speed_ud
        self.drone_vel_msg.angular.z = 0.0
        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        # rotation_matrix[0:3,3:] = np.array([tvecs[i][0]]).T
        # rotation_matrix = np.linalg.inv(rotation_matrix)

        if self.dist > 0.005 and self.dist < 0.5:
          request = TelloAction.Request()
          request.cmd = "land"
          self.drone_action_client.call_async(request)

        #another way to take the inverse rotation and translation
        Rnew, _ = cv2.Rodrigues(np.array(rvecs[i][0]))
        Rnew = np.matrix(Rnew).T
        invTvec = np.dot(Rnew, np.matrix(-tvecs[i][0]).T)

        r = R.from_matrix(rotation_matrix[0:3, 0:3].T)


        #self.pub_angle.publish(angle_vector)
        quat = r.as_quat()


        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_depth_frame'
        t.child_frame_id = self.aruco_marker_name
        t.transform.translation.x = invTvec.item(0,0)
        t.transform.translation.y = invTvec.item(1,0)
        t.transform.translation.z = invTvec.item(2,0)
        t.transform.rotation.x = quat[3]
        t.transform.rotation.y = quat[0]
        t.transform.rotation.z = quat[1]
        t.transform.rotation.w = quat[2]

        self.pErrorX = errorX
        self.pErrorY = errorY
        self.pErrorZ = errorZ
        self.pErrorYaw = errorYaw


        if marker_id[0] == 1:
          #self.area = ((top_left_x*top_right_y - top_left_y*top_right_x) + (top_right_x*bottom_left_y - top_right_y*bottom_left_x) + (bottom_left_x*bottom_right_y - bottom_left_y*bottom_right_x) + (bottom_right_x*top_left_y - bottom_right_y*top_left_x))/2
          #print(self.area, self.distance)


          #print(i,tvecs[i][0][0],tvecs[i][0][1],tvecs[i][0][2])


          self.distance = (tvecs[0][0][0]**2 + tvecs[0][0][1]**2 + tvecs[0][0][2]**2)**0.5
        # Draw the axes on the marker

        cv2.aruco.drawAxis(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.1)
    else:
      t = TransformStamped()
      t.header.stamp = self.get_clock().now().to_msg()
      t.header.frame_id = 'camera_depth_frame'
      t.child_frame_id = self.aruco_marker_name
      t.transform.translation.x = 0.0
      t.transform.translation.y = 0.0
      t.transform.translation.z = 0.0
      t.transform.rotation.x = 0.0
      t.transform.rotation.y = 0.0
      t.transform.rotation.z = 0.0
      t.transform.rotation.w = 1.0

      self.drone_vel_msg.linear.x = 0.0
      self.drone_vel_msg.linear.y = 0.0
      self.drone_vel_msg.linear.z = 0.0
      self.drone_vel_msg.angular.z = 0.0


    self.t_prev = t_now
    # Display image
    self.tfbroadcaster.sendTransform(t)

    #publish the velocity
    self.drone_vel_pub.publish(self.drone_vel_msg)

    # Send the transform
    cv2.imshow("camera", current_frame)

    cv2.waitKey(1)

  # def imu_subscriber(self,msg):
  #   """This callback function gets the imu data of the drone"""
  #   t_stamp_sec = msg.header.stamp.sec
  #   t_stamp_nsec = msg.header.stamp.nanosec
  #   t_stamp = float(t_stamp_sec + t_stamp_nsec/(10**9))
  #   #self.dt = t_stamp - self.t_old #This time difference is about 0.025 s and doesnt work well
  #   self.drone_roll = msg.roll*np.pi/180
  #   self.drone_pitch = msg.pitch*np.pi/180
  #   self.drone_yaw = msg.yaw*np.pi/180
  #   #self.z_barom = msg.
  #   self.t_old = t_stamp

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  aruco_node = ArucoNode()

  # Spin the node so the callback function is called.
  rclpy.spin(aruco_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  aruco_node.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
    #time.sleep(5)
    main()