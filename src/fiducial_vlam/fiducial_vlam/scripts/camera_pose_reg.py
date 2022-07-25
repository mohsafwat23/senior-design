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
from geometry_msgs.msg import TransformStamped, Pose, Vector3 # Handles TransformStamped message
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import os
import math
import time
import sys
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler, euler2quat
from scipy.spatial.transform import Rotation as R
from tello_msgs.msg import FlightData



 
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
    self.declare_parameter("aruco_dictionary_name", "DICT_ARUCO_ORIGINAL")
    #self.declare_parameter("aruco_dictionary_name", "DICT_6X6_250")
    #self.declare_parameter("aruco_marker_side_length", 0.061)
    self.declare_parameter("aruco_marker_side_length", 0.1778)
    self.declare_parameter("camera_calibration_parameters_filename", "/home/mohamed/dev_ws/src/opencv_tools/data/calibration_chessboard.yaml")
    self.declare_parameter("image_topic", "/drone1/image_raw")
    self.declare_parameter("aruco_marker_name", "aruco_marker")
     
    # Read parameters
    aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
    self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
    self.camera_calibration_parameters_filename = self.get_parameter(
      "camera_calibration_parameters_filename").get_parameter_value().string_value
    image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
    self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value
 
    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
      self.get_logger().info("[INFO] ArUCo tag of '{}' is not supported".format(
        args["type"]))

  
    # Load the camera parameters from the saved file
    cv_file = cv2.FileStorage(
      self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
    #self.mtx = cv_file.getNode('K').mat()
    #self.dst = cv_file.getNode('D').mat()
    
    #CHARUCO BOARD CALIBRATION (COMPUTER)
    # self.mtx = np.array([
    # [743.41116567, 0.          , 479.16745299],
    #  [0.          , 742.16273303, 269.83681487],
    #  [0.          , 0.          , 1.          ]])
    # self.dst = np.array([[ 0.24915784, -0.60878258,  0.00273825,  0.0115768,   0.52518434]])
    # self.mtx = np.array([
    #  [921.9938565545156, 0.               , 480.5],
    #  [0.               , 921.9938565545156, 360.5],
    #  [0.               , 0.               , 1.   ]
    #  ])
    
    #######################TELLO DRONE CALIBRATION####################
    # self.mtx = np.array([
    #   [1.41751417e+03, 0.00000000e+00, 5.73407595e+02],
    #   [0.00000000e+00, 1.42339298e+03, 3.92504178e+02],
    #   [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    # self.dst = np.array([[ 1.00585204e+00, -3.01089540e+01,  9.82743988e-03, -1.41835250e-02,
    #             2.87673404e+02]])

    #####################Simulation distortions#############################
    self.mtx = np.array([
     [921.9938565545156, 0.               , 480.5],
     [0.               , 921.9938565545156, 360.5],
     [0.               , 0.               , 1.   ]
     ])
    self.dst = np.array([[ 0., 0.,  0.,  0., 0.]])

    self.distance_des = 8.0

    cv_file.release()
     
    self.t_prev = float(time.time())

    # Load the ArUco dictionary
    self.get_logger().info("[INFO] detecting '{}' markers...".format(
      aruco_dictionary_name))
    self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
    self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
      
    # make a publisher that publishes the angle
    #self.pub_angle = self.create_publisher(Vector3, "/angle_flipped", 1)
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

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
    self.pose_pub = self.create_publisher(Pose, '/drone/pose', 1)
    self.pos = Pose()

    # self.imu_sub = self.create_subscription(
    #   Imu, 
    #   '/drone1/imu', 
    #   self.imu_subscriber, 
    #   1)
    
    self.imu_sub = self.create_subscription(
        FlightData, 
        '/flight_data', 
        self.imu_subscriber, 
        1)
    

    self.pose_bad_pub = self.create_publisher(Pose, '/drone/poseBad', 1)
    self.pos_bad = Pose()
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()

    #distance between camera and marker
    self.distance = 10.0

    self.drone_roll = 0.0
    self.drone_pitch = 0.0
    self.drone_yaw = 0.0

    self.quat3 = np.array([1.0,0.0,0.0,0.0])
    
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    #self.get_logger().info('Receiving video frame')
  
    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data)

    if self.distance > self.distance_des:
      self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT['DICT_ARUCO_ORIGINAL'])
      self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
      #self.aruco_marker_side_length = 0.1778
      self.aruco_marker_side_length = 0.296
      self.large_marker.data = True
    else:# self.distance < self.distance_des:# and self.area > 4000.0:
      self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT['DICT_6X6_50'])
      #self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT['DICT_ARUCO_ORIGINAL'])
      self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
      #self.aruco_marker_side_length = 0.1778
      self.aruco_marker_side_length = 0.1355
      self.large_marker.data = False
    
    self.bool_publisher.publish(self.large_marker)

    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
      current_frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters,
      cameraMatrix=self.mtx, distCoeff=self.dst)
    
    t_now = float(time.time())
    dT = (t_now - self.t_prev)
    
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
 
       
        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]

        #another way to take the inverse rotation and translation
        # Rnew, _ = cv2.Rodrigues(np.array(rvecs[i][0]))
        # Rnew = np.matrix(Rnew).T
        # invTvec = np.dot(Rnew, np.matrix(-tvecs[i][0]).T)

        
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        #self.pub_angle.publish(angle_vector)
        quat = r.as_quat()   
        
        # self.pos_bad.position.x = rotation_matrix[0,3]
        # self.pos_bad.position.y = rotation_matrix[1,3]
        # self.pos_bad.position.z = rotation_matrix[2,3]


        self.pos.position.x = tvecs[i][0][0]#rotation_matrix[0,3]#
        self.pos.position.y = tvecs[i][0][1]#rotation_matrix[1,3]#
        self.pos.position.z = tvecs[i][0][2]#rotation_matrix[2,3]#
        self.pos.orientation.x = quat[3] 
        self.pos.orientation.y = quat[0] 
        self.pos.orientation.z = quat[1] 
        self.pos.orientation.w = quat[2]


        if marker_id[0] == 1:
          self.distance = (tvecs[0][0][0]**2 + tvecs[0][0][1]**2 + tvecs[0][0][2]**2)**0.5
          #print(self.distance)       # Draw the axes on the marker
        cv2.aruco.drawAxis(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)
    else:
      
      self.pos.position.x = 0.0
      self.pos.position.y = 0.0
      self.pos.position.z = 0.0
      self.pos.orientation.x = 0.0 
      self.pos.orientation.y = 0.0 
      self.pos.orientation.z = 0.0 
      self.pos.orientation.w = 1.0   

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


    self.t_prev = t_now   
    # Display image
    self.pose_pub.publish(self.pos)

    #self.pose_bad_pub.publish(self.pos_bad)
    # Send the transform
    cv2.imshow("camera", current_frame)
    
    cv2.waitKey(1)
    
   
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
    main()