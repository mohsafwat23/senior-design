#!/usr/bin/env python3
# license removed for brevity
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from time import time
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import random
from geometry_msgs.msg import Quaternion, Vector3
from transforms3d.euler import euler2quat, quat2euler
import sys
#from pyquaternion import Quaternion
sys.path.append('/home/mohamed/openzen/build')
# sys.path.append('/home/ahmedkhalil/openzen')
import openzen

class ImuNode(Node):
    """
    Create an ImuNode that retrieves data from lpms 
    
    """
    def __init__(self):
        super().__init__('imu_publisher')

        #self.imuPub = self.create_publisher(Quaternion,"/imu_data",2)
        self.angVelPub = self.create_publisher(Vector3,"/gyr_data", 2)
        self.angVel = Vector3()

        # self.imu_angle = Quaternion()
        # self.imu_angle.x = 0.0
        # self.imu_angle.y = 0.0
        # self.imu_angle.z = 0.0
        # self.imu_angle.w = 1.0

        timer_period = 0.0025  # seconds

        self.counter = 0

        self.quat_init = np.array([1,0,0,0])

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timed_callback)
    
    def timed_callback(self):
        #try:

            
        zenEvent = client.wait_for_next_event()

        # check if its an IMU sample event and if it
        # comes from our IMU and sensor component
        if zenEvent.event_type == openzen.ZenEventType.ImuData and \
            zenEvent.sensor == imu.sensor and \
            zenEvent.component.handle == imu.component.handle:

            imu_data = zenEvent.data.imu_data

        if self.counter < 100:
            self.quat_init = np.array(imu_data.q)
            # print(self.quat_init)
            # print(self.counter)
        else:
            quat = np.array(imu_data.q)
            quat1 = np.array([quat[0],-quat[1],-quat[2],-quat[3]])
            # quat[0] = quat[0]
            # quat[1] = -quat[1]
            # quat[2] = -quat[2]
            # quat[3] = -quat[3]
            gyr = np.array(imu_data.g)
            #quat_new = quat1*self.quat_init
            #quat_new = self.quaternion_multiply(quat1,self.quat_init)
            # quat_new = self.quaternion_multiply(self.quat_init,quat1)
            # self.imu_angle.w = quat_new[0]
            # self.imu_angle.x = -quat_new[1]
            # self.imu_angle.y = -quat_new[2]
            # self.imu_angle.z = -quat_new[3]

            self.angVel.x = gyr[0]*np.pi/180.0
            self.angVel.y = gyr[1]*np.pi/180.0
            self.angVel.z = gyr[2]*np.pi/180.0

            # eul_angle = quat2euler((quat_new[0],-quat_new[1],-quat_new[2],-quat_new[3]))
            # print(eul_angle[0]*180/np.pi,eul_angle[1]*180/np.pi,eul_angle[2]*180/np.pi)
            
            #self.imuPub.publish(self.imu_angle)
            self.angVelPub.publish(self.angVel)

        
        self.counter+=1
        # except:
        #     pass

    
    def quaternion_multiply(self,quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                    x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                    -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                    x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)


def main(args=None):
   
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    imu_publisher = ImuNode()

    # Spin the node so the callback function is called.
    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

   
if __name__ == '__main__':
    openzen.set_log_level(openzen.ZenLogLevel.Warning)
    error, client = openzen.make_client()
    if not error == openzen.ZenError.NoError:
        print ("Error while initializing OpenZen library")
        sys.exit(1)

    error = client.list_sensors_async()

    # check for events
    sensor_desc_connect = None
    while True:
        zenEvent = client.wait_for_next_event()

        if zenEvent.event_type == openzen.ZenEventType.SensorFound:
            print ("Found sensor {} on IoType {}".format( zenEvent.data.sensor_found.name,
                zenEvent.data.sensor_found.io_type))
            if sensor_desc_connect is None:
                sensor_desc_connect = zenEvent.data.sensor_found

        if zenEvent.event_type == openzen.ZenEventType.SensorListingProgress:
            lst_data = zenEvent.data.sensor_listing_progress
            print ("Sensor listing progress: {} %".format(lst_data.progress * 100))
            if lst_data.complete > 0:
                break
    print ("Sensor Listing complete")

    if sensor_desc_connect is None:
        print("No sensors found")
        sys.exit(1)

    # connect to the first sensor found
    error, sensor = client.obtain_sensor(sensor_desc_connect)

    # or connect to a sensor by name
    #error, sensor = client.obtain_sensor_by_name("LinuxDevice", "LPMSCU2000003")

    if not error == openzen.ZenSensorInitError.NoError:
        print ("Error connecting to sensor")
        sys.exit(1)

    print ("Connected to sensor !")

    imu = sensor.get_any_component_of_type(openzen.component_type_imu)
    if imu is None:
        print ("No IMU found")
        sys.exit(1)

    ## read bool property
    error, is_streaming = imu.get_bool_property(openzen.ZenImuProperty.StreamData)
    if not error == openzen.ZenError.NoError:
        print ("Can't load streaming settings")
        sys.exit(1)

    print ("Sensor is streaming data: {}".format(is_streaming))
    main()