#!/usr/bin/python3
# -*- coding: utf-8 -*-

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from setuptools import setup, find_packages
import math
import numpy as np
from geometry_msgs.msg import Point, Quaternion, TransformStamped
import tf2_ros

def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    
    Args:
        x, y, z, w (float): Components of the quaternion.
        
    Returns:
        tuple: A tuple containing three Euler angles (roll, pitch, yaw) in radians.
    """
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

class OdometrySubscriber(Node):

    def __init__(self, clientID):
        super().__init__('odometry_coppelia')
        self.sub_ = self.create_subscription(Odometry, 'P3DX/odom', self.odom_callback, 10)
        # Initialize the transform broadcaster
        self.timer = self.create_timer(0.1, self.send) 
        self.x = + 19.16 + 1.83
        self.y = -0.2 + -2.8
        self.th = 0
        self.clientID = clientID
        res, self.p3_handle = sim.simxGetObjectHandle(self.clientID, './16_Pioneer_P3DX', sim.simx_opmode_blocking)

    def odom_callback(self, data):
        self.msg = data
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.x = data.pose.pose.position.x / 1000
        self.y = data.pose.pose.position.y / 1000
        roll, pitch, yaw = quaternion_to_euler(self.msg.pose.pose.orientation.x, self.msg.pose.pose.orientation.y, self.msg.pose.pose.orientation.z, self.msg.pose.pose.orientation.w)
        self.th = yaw
        
    def send(self):
        # Send the transformation
        sim.simxSetObjectPosition(self.clientID, self.p3_handle, -1, [self.x, self.y, self.th], sim.simx_opmode_oneshot)
        print(f"Send: \n x: {self.x}; y: {self.y}; th: {self.th}")

def main(args=None):
    
    
    sim.simxFinish(-1)  # Just in case, close all opened connections
    clientID = sim.simxStart('10.1.3.104', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
    rclpy.init(args=args)
    if clientID != -1:
        print('Connected to remote API server')
        node = OdometrySubscriber(clientID)
        rclpy.spin(node)  # Keep the node running
    else:
        print('Failed to connect to CoppeliaSim')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
