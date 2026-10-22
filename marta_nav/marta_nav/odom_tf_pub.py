#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from setuptools import setup, find_packages
import marta_nav.restest.restthru 
import threading
import math
import numpy as np
from geometry_msgs.msg import Point, Quaternion, TransformStamped
import tf2_ros



def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        # Initialize the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.t = TransformStamped()
        self.msg = Odometry()
        self.timer = self.create_timer(0.033, self.publish)


    def publish_odometry(self, x, y, z, quat_x, quat_y, quat_z, quat_w):
        self.msg = Odometry()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.pose.pose.position.x = x/1000
        self.msg.pose.pose.position.y = y/1000
        self.msg.pose.pose.position.z = z/1000
        self.msg.pose.pose.orientation.x = quat_x
        self.msg.pose.pose.orientation.y = quat_y
        self.msg.pose.pose.orientation.z = quat_z
        self.msg.pose.pose.orientation.w = quat_w
        
    
    def tf_odom_base_link(self, pose_odom, q):
        self.t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        self.t.transform.translation.x = (pose_odom['x'])/1000
        self.t.transform.translation.y = (pose_odom['y'])/1000
        self.t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]

    def publish(self):
        # Send the transformation
        self.tf_broadcaster.sendTransform(self.t)
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    host = 'http://10.1.3.130:4950'
    pose = "/motion/pose"
    max_ang_vel = '/motion/limits'

    marta_nav.restest.restthru.http_init()
    pose_inicial = {}
    pose_inicial['x'] = 810
    pose_inicial['y'] = 1110
    pose_inicial['th'] =  0
    resource = 'http://10.1.3.130:4950'+pose
    #print(resource)
    status = marta_nav.restest.restthru.http_put('http://10.1.3.130:4950'+pose, pose_inicial)
    limits = {}
    limits['maxrotvel'] = 6
    status = marta_nav.restest.restthru.http_put('http://10.1.3.130:4950'+max_ang_vel, limits)
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    while rclpy.ok():
        pose1, code = marta_nav.restest.restthru.http_get(host+pose)
        pose1['th'] = math.radians(pose1['th'])
        quat = get_quaternion_from_euler(0,0,pose1['th'])
        node.publish_odometry(pose1['x'], pose1['y'], 0, quat[0], quat[1], quat[2], quat[3])
        node.tf_odom_base_link(pose1, quat)

if __name__ == '__main__':
    main()
