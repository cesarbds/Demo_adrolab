#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from setuptools import setup, find_packages
import marta_nav.restest.restthru
import threading
import math
import numpy as np
from geometry_msgs.msg import Point, Quaternion, TransformStamped
import tf2_ros



class ScanPublisher(Node):
    def __init__(self):
        super().__init__('laser_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 1)
        self.timer = self.create_timer(0.05, self.publish)
        self.laser_msg = LaserScan()

    def publish_scan(self, rposes2):
        self.laser_msg = LaserScan()
        self.laser_msg.header.stamp = self.get_clock().now().to_msg()
        self.laser_msg.header.frame_id =  'laser_link'
        self.laser_msg.angle_min = -3*3.1415/4
        self.laser_msg.angle_max = 3*3.1415/4
        self.laser_msg.angle_increment = 0.00872665
        self.laser_msg.scan_time = 1/1000000000.0
        self.laser_msg.time_increment = 1/25/1000000000.0
        self.laser_msg.range_min = 0.05
        self.laser_msg.range_max = 10.0
        self.laser_msg.ranges = rposes2
    
    def publish(self):
        self.publisher_.publish(self.laser_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = ScanPublisher()
    host = 'http://10.1.3.130:4950'
    laser = '/perception/laser/0/distances'

    marta_nav.restest.restthru.http_init()
    
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(20)
    while rclpy.ok():
        rposes2, code = marta_nav.restest.restthru.http_get(host+laser)
        rposes2 = [x/1000 for x in rposes2]
        node.publish_scan(rposes2)
        #rate.sleep()

if __name__ == '__main__':
    main()
