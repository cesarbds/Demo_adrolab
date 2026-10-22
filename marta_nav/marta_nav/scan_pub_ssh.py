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
        self.laser_msg = LaserScan()
        self.timer = self.create_timer(0.033, self.publish)       
    
    def publish(self):
        self.publisher_.publish(self.laser_msg)

    def publish_scan(self, rposes2):
        i = 0
        rposes2[0] = 10.0
        rposes2[1] = 10.0
        rposes2[2] = 10.0
        rposes2[3] = 10.0
        rposes2[4] = 10.0
        rposes2[5] = 10.0
        rposes2[6] = 10.0
        rposes2[7] = 10.0
        rposes2[8] = 10.0
        rposes2[9] = 10.0
        rposes2[10] = 10.0
        rposes2[11] = 10.0
        rposes2[12] = 10.0
        rposes2[13] = 10.0
        rposes2[14] = 10.0
        rposes2[15] = 10.0
        rposes2[16] = 10.0
        rposes2[17] = 10.0
        rposes2[525] = 10.0
        rposes2[524] = 10.0
        rposes2[526] = 10.0
        rposes2[527] = 10.0
        rposes2[528] = 10.0
        rposes2[529] = 10.0
        rposes2[530] = 10.0
        rposes2[531] = 10.0
        rposes2[532] = 10.0
        rposes2[533] = 10.0
        rposes2[534] = 10.0
        rposes2[535] = 10.0
        rposes2[536] = 10.0
        rposes2[537] = 10.0
        rposes2[538] = 10.0
        rposes2[539] = 10.0
        rposes2[540] = 10.0
        
        
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
    
def main(args=None):
    rclpy.init(args=args)
    node = ScanPublisher()
    host = 'http://143.106.11.151:4950'
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
