#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField

import requests
from setuptools import setup, find_packages
import marta_nav.restest.restthru
import threading
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Header


class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/camera/pcl', 10)
        self.bridge = CvBridge()
        self.image_msg = Image()
        #cv2.namedWindow('Grayscale Image', cv2.WINDOW_NORMAL)


    def publish_depth(self, res):
        image1 = np.asarray(bytearray(res.raw.read()), dtype = "uint8")
        image1 = cv2.imdecode(image1, cv2.IMREAD_GRAYSCALE)
        depth_image = image1.astype(np.float32) 
        print(image1.shape)
        points = []
        height = 480
        width = 640

        center_row = height // 2
        center_col = width // 2
        print(image1[center_row, center_col])
            #depth = depth_image_in_meters[v, u]
            #points.append([u, v, depth])
        # Create a PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Replace with your frame ID
        pc_data = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            point_step=12,
            row_step=12 * len(points),
            data=np.array(points, dtype=np.float32).tobytes()
        )

        self.publisher_.publish(pc_data)
        # Display the original grayscale image
        cv2.imshow('Grayscale Image', image1)
        cv2.waitKey(1)  # Adjust the delay as needed


def main(args=None):
    rclpy.init(args=args)
    node = DepthPublisher()
    host = 'http://143.106.11.151'
    camera = '/perception/camera/0/depth'
    url = host + camera    

    marta_nav.restest.restthru.http_init()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    while rclpy.ok():
        
        res = requests.get('http://143.106.11.151:4950/perception/camera/0/depth', stream=True)
        if res.status_code == 200:
            node.publish_depth(res)

if __name__ == '__main__':
    main()
