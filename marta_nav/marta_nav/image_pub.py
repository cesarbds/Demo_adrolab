#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import requests
from setuptools import setup, find_packages
import marta_nav.restest.restthru
import threading
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image', 10)
        self.bridge = CvBridge()
        self.image_msg = Image()


    def publish_image(self, res):
        image1 = np.asarray(bytearray(res.raw.read()), dtype = "uint8")
        image1 = cv2.imdecode(image1, cv2.IMREAD_COLOR)
        try:
                self.image_msg = self.bridge.cv2_to_imgmsg(image1, "bgr8")
                self.image_msg.header.stamp = self.get_clock().now().to_msg()
                self.image_msg.header.frame_id = 'P3DX/camera'
                self.publisher_.publish(self.image_msg)
        except CvBridgeError as e:
            print(e)
            #print('Imagem sucesso')
        else:
            print('Sem sucesso')
    

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    host = 'http://10.1.3.130:4950'
    camera = '/perception/camera/0/image'
    url = host + camera    

    marta_nav.restest.restthru.http_init()

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    while rclpy.ok():
        
        res = requests.get('http://10.1.3.130:4950/perception/camera/0/image', stream=True)
        if res.status_code == 200:
            node.publish_image(res)

if __name__ == '__main__':
    main()
