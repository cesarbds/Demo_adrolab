#!/usr/bin/env python3


import rclpy
import threading
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

def yaml_to_camerainfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle, Loader=yaml.FullLoader)
    # Parse
    camera_info_msg = CameraInfo()
    #print(camera_info_msg)
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.k = calib_data["camera_matrix"]["data"]
    camera_info_msg.d = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.r = calib_data["rectification_matrix"]["data"]
    camera_info_msg.p = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

class camerainfopublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.publisher = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.camera_info_msg = yaml_to_camerainfo("/home/cesar/ros2_ws/src/marta_nav/info/calibaxis.yaml")

def main(args=None):

    rclpy.init()
    node = camerainfopublisher()
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(10)

    while rclpy.ok():
        node.publisher.publish(node.camera_info_msg)
        rate.sleep()
    
if __name__ == "__main__":
    main()
    
