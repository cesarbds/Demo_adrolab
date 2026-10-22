#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO  
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import CameraInfo
from sensor_msgs_py import point_cloud2
import struct
import pyrealsense2 as rs
from functions_3d import *
import sensor_msgs_py.point_cloud2 as pc2
from PIL import Image as PILImage




# Load the YOLO model
model = YOLO('yolo11n-seg.pt')  

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            5)
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            5)
        
        # Camera info subscription
        self.camera_info_subscription_color = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.camera_info_callback,
            5)

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.depth_frame = None
        
        self.segmented_rgb_image = None
        self.camera_matrix = None
        self.depth_intrinsics = None
        self.got_info  = False

    def camera_info_callback(self, msg):
        # Extract camera matrix (3x3) and distortion coefficients
        self.camera_matrix = np.array(msg.k).reshape(3, 3)

        # Get depth intrinsics if available
        self.depth_intrinsics = {
            "width": msg.width,
            "height": msg.height,
            "fx": msg.k[0],
            "fy": msg.k[4],
            "cx": msg.k[2],
            "cy": msg.k[5]
        }
        self.got_info = True

    def depth_callback(self, msg):
        # Convert the ROS image to a CV2 image
        #print("oi")
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Convert OpenCV image to PIL image
        self.depth_frame = PILImage.fromarray(cv_image)

    def rgb_callback(self, msg):
        # Convert the ROS image to a CV2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display the original image (optional)
        cv2.imshow("Image", cv_image)
        
       
        # Assuming results have a method to visualize or process the image
        # Display segmented results
         # Assuming 'plot' method generates an output image
        
        # resize rgb image to match depth image size if needed
        if np.array(cv_image).shape[0] != np.array(self.depth_frame).shape[0] or np.array(cv_image).shape[1] != np.array(self.depth_frame).shape[1]:
            cv_image = cv2.resize(cv_image, (np.array(self.depth_frame).shape[1], np.array(self.depth_frame).shape[0]), interpolation=cv2.INTER_AREA)

         # Perform segmentation with the YOLO model
        results = model(cv_image, show_boxes=False)  # No need to convert to np.array

        
        self.segmented_rgb_image = results[0].plot(boxes=False)
        if self.segmented_rgb_image is None:
            self.get_logger().error("Segmented RGB image is None.")
            return
        cv2.imshow("Segmented Image", self.segmented_rgb_image)

        cv2.waitKey(1)
        if self.depth_frame != None and self.got_info:
            pcd = create_pcd(self.camera_matrix, self.segmented_rgb_image, self.depth_frame, np.eye(4), mask_img = False)
            pc = convertCloudFromOpen3dToros2(pcd)
            self.publisher_.publish(pc)


def main(args=None):
    rclpy.init(args=args)
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.depth, 480, 848, rs.format.z16, 30)
    # config.enable_stream(rs.stream.color, 720, 1280, rs.format.yuyv, 30)

    # #Create pointcloud object
    # pc= rs.pointcloud()
    # points = rs.points()
    # #Define filters
    # #Decimation:
    # decimation = rs.decimation_filter()                     # filter magnitude between 2 and 8 def = 2
    # #Depth to disparity        
    # depth_to_disparity = rs.disparity_transform(True)
    # disparity_to_depth = rs.disparity_transform(False)
    # #Spatial:
    # spatial = rs.spatial_filter()
    # spatial.set_option(rs.option.holes_fill, 0)             # between 0 and 5 def = 0
    # spatial.set_option(rs.option.filter_magnitude, 2)       # between 1 and 5 def=2
    # spatial.set_option(rs.option.filter_smooth_alpha, 0.5)   # between 0.25 and 1 def=0.5
    # spatial.set_option(rs.option.filter_smooth_delta, 20)   # between 1 and 50 def=20
    # #Temporal:
    # temporal = rs.temporal_filter() 
    # temporal.set_option(rs.option.filter_smooth_alpha, 0.4)   
    # temporal.set_option(rs.option.filter_smooth_delta, 20)

    # colorizer = rs.colorizer()
    # yuyv_dec = rs.yuy_decoder()

    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
