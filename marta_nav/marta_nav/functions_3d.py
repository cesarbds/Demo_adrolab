import numpy as np
import open3d as o3d
import cv2
from ctypes import *  # convert float to uint32


from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

# The data structure of each point in ROS 2 PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + [
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
# T = np.eye(4)
# T[:3, :3] = mesh.get_rotation_matrix_from_xyz((0, np.pi / 3, np.pi / 2))
# T[0, 3] = 1
# T[1, 3] = 1.3
# print(T)
# mesh_t = copy.deepcopy(mesh).transform(T)
# o3d.visualization.draw_geometries([mesh, mesh_t])


def create_pcd(camera_matrix, rgb, depth, camera_pose=None, mask_img=False, filter_distance=3):
        """
        Create Open3D point cloud from RGB and depth images, and camera pose. filter_distance is used to filter out
        points that are further than a certain distance.
        :param rgb (pil image): RGB image
        :param depth (pil image): Depth image
        :param camera_pose (np.array): Camera pose
        :param mask_img (bool): Mask image
        :param filter_distance (float): Filter distance
        :return: Open3D point cloud
        """
        # convert rgb and depth images to numpy arrays
        rgb = np.array(rgb).astype(np.uint8)
        depth = np.array(depth)
        
        # load depth camera intrinsics
        H = rgb.shape[0]
        W = rgb.shape[1]
        scale = 1.0
        # create point cloud
        y, x = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
        depth = depth.astype(np.float32) / scale
        if mask_img:
            depth = depth * rgb
        mask = depth > 0
        x = x[mask]
        y = y[mask]
        depth = depth[mask]
        # convert to 3D
        X = (x - camera_matrix[0, 2]) * depth / camera_matrix[0, 0]
        Y = (y - camera_matrix[1, 2]) * depth / camera_matrix[1, 1]
        Z = depth
        Z[Z>3000] = np.inf
        # if Z.mean() > filter_distance:
        #     return o3d.geometry.PointCloud()
        # convert to open3d point cloud
        points = np.hstack((X.reshape(-1, 1), Y.reshape(-1, 1), Z.reshape(-1, 1)))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        if not mask_img:
            colors = rgb[mask]
            pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)
        pcd.transform(camera_pose)
        return pcd

def convertCloudFromOpen3dToros2(open3d_cloud, frame_id="camera_link"):
    header = Header()
    clock = Clock()
    header.stamp = clock.now().to_msg()
    header.frame_id = frame_id

    points = np.asarray(open3d_cloud.points)/1000.0
    if not open3d_cloud.colors:  # XYZ only
        fields = FIELDS_XYZ
        cloud_data = points
    else:  # XYZ + RGB
        fields = FIELDS_XYZRGB
        colors = np.floor(np.asarray(open3d_cloud.colors) * 255)
        colors = colors[:, 2] * BIT_MOVE_16 + colors[:, 1] * BIT_MOVE_8 + colors[:, 0]
        
        cloud_data = np.c_[points, colors]
    
    return pc2.create_cloud(header, fields, cloud_data)