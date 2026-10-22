from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='marta_nav',
            namespace='P3DX',
            executable='odom_tf_pub',
            name='odom'
        ),
        Node(
            package='marta_nav',
            namespace='P3DX',
            executable='scan_pub',
            name='scan'
        ),
        
        Node(
             package="tf2_ros",
             executable="static_transform_publisher",
             output="screen",
             arguments=["0.0375", "-0.0085", "0", "0", "0", "0", "1", "/base_link", "/laser_link"]
        ),
        Node(
             package="tf2_ros",
             executable="static_transform_publisher",
             output="screen",
             arguments=["0", "0", "0", "0", "0", "0", "1", "/base_footprint", "/base_link"]
        ),
        Node(
             package="tf2_ros",
             executable="static_transform_publisher",
             output="screen",
             arguments=["0", "0", "0", "0", "0", "0", "1", "/map", "/odom"]
        )
    ])
