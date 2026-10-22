import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnExecutionComplete
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import if_condition
from launch.substitutions import LaunchConfiguration
import yaml

def generate_launch_description():

    camera_info_dir = os.path.dirname("/home/cesar/ros2_ws/src/marta_nav/launch")
    param_config = os.path.join(camera_info_dir, 'info', 'axis_param.yaml')
    with open(param_config, 'r') as f:
        params = yaml.safe_load(f)["driver"]["ros__parameters"]
    return LaunchDescription ([   
        Node(
            package='marta_nav',
            name='driver',
            executable='axis_driver',
            output='screen',
            parameters=[params]
            ),
        Node(
            package='marta_nav',
            name='twist',
            executable='axis_twist',
            output='screen',
            parameters=[params]
            )
            
    ])    
    
