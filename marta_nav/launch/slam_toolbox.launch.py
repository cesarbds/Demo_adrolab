from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():  
    
    return LaunchDescription([
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    output='screen',
        #    arguments=['0.240000', '0.000000', '0.128300', '1.000000', '0.000000', '0.000000', '0.000796', "base_link", "laser_link"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': 'config/mapper_params_online_async.yaml'
            }.items()
        )
    ])
