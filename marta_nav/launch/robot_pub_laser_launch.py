from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_dir = os.path.dirname("/home/maria/demo/ros2_ws/src/marta_nav/launch")
    send_params = os.path.join(
        get_package_share_directory('marta_nav'),
        'info',
        'server.yaml'
        )
    with open(send_params, 'r') as f:
        send_info = yaml.safe_load(f)["server"]
    return LaunchDescription([
        Node(
            package='marta_nav',
            namespace='P3DX',
            executable='odom_tf_pub_ssh',
            name='odom'
        ),
	Node(
	    name='rplidar_composition',
	    package='rplidar_ros',
	    namespace='P3DX',
	    executable='rplidar_composition',
	    output='screen',
	    parameters=[{
		'serial_port': '/dev/ttyUSB0',
		'serial_baudrate': 115200,
		'frame_id': 'laser_link',
		'inverted': True,
		'angle_compensate': True,
		'flip_x_axis':True,
	    }],
	),
        
        Node(
             package="tf2_ros",
             executable="static_transform_publisher",
             output="screen",
             arguments=['--x',"0.085",'--y', "0.00",'--z', "0.165", '--qx', "0", '--qy',"0", '--qz',"0",'--qw', "-1",'--frame-id', "/base_link", '--child-frame-id', "/laser_link"]
        ),
#        Node(
#             package="tf2_ros",
#             executable="static_transform_publisher",
#             output="screen",
#             arguments=['--x',"0.0375",'--y', "-0.0085",'--z', "0.365", '--qx', "-0.5", '--qy',"0.5", '--qz',"-0.5",'--qw', "0.5", '--frame-id', "/base_link", '--child-frame-id', "/meta_sense_a010"]
  #      ),
       # Node(
       #      package="tf2_ros",
       #      executable="static_transform_publisher",
       #      output="screen",
       #      arguments=['--x',"0.17",'--y', "0.00",'--z', "0.255", '--qx', "0", '--qy',"0", '--qz',"0",'--qw', "1",'--frame-id', "/base_link", '--child-frame-id', "d_435i_link"]
       # ),
        #Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     output="screen",
        #     arguments=["0", "0", "0", "0", "0", "0", "1", "/map", "/odom"]
        #),
        Node(
            package="marta_nav",
            namespace='P3DX',
            executable='p3_control',
            name='control',
        )
        #Node(
        #    package="marta_nav",
        #    namespace='P3DX',
        #    executable='current_room',
        #    name='current',
        #)
        # Node(
        #     package='marta_nav',
        #     name='updater',
        #     namespace='P3DX',
        #     executable='updater_json',
        #     output='screen',
        #     parameters=[send_info]
        #     )
    ])
