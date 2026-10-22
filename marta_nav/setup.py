from setuptools import setup
import os
from glob import glob

package_name = 'marta_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'marta_nav/restest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), 
        (os.path.join('share', package_name, 'info'), glob('info/*.yaml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cesar',
    maintainer_email='cesar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_pub = marta_nav.odom_tf_pub:main',
            'scan_pub = marta_nav.scan_pub:main',
            'p3_control = marta_nav.ros2controlap3:main',
            'go_to = marta_nav.gototargets:main',
            'go_to_more = marta_nav.gotomoretargets:main',
            'current_room = marta_nav.current_room:main',
            'image_pub = marta_nav.image_pub:main',
            'camera_info_pub = marta_nav.camera_info_pub:main',
            'updater_json = marta_nav.update_json:main',
            'odom_tf_pub_ssh = marta_nav.odom_tf_pub_ssh:main',
            'scan_pub_ssh = marta_nav.scan_pub_ssh:main',
            '3d_map = marta_nav.mapping:main',
            'octomap_coppelia = marta_nav.octomap_coppelia:main',
            'depth_pub = marta_nav.depth_pub:main',
        ],
    },
)
