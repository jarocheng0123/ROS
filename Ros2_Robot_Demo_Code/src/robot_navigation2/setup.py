from setuptools import setup
import os
from glob import glob
package_name = 'robot_navigation2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/',package_name,"launch"), glob('launch/*launch.py')),
        (os.path.join('share/',package_name,"maps"), glob('maps/*')),
        (os.path.join('share/',package_name,"rviz"), glob('rviz/*')),
        (os.path.join('share/',package_name,"param"), glob('param/*')),
        (os.path.join('share/',package_name,"behavior_trees"), glob('behavior_trees/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='znn',
    maintainer_email='znn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "demo_nav_to_pose_topic=robot_navigation2.demo_nav_to_pose_topic:main",
        "demo_nav_to_pose_simple=robot_navigation2.demo_nav_to_pose_simple:main",
        "demo_nav_waypoint_follower=robot_navigation2.demo_nav_waypoint_follower:main",
        "demo_nav_through_poses=robot_navigation2.demo_nav_through_poses:main",
        ],
    },
)
