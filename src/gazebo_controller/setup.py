from glob import glob
from setuptools import find_packages, setup

package_name = 'gazebo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.csv')),
        ('share/' + package_name + '/sdf', glob('sdf/*.sdf')),
        ('share/' + package_name + '/sdf/basic_maze', glob('sdf/basic_maze/*')),
        ('share/' + package_name + '/sdf/Maze_hr', glob('sdf/Maze_hr/*')),
        ('share/' + package_name + '/sdf/Maze_ql_1', glob('sdf/Maze_ql_1/*')),
        ('share/' + package_name + '/sdf/Maze_ng', glob('sdf/Maze_ng/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Galen Mullins',
    maintainer_email='gmullin3@jh.edu',
    description='EN605.613 Final Project Template - ROS 2 differential drive robot controller with Gazebo simulation and multiple maze environments',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diffdrive_pid = gazebo_controller.diffdrive_pid:main',
            'astar_planner = gazebo_controller.astar_planner:main',
            'map_publisher = gazebo_controller.map_publisher:main',
            'goal_points_publisher = gazebo_controller.goal_points_publisher:main',
            'lidar_mapper = gazebo_controller.lidar_mapper:main',
            'frontier_planner = gazebo_controller.frontier_planner:main',
        ],
    },
)
