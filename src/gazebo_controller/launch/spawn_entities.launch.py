import os
import csv

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_controller')

    # Declare maze selection argument
    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value='basic_maze',
        description='Maze folder to load (Maze_hr, Maze_ng, Maze_ql_1, basic_maze)'
    )

    maze_arg = DeclareLaunchArgument(
        'maze',
        default_value='',
        description='Deprecated alias. Use map_folder instead.'
    )
    
    def launch_setup(context):
        maze_alias = context.perform_substitution(LaunchConfiguration('maze'))
        maze_from_map_folder = context.perform_substitution(LaunchConfiguration('map_folder'))
        maze_folder = maze_alias if maze_alias else maze_from_map_folder
        
        # Vehicle model SDF file
        vehicle_model_file = os.path.join(pkg_share, 'sdf', 'vehicle_blue_model.sdf')

        # Goal sphere SDF file
        goal_sphere_file = os.path.join(pkg_share, 'sdf', 'goal_sphere.sdf')

        # Poses CSV file
        poses_file = os.path.join(pkg_share, 'sdf', maze_folder, 'poses.csv')

        # Read poses from CSV
        poses = {}
        with open(poses_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                poses[row['name']] = {
                    'x': row['x'],
                    'y': row['y'],
                    'z': row['z'],
                    'yaw': row['yaw']
                }

        # Spawn the vehicle model in Gazebo
        robot_pose = poses['robot']
        spawn_vehicle = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', vehicle_model_file,
                '-name', 'vehicle_blue',
                '-x', robot_pose['x'], '-y', robot_pose['y'], '-z', robot_pose['z'],
                '-Y', robot_pose['yaw']
            ],
            output='screen'
        )

        # Spawn goal spheres along x=0 behind each inner wall
        goal1_pose = poses['goal_1']
        spawn_goal1 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', goal_sphere_file,
                '-name', 'goal_1',
                '-x', goal1_pose['x'], '-y', goal1_pose['y'], '-z', goal1_pose['z']
            ],
            output='screen'
        )

        goal2_pose = poses['goal_2']
        spawn_goal2 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', goal_sphere_file,
                '-name', 'goal_2',
                '-x', goal2_pose['x'], '-y', goal2_pose['y'], '-z', goal2_pose['z']
            ],
            output='screen'
        )

        goal3_pose = poses['goal_3']
        spawn_goal3 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', goal_sphere_file,
                '-name', 'goal_3',
                '-x', goal3_pose['x'], '-y', goal3_pose['y'], '-z', goal3_pose['z']
            ],
            output='screen'
        )

        # Static transform publishers
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        )
        
        maze_world_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'maze_world'],
            output='screen'
        )

        lidar_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.16', '0', '0.08', '0', '0', '0', 'vehicle_blue/base_link', 'vehicle_blue/lidar/lidar_sensor'],
            output='screen'
        )

        # Alias the scoped Gazebo base frame to the conventional base_link frame.
        base_link_alias_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'vehicle_blue/base_link', 'base_link'],
            output='screen'
        )

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(vehicle_model_file, 'r').read()
            }]
        )

        # Goal points publisher (reads goal positions from poses.csv)
        goal_points_publisher = Node(
            package='gazebo_controller',
            executable='goal_points_publisher',
            name='goal_points_publisher',
            output='screen',
            parameters=[{'maze': maze_folder}]
        )

        # NOTE: map_publisher (static map) is intentionally removed.
        # The lidar_mapper node in full_simulation.launch.py builds /map
        # incrementally from lidar scans, enabling frontier-based exploration.

        return [
            spawn_vehicle,
            spawn_goal1,
            spawn_goal2,
            spawn_goal3,
            static_tf,
            maze_world_tf,
            lidar_tf,
            base_link_alias_tf,
            robot_state_publisher,
            goal_points_publisher
        ]

    return LaunchDescription([
        map_folder_arg,
        maze_arg,
        OpaqueFunction(function=launch_setup)
    ])
