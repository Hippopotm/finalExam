import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_controller')

    # Declare maze selection argument
    drive_profile_arg = DeclareLaunchArgument(
        'drive_profile',
        default_value='fast',
        description='Controller tuning profile: fast or presentation_safe'
    )

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
        drive_profile = context.perform_substitution(LaunchConfiguration('drive_profile'))
        maze_folder = maze_alias if maze_alias else maze_from_map_folder
        world_file = os.path.join(pkg_share, 'sdf', maze_folder, 'maze_world.sdf')

        # Fast defaults and per-maze tuning to maximize speed while preserving wall clearance.
        pid_params = {
            'kp': 1.2,
            'kd': 0.35,
            'ki': 0.0,
            'lookahead': 0.60,
            'publish_rate': 40.0,
            'max_linear_vel': 1.8,
            'max_angular_vel': 2.6,
            'goal_tolerance': 0.30,
            'heading_rotate_threshold': 1.20,
            'heading_slowdown_threshold': 0.45,
            'min_turn_speed_scale': 0.35,
        }
        planner_params = {
            'goal_reach_tolerance': 0.40,
            'waypoint_stride_cells': 6,
            'obstacle_inflation_cells': 2,
        }

        if drive_profile == 'presentation_safe':
            pid_params.update({
                'kp': 0.95,
                'kd': 0.45,
                'lookahead': 0.50,
                'publish_rate': 35.0,
                'max_linear_vel': 1.15,
                'max_angular_vel': 2.2,
                'goal_tolerance': 0.22,
                'heading_rotate_threshold': 1.00,
                'heading_slowdown_threshold': 0.35,
                'min_turn_speed_scale': 0.25,
                'base_frame': 'vehicle_blue/base_link',
            })
            planner_params.update({
                'goal_reach_tolerance': 0.30,
                'waypoint_stride_cells': 4,
                'obstacle_inflation_cells': 3,
            })

        if maze_folder == 'Maze_ng':
            # Dense maze: keep faster than baseline, but preserve tighter turn margin.
            pid_params.update({
                'max_linear_vel': 1.45,
                'max_angular_vel': 2.8,
                'lookahead': 0.55,
                'heading_rotate_threshold': 1.10,
                'goal_tolerance': 0.28,
            })
            planner_params.update({
                'waypoint_stride_cells': 4,
                'obstacle_inflation_cells': 3,
            })
        elif maze_folder == 'Maze_hr':
            pid_params.update({
                'max_linear_vel': 1.80,
                'max_angular_vel': 2.7,
                'lookahead': 0.60,
                'heading_rotate_threshold': 1.20,
            })
            planner_params.update({
                'waypoint_stride_cells': 6,
                'obstacle_inflation_cells': 2,
            })
        elif maze_folder == 'Maze_ql_1':
            # More open sections: longest stride and highest linear speed.
            pid_params.update({
                'max_linear_vel': 2.10,
                'max_angular_vel': 2.5,
                'lookahead': 0.65,
                'heading_rotate_threshold': 1.30,
                'goal_tolerance': 0.33,
            })
            planner_params.update({
                'waypoint_stride_cells': 8,
                'obstacle_inflation_cells': 2,
                'goal_reach_tolerance': 0.45,
            })

        # Keep per-maze behavior but cap to safer limits under presentation profile.
        if drive_profile == 'presentation_safe':
            if maze_folder == 'Maze_ng':
                pid_params.update({
                    'max_linear_vel': 1.00,
                    'max_angular_vel': 2.3,
                    'lookahead': 0.48,
                    'heading_rotate_threshold': 0.95,
                })
                planner_params.update({
                    'waypoint_stride_cells': 3,
                    'obstacle_inflation_cells': 4,
                })
            elif maze_folder == 'Maze_hr':
                pid_params.update({
                    'max_linear_vel': 1.15,
                    'max_angular_vel': 2.2,
                    'lookahead': 0.50,
                })
                planner_params.update({
                    'waypoint_stride_cells': 4,
                    'obstacle_inflation_cells': 3,
                })
            elif maze_folder == 'Maze_ql_1':
                pid_params.update({
                    'max_linear_vel': 1.30,
                    'max_angular_vel': 2.2,
                    'lookahead': 0.55,
                })
                planner_params.update({
                    'waypoint_stride_cells': 5,
                    'obstacle_inflation_cells': 3,
                })

        # Launch Gazebo with the world
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
                )
            ]),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        )

        # Include spawn entities launch file (robot, goals, map, static transforms)
        spawn_entities = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'spawn_entities.launch.py')
            ),
            launch_arguments={
                'map_folder': maze_folder,
                'maze': maze_folder,
            }.items()
        )

        # Bridge, PID, RViz
        bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'ros_gz_bridge.launch.py')
            )
        )

        pid_controller = Node(
            package='gazebo_controller',
            executable='diffdrive_pid',
            name='diffdrive_pid',
            output='screen',
            parameters=[pid_params]
        )

        # Frontier-based exploration planner (replaces static astar_planner)
        planner = Node(
            package='gazebo_controller',
            executable='frontier_planner',
            name='frontier_planner',
            output='screen',
        )

        # Lidar-based incremental mapper (replaces static map_publisher)
        mapper = Node(
            package='gazebo_controller',
            executable='lidar_mapper',
            name='lidar_mapper',
            output='screen',
        )

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'rviz_view.rviz')]
        )

        return [
            gazebo,
            spawn_entities,
            bridge,
            mapper,
            planner,
            pid_controller,
            rviz
        ]

    return LaunchDescription([
        drive_profile_arg,
        map_folder_arg,
        maze_arg,
        OpaqueFunction(function=launch_setup)
    ])