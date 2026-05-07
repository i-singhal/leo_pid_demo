"""
Master Launch — leo_pid_demo
=============================
Single command to launch the entire simulation stack:
  Gazebo + Leo Rover + Ground Truth Bridge + PID Node + RViz

Usage:
  ros2 launch leo_pid_demo master_launch.py
  ros2 launch leo_pid_demo master_launch.py world:=obstacle
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch argument: world ───────────────────────────────────
    # Lets you pick which Gazebo world to load at launch time.
    # Default is 'empty' which uses the built-in leo_empty world.
    # Later you can add 'obstacle' or any other world name.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World to load: empty (default) or obstacle',
    )
    world_name = LaunchConfiguration('world')

    # ── Resolve the world SDF file path ──────────────────────────
    # 'empty' → use leo_gz_worlds built-in leo_empty.sdf
    # anything else → look in our package's worlds/ folder
    leo_gz_worlds = get_package_share_directory('leo_gz_worlds')
    pkg_dir = get_package_share_directory('leo_pid_demo')

    # We need a Python function to pick the right path at launch
    # time based on the argument value. OpaqueFunction lets us
    # do this — it runs a Python function during launch.
    from launch.actions import OpaqueFunction

    def launch_setup(context):
        """Called at launch time when argument values are known."""

        chosen_world = context.launch_configurations['world']

        # Pick the world SDF path
        if chosen_world == 'empty':
            world_path = os.path.join(
                leo_gz_worlds, 'worlds', 'leo_empty.sdf'
            )
            gz_world_name = 'leo_empty'
        else:
            world_path = os.path.join(
                pkg_dir, 'worlds', f'leo_{chosen_world}.sdf'
            )
            gz_world_name = f'leo_{chosen_world}'

        # ── 1. Gazebo + Leo Rover ────────────────────────────────
        leo_gz_bringup = get_package_share_directory('leo_gz_bringup')
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(leo_gz_bringup, 'launch', 'leo_gz.launch.py')
            ),
            launch_arguments={'sim_world': world_path}.items(),
        )

        # ── 2. Ground Truth Bridge (5s delay) ────────────────────
        # The topic name includes the world name, so we build it
        # dynamically based on which world was chosen.
        gt_topic = (
            f'/world/{gz_world_name}/dynamic_pose/info@'
            f'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        )
        ground_truth_bridge = TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_bridge',
                        'parameter_bridge', gt_topic,
                    ],
                    output='screen',
                )
            ],
        )

        # ── 2b. LiDAR Bridge (5s delay, only for obstacle world) ─
        # Bridges the overhead LiDAR point cloud from Gazebo to ROS 2.
        # Only needed when the obstacle world is loaded.
        lidar_bridge = None
        if chosen_world != 'empty':
            lidar_bridge = TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'run', 'ros_gz_bridge',
                            'parameter_bridge',
                            '/overhead_lidar/points@'
                            'sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                        ],
                        output='screen',
                    ),
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                            '--x', '0', '--y', '0', '--z', '12',
                            '--roll', '0', '--pitch', '1.5708', '--yaw', '0',
                            '--frame-id', 'world',
                            '--child-frame-id', 'overhead_lidar/link/overhead_lidar_sensor',
                        ],
                        output='screen',
                    ),
                ],
            )

        # ── 3. PID Waypoint Follower (7s delay, own terminal) ────
        pid_node = TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='leo_pid_demo',
                    executable='pid_waypoint_follower',
                    name='pid_waypoint_follower',
                    output='screen',
                    prefix='xterm -e',
                    parameters=[{
                        'gz_world_name': gz_world_name,
                    }],
                )
            ],
        )

        # ── 4. RViz (8s delay) ───────────────────────────────────
        rviz_config = os.path.join(pkg_dir, 'waypoint_follower.rviz')
        rviz = TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['rviz2', '-d', rviz_config],
                    output='screen',
                )
            ],
        )

        actions = [
            gazebo_launch,
            ground_truth_bridge,
            pid_node,
            rviz,
        ]
        if lidar_bridge is not None:
            actions.append(lidar_bridge)
        return actions

    # ── Assemble ─────────────────────────────────────────────────
    # world_arg is declared first so the launch system knows
    # about it. OpaqueFunction runs launch_setup() once the
    # argument value is resolved.
    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup),
    ])