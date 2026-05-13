"""
autonomous_nav.launch.py — Full autonomous navigation pipeline for Puzzlebot.

Launches (all in one terminal):
  1. Initial /clock publisher (unblocks sim-time nodes before Gazebo loads)
  2. Gazebo simulation (world + robot + ros2_control + RSP)
  3. Differential drive controller + twist relay
  4. SLAM node (builds occupancy-grid map in real time)
  5. A* global planner (reads /map + /goal_pose → publishes /path)
  6. Pure-pursuit path follower (reads /path + /slam_pose → /cmd_vel)
  7. Dynamic obstacle spawner (periodically adds boxes to Gazebo)
  8. RViz2 (visualises map, path, robot, particles)

Usage:
  ros2 launch puzzlebot_navigation autonomous_nav.launch.py
  ros2 launch puzzlebot_navigation autonomous_nav.launch.py world_name:=obstacles
  ros2 launch puzzlebot_navigation autonomous_nav.launch.py spawn_obstacles:=false
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav  = get_package_share_directory('puzzlebot_navigation')
    pkg_desc = get_package_share_directory('puzzlebot_description')
    pkg_ctrl = get_package_share_directory('puzzlebot_controller')
    pkg_slam = get_package_share_directory('puzzlebot_slam')

    nav_params  = os.path.join(pkg_nav,  'config', 'nav_params.yaml')
    nav_rviz    = os.path.join(pkg_nav,  'rviz',   'autonomous_nav.rviz')
    slam_params = os.path.join(pkg_slam, 'config', 'slam_params.yaml')

    # Use the plain URDF — it has wheel visual origins at xyz="0 0 0" so
    # the wheels render at the correct side positions in RViz.
    urdf_model = os.path.join(pkg_desc, 'urdf', 'puzzlebot.urdf.xacro')

    # ── Launch arguments ──────────────────────────────────────────────────
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='obstacles',
        description='Gazebo world name (e.g. "obstacles", "empty")',
    )
    spawn_obstacles_arg = DeclareLaunchArgument(
        'spawn_obstacles',
        default_value='true',
        description='Periodically spawn random dynamic obstacles in Gazebo',
    )

    world_name      = LaunchConfiguration('world_name')
    spawn_obstacles = LaunchConfiguration('spawn_obstacles')

    # ── 0a. Initial clock publisher ───────────────────────────────────────
    # Publishes /clock at t=0 until Gazebo's bridge takes over, so all
    # use_sim_time:True nodes unblock immediately instead of waiting 10-20 s.
    # Stops as soon as a second /clock publisher is detected (the bridge).
    initial_clock = ExecuteProcess(
        cmd=['python3', '-c',
             'import rclpy, time\n'
             'from rclpy.node import Node\n'
             'from rosgraph_msgs.msg import Clock\n'
             'rclpy.init()\n'
             'n = Node("init_clk")\n'
             'pub = n.create_publisher(Clock, "/clock", 10)\n'
             'deadline = time.monotonic() + 30\n'
             'while time.monotonic() < deadline:\n'
             '    if n.count_publishers("/clock") > 1: break\n'
             '    pub.publish(Clock())\n'
             '    time.sleep(0.05)\n'
             'n.destroy_node(); rclpy.shutdown()\n'],
        output='screen',
        name='initial_clock_publisher',
    )

    # ── 0b-extra. Initial odom seed ───────────────────────────────────────
    # SLAM's _odom_ready flag stays False until it receives a message on
    # /puzzlebot_controller/odom.  joint_state_broadcaster (which drives
    # simple_controller, which publishes odom) can take 30-60 s to spawn
    # after Gazebo loads.  Publishing a zero-pose odom immediately lets SLAM
    # start processing LiDAR scans and building the map right away.
    # Stops as soon as simple_controller (a real publisher) appears.
    initial_odom = ExecuteProcess(
        cmd=['python3', '-c',
             'import rclpy, time\n'
             'from rclpy.node import Node\n'
             'from nav_msgs.msg import Odometry\n'
             'rclpy.init()\n'
             'n = Node("init_odom")\n'
             'pub = n.create_publisher(Odometry, "/puzzlebot_controller/odom", 10)\n'
             'msg = Odometry()\n'
             'msg.header.frame_id = "odom"\n'
             'msg.child_frame_id = "base_footprint"\n'
             'msg.pose.pose.orientation.w = 1.0\n'
             'deadline = time.monotonic() + 90\n'
             'while time.monotonic() < deadline:\n'
             '    if n.count_publishers("/puzzlebot_controller/odom") > 1: break\n'
             '    pub.publish(msg)\n'
             '    time.sleep(0.1)\n'
             'n.destroy_node(); rclpy.shutdown()\n'],
        output='screen',
        name='initial_odom_publisher',
    )

    # ── 0b. Static TF placeholders ────────────────────────────────────────
    # map → odom: identity until SLAM broadcasts its own dynamic TF.
    # odom → base_footprint: identity until simple_controller starts.
    # Both let RViz place the robot in the map frame before any
    # navigation node has produced data.
    map_odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    odom_base_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    )

    # ── 1a. Gazebo + RSP (use_sim_time:True) ──────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'gazebo2.launch.py')),
        launch_arguments={
            'world_name': world_name,
            'model':      urdf_model,
        }.items(),
    )

    # ── 1b. ros2_control: joint_state_broadcaster + simple_controller ────
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ctrl, 'launch', 'controller.launch.py')),
        launch_arguments={
            'use_sim_time':          'true',
            'use_simple_controller': 'true',
        }.items(),
    )

    # ── 2. SLAM ───────────────────────────────────────────────────────────
    slam_node = Node(
        package='puzzlebot_slam',
        executable='slam_node',
        name='slam_node',
        parameters=[slam_params, {'use_sim_time': True}],
        remappings=[('/odom', '/puzzlebot_controller/odom')],
        output='screen',
    )

    # ── 3. A* global planner ──────────────────────────────────────────────
    planner_node = Node(
        package='puzzlebot_navigation',
        executable='astar_planner',
        name='astar_planner',
        parameters=[nav_params, {'use_sim_time': True}],
        output='screen',
    )

    # ── 4. Pure-pursuit path follower ─────────────────────────────────────
    follower_node = Node(
        package='puzzlebot_navigation',
        executable='path_follower',
        name='path_follower',
        parameters=[nav_params, {'use_sim_time': True}],
        output='screen',
    )

    # ── 5. Dynamic obstacle spawner (optional) ────────────────────────────
    spawner_node = Node(
        package='puzzlebot_navigation',
        executable='obstacle_spawner',
        name='obstacle_spawner',
        parameters=[nav_params, {'use_sim_time': True}],
        output='screen',
        condition=IfCondition(spawn_obstacles),
    )

    # ── 6. RViz2 ──────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav_rviz],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        world_name_arg,
        spawn_obstacles_arg,
        initial_clock,
        initial_odom,
        map_odom_static_tf,
        odom_base_static_tf,
        gazebo_launch,
        controller_launch,
        slam_node,
        planner_node,
        follower_node,
        spawner_node,
        rviz_node,
    ])
