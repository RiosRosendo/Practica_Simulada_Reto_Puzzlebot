# Velocity Bridge (converts /VelocityEncl, /VelocityEncnR → /wl, /wr)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_loc = get_package_share_directory('puzzlebot_localization')
    rviz_config = os.path.join(pkg_loc, 'config', 'rviz.rviz')

    # Velocity Bridge — convert real Jetson topics to MCL format
    velocity_bridge = Node(
        package='puzzlebot_localization',
        executable='velocity_bridge',
        name='velocity_bridge',
        output='screen',
    )

    # MCL Node (tuned for real hardware)
    mcl_node = Node(
        package='puzzlebot_localization',
        executable='mcl_node',
        name='mcl_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,  # Real hardware (not simulated time)
            'n_particles':   LaunchConfiguration('n_particles'),
            'sigma_field_m': LaunchConfiguration('sigma_field_m'),
            'ray_step':      LaunchConfiguration('ray_step'),
            'keep_fraction': LaunchConfiguration('keep_fraction'),
            'sigma_xy':      LaunchConfiguration('sigma_xy'),
            'sigma_theta':   LaunchConfiguration('sigma_theta'),
        }],
    )

    # RViz2 (top-down 2D view)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # Teleop (keyboard control)
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('cmd_vel', '/cmd_vel')],
    )

    return LaunchDescription([
        # Launch arguments (tuned for real hardware)
        DeclareLaunchArgument('n_particles',   default_value='300'),   # Reduced for Jetson
        DeclareLaunchArgument('sigma_field_m', default_value='0.2'),
        DeclareLaunchArgument('ray_step',      default_value='5'),
        DeclareLaunchArgument('keep_fraction', default_value='0.5'),
        DeclareLaunchArgument('sigma_xy',      default_value='0.05'),  # More noise for real sensors
        DeclareLaunchArgument('sigma_theta',   default_value='0.1'),   # More noise for real motion

        # Launch all nodes
        velocity_bridge,
        mcl_node,
        rviz,
        teleop,
    ])
