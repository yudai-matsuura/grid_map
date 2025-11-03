from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config_path = os.path.join(
        get_package_share_directory('traversability_mapper'),
        'config',
        '2d_grid_map_config_madmax.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (dataset) clock if true'
        ),

        # --- Traversability Mapper node ---
        Node(
            package='traversability_mapper',
            executable='traversability_publisher_for_madmax',
            name='traversability_publisher_for_madmax',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', rviz_config_path],
        )
    ])
