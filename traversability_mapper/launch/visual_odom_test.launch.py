from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config_path = os.path.join(
        get_package_share_directory('traversability_mapper'),
        'config',
        '2d_grid_map_config.rviz'
    )

    rtabmap_params = {
        'frame_id': 'base_link',
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        'subscribe_imu': False,
        'approx_sync': True,
        'wait_imu_to_init': False,
        'queue_size': 10,
        'Vis/MinInliers': '5',
        'Odom/Strategy': '1',
        'Odom/ResetCountdown': '1',
        'OdomF2M/MaxSize': '2000',
        'Odom/ScanMatching/Enabled': 'true',
        'Odom/ScanMatching/MaxCorrespondenceDistance': '0.1',
        'Reg/Force3DoF': 'false'
    }

    odom_remappings = [
        ('rgbd_image', '/rgbd_image')]

    traversability_remappings = [
        ('/classified_region', '/classified_region')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        ComposableNodeContainer(
            name='rtabmap_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # 1. Syns and register components
                ComposableNode(
                    package='rtabmap_sync',
                    plugin='rtabmap_sync::RGBDSync',
                    name='rgbd_sync',
                    parameters=[{
                        'approx_sync': True,
                        'use_sim_time': use_sim_time,
                        'approx_sync_max_interval': 0.7
                    }],
                    remappings=[
                        ('rgb/image', '/camera/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/camera/color/camera_info'),
                        ('depth/image', '/camera/camera/depth/image_rect_raw')
                    ]),
                # 2. Calculate odometry components
                ComposableNode(
                    package='rtabmap_odom',
                    plugin='rtabmap_odom::RGBDOdometry',
                    name='rgbd_odometry',
                    parameters=[rtabmap_params, {'use_sim_time': use_sim_time}],
                    remappings=odom_remappings),
            ],
            output='screen',
        ),

        Node(
            package='traversability_mapper',
            executable='traversability_publisher_ros2',
            name='traversability_publisher_ros2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=traversability_remappings
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', rviz_config_path],
        )
    ])