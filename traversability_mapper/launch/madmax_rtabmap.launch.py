from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'tcp_base',
                'odom_frame_id': 'nav',
                'subscribe_rgbd': False,
                'map_frame_id': 'nav',
                'subscribe_rgbd': False,
                'approx_sync': True,
                'use_sim_time': True,
                'queue_size': 20,
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/OptimizeFromGraphEnd': 'true',
                'RGBD/LinearUpdate': '0.1',
                'RGBD/AngularUpdate': '0.05',
                'Reg/Strategy': '1',
                'Vis/MaxFeatures': '2000',
                'RGBD/LoopClosureReextractFeatures': 'true',
                'Vis/MinInliers': '15',
                'Optimizer/Strategy': '1',
                'RGBD/DepthMax': 5.0,
                'Grid/FromDepth': True,
                'Grid/RangeMax': '5.0',
                'qos_odom': 2,
                'qos_image': 2,
                'qos_depth': 2,
                'qos_camera_info': 2,
            }],
            remappings=[
                ('/odom', '/nav'),
                ('rgb/image', '/fixed/left/image'),
                ('rgb/camera_info', '/fixed/left/camera_info'),
                ('depth/image', '/fixed/left/depth')
            ],
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('rgb/image', '/fixed/left/image'),
                ('depth/image', '/fixed/left/depth'),
                ('rgb/camera_info', '/fixed/left/camera_info'),
                ('odom', 'nav')
            ],
        ),

        Node(
            package='traversability_mapper',
            executable='fix_camera_frame',
            name='fix_camera_frame',
            output='screen'
        )
    ])
