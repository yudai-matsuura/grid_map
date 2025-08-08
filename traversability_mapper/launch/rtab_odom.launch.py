from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
   parameters={
       'frame_id':'base_link',
       'subscribe_depth':True,
       'subscribe_rgb':True,
       'subscribe_odom_info':True,
       'subscribe_imu':True,
       'approx_sync':True,
       'wait_imu_to_init':True,
       'queue_size':100,
       'Odom/Strategy':'1',
       'Vis/MinInliers':'8',
       'OdomF2M/MaxSize':'2000',
       'Reg/Force3DoF':'false'}

   remappings=[
       ('imu', '/imu/data'),
       ('rgb/image', '/throttle/camera/color/image_raw'),
       ('rgb/camera_info', '/throttle/camera/color/camera_info'),
       ('depth/image', '/throttle/camera/depth/image_rect_raw')]
   
   return LaunchDescription([
       # Nodes to launch       
       Node(
           package='rtabmap_odom', executable='rgbd_odometry', output='screen',
           parameters=[parameters],
           remappings=remappings),

       # Compute quaternion of the IMU
       Node(
           package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
           parameters=[{'use_mag': False, 
                        'world_frame':'enu', 
                        'publish_tf':False}],
           remappings=[('imu/data_raw', '/camera/imu')]),

   ])


