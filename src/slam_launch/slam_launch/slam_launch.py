from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')
    rplidar_launch_dir = get_package_share_directory('rplidar_ros')
    return LaunchDescription([
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', '/home/isla/code/ascam_ros2_ws/src/config/rtabmap.rviz']

        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rplidar_launch_dir, '/launch/rplidar_s2_launch.py']),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z','0.3',
                          '--yaw', '0', '--pitch', '0', '--roll', '0',
                            '--frame-id', 'base_link', '--child-frame-id', 'laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0.5',
                          '--yaw', '-1.57', '--pitch', '0', '--roll', '-1.57',
                            '--frame-id', 'laser', '--child-frame-id', 'ascamera_hp60c_camera_link_0']
        ),

        # Node(
        #     package='sls_octmap',
        #     executable='octmap',
        #     name='octmap',
        # ),
        Node(
        namespace= "ascamera_hp60c",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "/home/isla/code/rtabmap_ws/src/ascamera/configurationfiles"},
            {"color_pcl": False},
            {"pub_tfTree": True},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 25},
        ],
        remappings=[]
    ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rtabmap_launch_dir, '/launch/rtabmap.launch.py']),
            launch_arguments={
                'frame_id': 'base_link',
                'rgb_topic': '/ascamera_hp60c/camera_publisher/rgb0/image',
                'depth_topic': '/ascamera_hp60c/camera_publisher/depth0/image_raw',
                'camera_info_topic': '/ascamera_hp60c/camera_publisher/rgb0/camera_info',
                'point_cloud_topic': '/ascamera_hp60c/camera_publisher/depth0/points',
                'args': '--delete_db_on_start --Mem/IncrementalMemory true',
                'odom_args': '--Vis/FeatureType 5 --Vis/MaxFeatures 1000',
                'approx_sync': 'true',
                'queue_size': '50',
                'qos': '1',
                'icp_odometry': 'false',
                'visual_odometry': 'true',
                'rviz' : 'true',
                'wait_for_transform': '2.0',
                'odom_frame_id': 'odom',
                'sync_queue_size': '50',
                'topic_queue_size': '10',
                'approx_sync_max_interval':'0.3',
            }.items()
    ),
    ])
