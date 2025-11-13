'''
 # @ Create Time: 2025-11-10 17:00:00
 # @ Modified by: Diego Hernandez
 # @ Modified time: 2025-11-10 17:00:00
 # @ Description: Launch file for Monocular SLAM Wrapper with Realsense D555 Camera
 '''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Create nodes
    mono_inertial_node: Node = Node(
            package='ros2_orb_slam3',
            executable='realsense_node_cpp',
            name='realsense_node',
            namespace='ORB_SLAM3',
            output='screen',
            parameters=[
                {'settings_file': '/home/ubuntu/ws_blue/src/slam-wrapper-v2/orb_slam3/config/Stereo-Inertial/RealSense_D555.yaml'},
                {'voc_file': '/home/ubuntu/ws_blue/src/slam-wrapper-v2/orb_slam3/Vocabulary/ORBvoc.txt.bin'},
                {'img1_topic': '/cam_realsense/camera/infra1/image_rect_raw'},
                {'img2_topic': '/cam_realsense/camera/infra2/image_rect_raw'},
                {'imu_topic': '/cam_realsense/camera/imu'},
                {'enable_debug_window': True},
            ]
    )

    return LaunchDescription([
        mono_inertial_node,
    ])

