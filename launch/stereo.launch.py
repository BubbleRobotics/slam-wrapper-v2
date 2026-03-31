'''
 # @ Create Time: 2025-11-10 17:00:00
 # @ Modified by: Diego Hernandez
 # @ Modified time: 2025-11-10 17:00:00
 # @ Description: Launch file for Monocular SLAM Wrapper with Realsense D555 Camera
 '''

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # get package directory
    package_dir = get_package_share_directory(
        "ros2_orb_slam3")

    # Create nodes
    mono_inertial_node: Node = Node(
            package='ros2_orb_slam3',
            executable='stereo_node_cpp',
            name='stereo_node',
            namespace='ORB_SLAM3',
            output='screen',
            parameters=[
                {'settings_file': package_dir + '/orb_slam3/config/Stereo/paramsLake.yaml'},
                {'voc_file': package_dir + '/orb_slam3/Vocabulary/ORBvoc.txt.bin'},
                {'img0_topic': '/camera/left/image_dehazed/raw'},
                {'img1_topic': '/camera/right/image_dehazed/raw'},
                {'imu_topic': '/vectornav/Imu_body'},
                {'enable_debug_window': True},
                {'is_inertial': False},
                {'manual_time_sync': False},
                {'imu_from_yaml': True},
            ]
    )

    return LaunchDescription([
        mono_inertial_node,
    ])

