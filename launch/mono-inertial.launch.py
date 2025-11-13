'''
 # @ Create Time: 2025-11-03 10:14:15
 # @ Modified by: Paul Joseph
 # @ Modified time: 2025-11-03 12:22:08
 # @ Description: Launch file for Monocular-Inertial SLAM Wrapper with Realsense D455 Camera
 '''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # get package directory
    package_dir = get_package_share_directory(
        "ros2_orb_slam3")

    # Create nodes
    mono_inertial_node: Node = Node(
            package='ros2_orb_slam3',
            executable='mono_inertial_node_cpp',
            name='mono_inertial_node',
            namespace='ORB_SLAM3',
            output='screen',
            parameters=[
                {'settings_file': package_dir + '/orb_slam3/config/Monocular-Inertial/RealSense_D455.yaml'},
                {'voc_file': package_dir + '/orb_slam3/Vocabulary/ORBvoc.txt.bin'},
                {'img_topic': '/cam_realsense/camera/infra2/image_rect_raw'},
                {'imu_topic': '/cam_realsense/camera/imu'},
                {'enable_debug_window': True},
                {'is_inertial': True},
            ]
    )

    return LaunchDescription([
        mono_inertial_node,
    ])

