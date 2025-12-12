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
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # get package directory
    package_dir = get_package_share_directory(
        "ros2_orb_slam3")

    # Create nodes
    mono_inertial_node: Node = Node(
            package='ros2_orb_slam3',
            executable='dvl_stereo_node_cpp',
            name='dvl_stereo_node',
            namespace='ORB_SLAM3',
            output='screen',
            parameters=[
                {'settings_file': package_dir + '/orb_slam3/config/Stereo/RealSense_D455.yaml'},
                {'voc_file': package_dir + '/orb_slam3/Vocabulary/ORBvoc.txt.bin'},
                {'img0_topic': '/cam_realsense/camera/infra1/image_rect_raw'},
                {'img1_topic': '/cam_realsense/camera/infra2/image_rect_raw'},
                {'imu_topic': '/cam_realsense/camera/imu'},
                {'enable_debug_window': False},
                {'publish_tf': True}
            ]
    )

    return LaunchDescription([
        mono_inertial_node,
    ])

