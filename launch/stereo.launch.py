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

    # Set the standard deviation for the DVL velocity measurement
    declare_dvl_std = DeclareLaunchArgument(
        "dvl_std", 
        default_value="0.0101",
        description="Assumed measurement noise of DVL determines its influence in sensor fusion"    
    )

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
                {'settings_file': package_dir + '/orb_slam3/config/Stereo/paramsAquaSlam.yaml'},
                {'voc_file': package_dir + '/orb_slam3/Vocabulary/ORBvoc.txt.bin'},
                {'img0_topic': '/camera/left/image_dehazed/raw'},
                {'img1_topic': '/camera/right/image_dehazed/raw'},
                {'imu_topic': '/imu/data'},
                {'enable_debug_window': True},
                {'is_inertial': True},
                {'manual_time_sync': False},
                {'imu_from_yaml': True},
                {'dvl_std': LaunchConfiguration("dvl_std")}
            ]
    )

    return LaunchDescription([
        declare_dvl_std,
        mono_inertial_node,
    ])

