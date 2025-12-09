from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get paths to existing launch files
    pkg_share_orb = get_package_share_directory('ros2_orb_slam3')
    pkg_share_realsense = get_package_share_directory('sensors_camera')
    
    realsense_launch = os.path.join(pkg_share_realsense, 'launch', 'realsense.launch.py')
    realsense_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch)
    )

    orbslam_launch   = os.path.join(pkg_share_orb, 'launch', 'orbslam.launch.py')
    orbslam_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbslam_launch)
    )

    return LaunchDescription([
        realsense_launch_include,
        orbslam_launch_include,
    ])
