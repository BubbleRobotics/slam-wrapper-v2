'''
 # @ Create Time: 2026-01-05 14:49:00
 # @ Modified by: Diego Hernandez
 # @ Modified time: 2026-01-05 14:49:00
 # @ Description: Launch file for Stereo-IMU SLAM Wrapper with DVL Late Fusion for Gazebo Simulation
 '''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # get package directory
    # package_dir = get_package_share_directory(
        # "ros2_orb_slam3")

    # Create nodes
    dvl_node: Node = Node(
            package='ros2_orb_slam3',
            executable='dvl_node_cpp',
            name='dvl_node',
            namespace='DVL_EKF',
            output='screen',
            parameters=[
                {'dvl_topic': '/dvl/twist_data'}, 
                {'odometry_est': '/ORB_SLAM3/stereo_sim_node/odometry'},
            ]
    )

    return LaunchDescription([
        dvl_node,
    ])