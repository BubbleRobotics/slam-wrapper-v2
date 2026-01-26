# ********************************************************************************************************************************#
#@@@@@@@@@@@@@@@@@@@@@@@             @@@@@@@@@@@ @@@       @@@ @@@@@@@@@@@ @@@@@@@@@@@@ @@@        @@@@@@@@@@                    
#@@@@@@@@@@@@@@@@@@@@@@@@@           @@       @@@@@@       @@@ @@@      @@ @@@       @@ @@@        @@                            
#@@@@@@@@@@@@@@@@@@@@@@@@@@          @@@@@@@@@@@ @@@       @@@ @@@@@@@@@@@ @@@@@@@@@@@@ @@@        @@@@@@@@@@                    
#@@@@@@@@@@@@@@@@@@@@@@@@@@          @@        @@@@@       @@@ @@@      @@ @@@       @@ @@@        @@                            
#@@@@@@@@@@@@@@@@@@@@@@@@@           @@@@@@@@@@@@ @@@@@@@@@@@  @@@@@@@@@@@ @@@@@@@@@@@@ @@@@@@@@@@ @@@@@@@@@@                    
#                      @@@                                                                                                       
#                      @@@                                                                                                       
#                    @@@@@@                                                                                                   
#@@@@@@@@@@@@@@@@@@@@@@@@@@@         @@@@@@@@@@@  @@@@@@@@@@@  @@@@@@@@@@@  @@@@@@@@@@@ @@@@@@@@@@@ @@@ @@@@@@@@@@@@ @@@@@@@@@@@@
#@@@@@@@@@@@@@@@@@@@@@@@@@@@         @@       @@@@@@       @@@ @@       @@ @@@       @@@     @@     @@@ @@        @@ @@        
#@@@@@@@@@@@@@@@@@@@@@@@@@@          @@@@@@@@@@@ @@@       @@@ @@@@@@@@@@@ @@@       @@@     @@     @@@ @@            @@@@@@@@@@@
#@@@@@@@@@@@@@@@@@@@@@@@@@@          @@      @@  @@@       @@@ @@       @@@@@@       @@@     @@     @@@ @@        @@           @@
#@@@@@@@@@@@@@@@@@@@@@@@@            @@       @@  @@@@@@@@@@@  @@@@@@@@@@@  @@@@@@@@@@@      @@     @@@ @@@@@@@@@@@  @@@@@@@@@@@@
#
#   stereo.launch.py
#
#   Description: Launch file for stereo/stereo-inertial ORB-SLAM3 ROS2 wrapper node
#
#   By: Diego Hernandez <dhernandez@ethz.ch>
#
#   Created: 2026/01/26 15:50:15 by Diego Hernandez
#   Updated: 2026/01/26 15:50:15 by Diego Hernandez
#
#********************************************************************************************************************************#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # get package directory
    package_dir = get_package_share_directory("ros2_orb_slam3")

    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Set to true to use simulation topics'
    )

    use_inertial_arg = DeclareLaunchArgument(
        'use_inertial',
        default_value='true',
        description='Set to true to use inertial data'
    )

    use_sim = LaunchConfiguration('use_sim')
    use_inertial = LaunchConfiguration('use_inertial')

    # Realsense Camera Node
    stereo_real_node = Node(
            package='ros2_orb_slam3',
            executable='stereo_node_cpp',
            name='stereo_node',
            namespace='ORB_SLAM3',
            output='screen',
            parameters=[
                {'settings_file': package_dir + '/orb_slam3/config/Stereo/RealSense_D455.yaml'},
                {'voc_file': package_dir + '/orb_slam3/Vocabulary/ORBvoc.txt.bin'},
                {'img0_topic': '/cam_realsense/camera/infra1/image_rect_raw'},
                {'img1_topic': '/cam_realsense/camera/infra2/image_rect_raw'},
                {'imu_topic': '/cam_realsense/camera/imu'},
                {'enable_debug_window': True},
                {'is_inertial': use_inertial},
                {'manual_time_sync': False},
                {'imu_from_yaml': False},
            ],
            condition=UnlessCondition(use_sim)
    )

    # Simulation Node
    stereo_sim_node = Node(
            package='ros2_orb_slam3',
            executable='stereo_node_cpp',
            name='stereo_sim_node',
            namespace='ORB_SLAM3',
            output='screen',
            parameters=[
                {'settings_file': package_dir + '/orb_slam3/config/Stereo/Gazebo.yaml'},
                {'voc_file': package_dir + '/orb_slam3/Vocabulary/ORBvoc.txt.bin'},
                {'img0_topic': '/camera_d455/ir_left/image_raw'},
                {'img1_topic': '/camera_d455/ir_right/image_raw'},
                {'imu_topic': '/vectornav/Imu_raw'},
                {'enable_debug_window': True},
                {'is_inertial': use_inertial},
                {'manual_time_sync': False},
                {'imu_from_yaml': False},
            ],
            condition=IfCondition(use_sim)
    )

    return LaunchDescription([
        use_sim_arg,
        use_inertial_arg,
        stereo_real_node,
        stereo_sim_node
    ])

