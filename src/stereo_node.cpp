/*
* Originally adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
* Author: Diego Hernandez
* Version: 1.0
* Date: 10/11/2025
* Compatible for ROS2 Jazzy
*/

//* Import all necessary modules
#include "ros2_orb_slam3/stereo.hpp" //* equivalent to orbslam3_ros/include/common.h

//* main
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    
    //* Declare a node object
    auto node = std::make_shared<StereoMode>(); 
    
    rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}

// ------------------------------------------------------------ EOF ---------------------------------------------


