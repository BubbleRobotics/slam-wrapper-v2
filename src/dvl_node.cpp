/*
* Author: Diego Hernandez
* Version: 1.0
* Date: 05/01/2026
* Compatible for ROS2 Jazzy
*/

//* Import all necessary modules
#include "ros2_orb_slam3/dvl.hpp" 

//* main
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    
    //* Declare a node object
    auto node = std::make_shared<DVLMode>(); 
    
    rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}

// ------------------------------------------------------------ EOF ---------------------------------------------

