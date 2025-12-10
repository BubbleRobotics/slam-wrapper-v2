//* Import all necessary modules
#include "ros2_orb_slam3/dvl_stereo.hpp" //* equivalent to orbslam3_ros/include/stereo_dvl.hpp

//* main
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    
    //* Declare a node object
    auto node = std::make_shared<DvlStereoMode>(); 
    
    rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}