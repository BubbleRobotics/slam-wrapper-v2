#include "ros2_orb_slam3/dvl_stereo.hpp" // equivalent to orbslam3_ros/include/stereo_dvl.hpp

// CONSTRUCTOR
DvlStereoMode::DvlStereoMode()
    // initialiser list: initialising parent node class
    : Node("dvl_stereo_node")
{
    //Declare the node parameters
    this->declare_parameter("voc_file", "file_not_set"); // Needs to be overriden with appropriate file path  
    this->declare_parameter("settings_file", "file_not_set"); // path to settings file  
    this->declare_parameter("img0_topic", "/camera/left/image_dehazed/raw"); // topic to receive image messages
    this->declare_parameter("img1_topic", "/camera/right/image_dehazed/raw"); // topic to receive image messages
    this->declare_parameter("imu_topic", "/imu/data"); // topic IMU messages
    this->declare_parameter("dvl_topic", "/dvl/data");  // topic DVL messages
    this->declare_parameter("enable_debug_window", true); // Enable debug window showing SLAM in pangolin/opencv
    this->declare_parameter<bool>("publish_tf", true);  
    
    //Put parameter values into member variables too
    rclcpp::Parameter vocFilePathParam = this->get_parameter("voc_file");
    vocFilePath = vocFilePathParam.as_string();
    rclcpp::Parameter settingsFilePathParam = this->get_parameter("settings_file");
    settingsFilePath = settingsFilePathParam.as_string();
    rclcpp::Parameter img0TopicParam = this->get_parameter("img0_topic");
    img0Topic = img0TopicParam.as_string();
    rclcpp::Parameter img1TopicParam = this->get_parameter("img1_topic");
    img1Topic = img1TopicParam.as_string();
    rclcpp::Parameter imuTopicParam = this->get_parameter("imu_topic");
    imuTopic = imuTopicParam.as_string();
    rclcpp::Parameter enableDebugWindowParam = this->get_parameter("enable_debug_window");
    enableDebugWindow = enableDebugWindowParam.as_bool();
    rclcpp::Parameter publishTfParam = this->get_parameter("publish_tf");
    publishTf_ = publishTfParam.as_bool();
    rclcpp::Parameter dvlTopicParam = this->get_parameter("dvl_topic");
    dvlTopic = dvlTopicParam.as_string();

    //Debug print: confirming parameter values
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "img0_topic %s", img0Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "img1_topic %s", img1Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic %s", imuTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "dvl_topic %s", dvlTopic.c_str());
};

// DESTRUCTOR
DvlStereoMode::~DvlStereoMode(){}