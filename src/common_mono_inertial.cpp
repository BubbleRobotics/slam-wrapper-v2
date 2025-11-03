/**
 * @ Author: Paul Joseph
 * @ Create Time: 2025-10-31 13:56:42
 * @ Modified by: Paul Joseph
 * @ Modified time: 2025-11-03 12:24:24
 * @ Description: ROS2 node for Monocular-Inertial VSLAM using ORB-SLAM3
 */


//* Includes
#include "ros2_orb_slam3/common_mono_inertial.hpp"

//* Constructor
MonocularInertialMode::MonocularInertialMode() :Node("mono_inertial_node")
{
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 (mono-inertial) NODE STARTED");

    this->declare_parameter("node_name", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file", "file_path_not_set"); // path to settings file  
    this->declare_parameter("img_topic", "/camera/left/image_raw"); // topic to receive image messages
    this->declare_parameter("imu_topic", "/imu/data"); // topic to receive IMU messages
    this->declare_parameter("enable_debug_window", true); // Enable debug window showing SLAM in pangolin/opencv

    //* Populate parameter values
    rclcpp::Parameter nodeNameParam = this->get_parameter("node_name");
    nodeName = nodeNameParam.as_string();
    
    rclcpp::Parameter vocFilePathParam = this->get_parameter("voc_file");
    vocFilePath = vocFilePathParam.as_string();

    rclcpp::Parameter settingsFilePathParam = this->get_parameter("settings_file");
    settingsFilePath = settingsFilePathParam.as_string();

    rclcpp::Parameter imgTopicParam = this->get_parameter("img_topic");
    imgTopic = imgTopicParam.as_string();

    rclcpp::Parameter imuTopicParam = this->get_parameter("imu_topic");
    imuTopic = imuTopicParam.as_string();

    rclcpp::Parameter enableDebugWindowParam = this->get_parameter("enable_debug_window");
    enableDebugWindow = enableDebugWindowParam.as_bool();
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());\
    RCLCPP_INFO(this->get_logger(), "img_topic %s", imgTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic %s", imuTopic.c_str());

    // subscribe to the image messages
    imgMsgSub_= this->create_subscription<sensor_msgs::msg::Image>(imgTopic, rclcpp::SensorDataQoS(), std::bind(&MonocularInertialMode::Img_callback, this, _1));
    // subscribe to the imu messages
    imuMsgSub_= this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), std::bind(&MonocularInertialMode::Imu_callback, this, _1));

    //* Initialize the VSLAM framework
    initializeVSLAM();
}

//* Destructor
MonocularInertialMode::~MonocularInertialMode()
{   
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;
}

//* Method to bind an initialized VSLAM framework to this node
void MonocularInertialMode::initializeVSLAM(){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::IMU_MONOCULAR; 

    if (enableDebugWindow)
    {
        enablePangolinWindow = true; // Shows Pangolin window output
        enableOpenCVWindow = true; // Shows OpenCV window output
    }

    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularInertialMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback to process image message and run SLAM node
void MonocularInertialMode::Img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    double t = img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9;
    
    // Get IMU measurements for this frame
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        
        // Get all IMU measurements between last image and current image
        for(auto it = imu_buffer_.begin(); it != imu_buffer_.end(); )
        {
            if(it->t <= t)
            {
                vImuMeas.push_back(*it);
                it = imu_buffer_.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Image timestamp: %.6f, IMU measurements: %zu", t, vImuMeas.size());

    // Track with IMU measurements
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, t, vImuMeas);
    
    // Check if tracking was successful
    if (!Tcw.translation().isZero(1e-6))
    {
        // Get rotation and translation
        Eigen::Matrix3f R = Tcw.rotationMatrix();
        Eigen::Vector3f trans = Tcw.translation();
        
        // Publish your pose here
        RCLCPP_INFO(this->get_logger(), "Pose: [%.3f, %.3f, %.3f]", 
                   trans(0), trans(1), trans(2));
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Tracking lost!");
    }
}

void MonocularInertialMode::Imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    // Buffer IMU measurements
    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    
    ORB_SLAM3::IMU::Point imu_measurement(
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z,
        imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z,
        t
    );
    
    // Add to buffer (thread-safe with mutex)
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.push_back(imu_measurement);
}


