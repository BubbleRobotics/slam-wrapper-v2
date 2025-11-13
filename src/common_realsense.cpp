/**
 * @ Author: Diego Hernandez Est
 * @ Create Time: 2025-11-13 10:42:00
 * @ Modified by: Diego Hernandez Estrada
 * @ Modified time: 2025-11-13 10:42:00
 * @ Description: ROS2 node for Stereo-Inertial VSLAM using ORB-SLAM3
 */


//* Includes
#include "ros2_orb_slam3/common_realsense.hpp"

//* Constructor
RealsenseMode::RealsenseMode() :Node("realsense_node")
{
    homeDir = getenv("HOME");
    packagePath = "ws_blue/src/slam-wrapper-v2/";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 (Realsense) NODE STARTED");

    this->declare_parameter("node_name", "not_given"); // Name of this agent
    this->declare_parameter("voc_file", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file", "file_path_not_set"); // path to settings file  
    this->declare_parameter("img1_topic", "/cam_realsense/camera/infra1/image_rect_raw"); // topic to receive image messages
    this->declare_parameter("img2_topic", "/cam_realsense/camera/infra2/image_rect_raw"); // topic to receive image messages
    this->declare_parameter("imu_topic", "/cam_realsense/camera/imu");
    this->declare_parameter("enable_debug_window", true); // Enable debug window showing SLAM in pangolin/opencv

    //* Populate parameter values
    rclcpp::Parameter nodeNameParam = this->get_parameter("node_name");
    nodeName = nodeNameParam.as_string();
    
    rclcpp::Parameter vocFilePathParam = this->get_parameter("voc_file");
    vocFilePath = vocFilePathParam.as_string();

    rclcpp::Parameter settingsFilePathParam = this->get_parameter("settings_file");
    settingsFilePath = settingsFilePathParam.as_string();

    rclcpp::Parameter img1TopicParam = this->get_parameter("img1_topic");
    img1Topic = img1TopicParam.as_string();

    rclcpp::Parameter img2TopicParam = this->get_parameter("img2_topic");
    img2Topic = img2TopicParam.as_string();

    rclcpp::Parameter imuTopicParam = this->get_parameter("imu_topic");
    imuTopic = imuTopicParam.as_string();

    rclcpp::Parameter enableDebugWindowParam = this->get_parameter("enable_debug_window");
    enableDebugWindow = enableDebugWindowParam.as_bool();
    
    // Make sure that the file path is given and that it exists
    if (settingsFilePath == "file_path_not_set"  || !std::filesystem::exists(settingsFilePath))
    {
        RCLCPP_FATAL(this->get_logger(),"Invalid settings file path: %s", settingsFilePath.c_str());
        throw std::runtime_error("Invalid settings file path");
    }

    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set")
    {
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    }

    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "img1_topic %s", img1Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "img2_topic %s", img2Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic %s", imuTopic.c_str());

    initializeVISLAM();

    // subscribe to the imu messages
    imuMsgSub_= this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), std::bind(&RealsenseMode::imu_callback, this, std::placeholders::_1));

    //set up stereo subscribers with message_filters
    left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img1Topic);
    right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img2Topic);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), *left_sub_, *right_sub_);
    sync_->registerCallback(std::bind(&RealsenseMode::stereo_callback, this, std::placeholders::_1, std::placeholders::_2));
}

//* Destructor
RealsenseMode::~RealsenseMode()
{   
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
}

//* Method to bind an initialized VI-SLAM framework to this node
void RealsenseMode::initializeVISLAM(){
    
    // Watchdog, if the paths to vocabular and settings files are still not set (DOUBLECHECK)
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    sensorType = ORB_SLAM3::System::IMU_STEREO;

    if (enableDebugWindow)
    {
        enablePangolinWindow = true; // Shows Pangolin window output
        enableOpenCVWindow = true; // Shows OpenCV window output  
    }
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Stereo Node initialized");
}

//* Callback to process image messages and run Stereo-SLAM node
void RealsenseMode::stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img,
                                    const sensor_msgs::msg::Image::ConstSharedPtr &right_img)
{
    cv_bridge::CvImageConstPtr left_cv_ptr, right_cv_ptr; 
    try
    {
        left_cv_ptr = cv_bridge::toCvShare(left_img); 
        right_cv_ptr = cv_bridge::toCvShare(right_img); 
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    double t = left_img->header.stamp.sec + left_img->header.stamp.nanosec * 1e-9;

    // Get IMU measurement for this frame
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        
        // Get all IMU measurements between last image and current image
        for(auto it = imu_buffer_.begin(); it != imu_buffer_.end(); )
        {
            if(it->t <= t)
            {
                // Delete all used values
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
    //* Perform all ORB-SLAM3 operations in Stereo mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackStereo(left_cv_ptr->image, right_cv_ptr->image, t, vImuMeas);

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

void RealsenseMode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    // Buffer IMU measurements
    //RCLCPP_INFO(this->get_logger(), "Incoming IMU message");
    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    
    ORB_SLAM3::IMU::Point imu_measurement(
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z,
        imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z,
        t
    ); // This is defined in orb_slam3/include/ImuTypes.h (l. 46-59)
    
    // Add to buffer (thread-safe with mutex)
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.push_back(imu_measurement);
}
