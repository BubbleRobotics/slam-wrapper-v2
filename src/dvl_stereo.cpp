#include "ros2_orb_slam3/dvl_stereo.hpp" // equivalent to orbslam3_ros/include/stereo_dvl.hpp

// -------- CONSTRUCTOR -------- //

DvlStereoMode::DvlStereoMode()
    
    // initialiser list: initialising parent node class
    : Node("dvl_stereo_node")

{
    // ---- NODE PARAMETERS ---- //
    
    // Declare the node parameters
    this->declare_parameter("settings_file", "file_not_set"); // path to settings file  
    this->declare_parameter("voc_file", "file_not_set"); // Needs to be overriden with appropriate file path  
    this->declare_parameter("img0_topic", "/camera/left/image_dehazed/raw"); // topic to receive image messages
    this->declare_parameter("img1_topic", "/camera/right/image_dehazed/raw"); // topic to receive image messages
    this->declare_parameter("imu_topic", "/vectornav/Imu_raw");
    this->declare_parameter("dvl_topic", "/dvl/data");  // topic DVL messages
    this->declare_parameter("is_dvlused", false); // switch for inertial and non-inertial mode
    this->declare_parameter("is_inertial", false);
    this->declare_parameter("enable_debug_window", true); // Enable debug window showing SLAM in pangolin/opencv
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("manual_time_sync", false);
    this->declare_parameter("imu_from_yaml", false);

    // Put parameter values into member variables too
    rclcpp::Parameter settingsFilePathParam = this->get_parameter("settings_file");
    settingsFilePath = settingsFilePathParam.as_string();
    rclcpp::Parameter vocFilePathParam = this->get_parameter("voc_file");
    vocFilePath = vocFilePathParam.as_string();
    rclcpp::Parameter img0TopicParam = this->get_parameter("img0_topic");
    img0Topic = img0TopicParam.as_string();
    rclcpp::Parameter img1TopicParam = this->get_parameter("img1_topic");
    img1Topic = img1TopicParam.as_string();
    rclcpp::Parameter imuTopicParam = this->get_parameter("imu_topic");
    imuTopic = imuTopicParam.as_string();
    rclcpp::Parameter dvlTopicParam = this->get_parameter("dvl_topic");
    dvlTopic = dvlTopicParam.as_string();
    rclcpp::Parameter isDVLParam = this->get_parameter("is_dvlused");
    isDVLUsed = isDVLParam.as_bool();
    rclcpp::Parameter isIMUParam = this->get_parameter("is_inertial");
    isIMUUsed = isIMUParam.as_bool();
    rclcpp::Parameter enableDebugWindowParam = this->get_parameter("enable_debug_window");
    enableDebugWindow = enableDebugWindowParam.as_bool();
    rclcpp::Parameter publishTfParam = this->get_parameter("publish_tf");
    publishTf_ = publishTfParam.as_bool();
    rclcpp::Parameter manualTimeSyncParam = this->get_parameter("manual_time_sync");
    manualTimeSync_ = manualTimeSyncParam.as_bool();
    rclcpp::Parameter imuFromYamlParam = this->get_parameter("imu_from_yaml");
    imuFromYaml_ = imuFromYamlParam.as_bool();

    // Debug print: confirming parameter values
    RCLCPP_INFO(this->get_logger(), "voc_file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path: %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "img0_topic: %s", img0Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "img1_topic: %s", img1Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "dvl_topic: %s", dvlTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic: %s", imuTopic.c_str());

    // ---- SUBSCRIBERS ---- //

    // Subscribe to stereo images
    img0Sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img0Topic);
    img1Sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img1Topic);

    // Set up synchroniser for the two camera topics, define max queue length
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ImgSyncPolicy;
    sync_ = std::make_shared<message_filters::Synchronizer<ImgSyncPolicy>>(ImgSyncPolicy(10), *img0Sub_, *img1Sub_);
    sync_->registerCallback(
        std::bind(
            &DvlStereoMode::StereoCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    if (isDVLUsed)
    {
        dvlSub_ = this->create_subscription<dvl_msgs::msg::DVL>(dvlTopic, rclcpp::SensorDataQoS(), std::bind(&DvlStereoMode::DvlCallback, this, std::placeholders::_1));
    }

    if (isIMUUsed)
    {
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), std::bind(&DvlStereoMode::ImuCallback, this, std::placeholders::_1));
    }

    // ---- PUBLISHERS ---- //

    posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/camera_pose", 10);

    odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "~/odometry", 10);

    pathPub_ = this->create_publisher<nav_msgs::msg::Path>(
        "~/trajectory", 10);

    trackingImagePub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "~/tracking_image", 10);
    
    // TF broadcaster
    if (publishTf_) {
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    InitializeVSLAM();

};

// --------- DESTRUCTOR --------- //

DvlStereoMode::~DvlStereoMode(){
    pAgent->Shutdown();
}

// --------- INITIALIZE VSLAM --------- //

void DvlStereoMode::InitializeVSLAM(){

    // Watchdog, if the paths to vocabular and settings files are still not set (DOUBLECHECK)
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 

    if (isDVLUsed && isIMUUsed)
    {
        RCLCPP_INFO(this->get_logger(), "Setting to inertial DVL-IMU-Stereo mode");
        sensorType = ORB_SLAM3::System::DVL_IMU_STEREO;
    }
    else if (isIMUUsed)
    {
        RCLCPP_INFO(this->get_logger(), "Setting to IMU-stereo mode");
        sensorType = ORB_SLAM3::System::IMU_STEREO;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Setting to stereo mode");
        sensorType = ORB_SLAM3::System::STEREO;
    }

    if (enableDebugWindow)
    {
        enablePangolinWindow = true; // Shows Pangolin window output
        enableOpenCVWindow = true; // Shows OpenCV window output  
    }

    // Initializing System object:
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Stereo Node initialized");
};

bool StereoMode::InitImuCamTransform()
{
    // check if frame IDs are set
    if (cameraFrameId_ == "" || imuFrameId_ == "")
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Camera or IMU frame ID is empty, cannot lookup transform");
        RCLCPP_INFO(this->get_logger(), "cameraFrameId_: %s", cameraFrameId_.c_str());
        RCLCPP_INFO(this->get_logger(), "imuFrameId_: %s", imuFrameId_.c_str());
        return false;
    }

    // check if transform is already set
    if (transformImuCam.header.frame_id == cameraFrameId_ &&
        transformImuCam.child_frame_id == imuFrameId_)
    {
        return true;
    }

    try {
        transformImuCam = tf_buffer_.lookupTransform(
                cameraFrameId_, 
                imuFrameId_,
                tf2::TimePointZero);  // Get latest available transform
        RCLCPP_INFO(this->get_logger(), "Successfully looked up transform between IMU and Camera frames");
        RCLCPP_INFO(this->get_logger(), "cameraFrameId_: %s", cameraFrameId_.c_str());
        RCLCPP_INFO(this->get_logger(), "imuFrameId_: %s", imuFrameId_.c_str());
        RCLCPP_INFO(this->get_logger(), "transform: %f", transformImuCam.transform.translation.x);
        RCLCPP_INFO(this->get_logger(), "transform: %f", transformImuCam.transform.translation.y);
        RCLCPP_INFO(this->get_logger(), "transform: %f", transformImuCam.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "transform: %f", transformImuCam.transform.rotation.x);
        RCLCPP_INFO(this->get_logger(), "transform: %f", transformImuCam.transform.rotation.y);
        RCLCPP_INFO(this->get_logger(), "transform: %f", transformImuCam.transform.rotation.z);

        return true;
            
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Could not transform IMU to Camera frame: %s", ex.what());
        return false;
    }
}

// --------- STEREO CALLBACK --------- //

void DvlStereoMode::StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img,
                                   const sensor_msgs::msg::Image::ConstSharedPtr &right_img){

    // take left image as frame ID 
    cameraFrameId_ = left_img->header.frame_id;
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
    
    // Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackStereo(left_cv_ptr->image, right_cv_ptr->image, t);

    // Check if pipeline predicts a zero pose
    if(pAgent->GetTrackingState() == ORB_SLAM3::Tracking::OK)
    {
        // RCLCPP_INFO(this->get_logger(), "Successful tracking");
        PublishOrbSlamOutput(Tcw, left_img, left_cv_ptr);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Error tracking");
    }
};

// --------- DVL CALLBACK --------- //

void DvlStereoMode::DvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg){
    // DVL Message
    RCLCPP_INFO(this->get_logger(), "DVL Message received");
    RCLCPP_INFO(this->get_logger(), "velocity x: %f", msg->velocity.x);
    RCLCPP_INFO(this->get_logger(), "velocity y: %f", msg->velocity.y);
    RCLCPP_INFO(this->get_logger(), "velocity z: %f", msg->velocity.z);
    RCLCPP_INFO(this->get_logger(), "sec: %f", msg->header.stamp.sec);
    RCLCPP_INFO(this->get_logger(), "nanosec: %f", msg->header.stamp.nanosec);
};

// ---------- PUBLISHING ON TOPICS ---------- //

void DvlStereoMode::PublishOrbSlamOutput(const Sophus::SE3f& Tcw, 
                                         const sensor_msgs::msg::Image::ConstSharedPtr img_msg,
                                         const cv_bridge::CvImageConstPtr& cv_ptr)
{
    // Convert from camera-to-world to world-to-camera
    Sophus::SE3f Twc = Tcw.inverse();
    
    // Publish pose
    PublishPose(Twc, img_msg->header);
    
    // Publish odometry
    PublishOdometry(Twc, img_msg->header);

    // Publish path
    PublishPath(Twc, img_msg->header);

    // Publish TF
    if (publishTf_)
    {
        PublishTF(Twc, img_msg);
    }
};

void DvlStereoMode::PublishPose(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = header.stamp;
    pose_msg.header.frame_id = worldFrameId_;
    
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    posePub_->publish(pose_msg);
};

void DvlStereoMode::PublishOdometry(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = worldFrameId_;
    odom_msg.child_frame_id = cameraFrameOrbId_;
    
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    
    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();
    
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    odomPub_->publish(odom_msg);
};

void DvlStereoMode::PublishPath(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = header.stamp;
    pose.header.frame_id = worldFrameId_;
    
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    
    pose.pose.position.x = t.x();
    pose.pose.position.y = t.y();
    pose.pose.position.z = t.z();
    
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    path_.poses.push_back(pose);
    path_.header = pose.header;

    pathPub_->publish(path_);
}

void DvlStereoMode::PublishTF(const Sophus::SE3f& Twc, const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = img_msg->header.stamp;
    transform.header.frame_id = worldFrameId_;
    transform.child_frame_id = cameraFrameOrbId_;
    
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    
    transform.transform.translation.x = t.x();
    transform.transform.translation.y = t.y();
    transform.transform.translation.z = t.z();
    
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    tfBroadcaster_->sendTransform(transform);
}