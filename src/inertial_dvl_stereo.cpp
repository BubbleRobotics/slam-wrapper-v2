#include "ros2_orb_slam3/inertial_dvl_stereo.hpp" 

// -------- CONSTRUCTOR -------- //

InertialDvlStereoMode::InertialDvlStereoMode() :Node("inertial_dvl_stereo_node"), tf_buffer_(this->get_clock()),
     tf_listener_(tf_buffer_)
{
    // ---- NODE PARAMETERS ---- //
    RCLCPP_INFO(this->get_logger(), "\nAQUA-SLAM (inertial-dvl-stereo) NODE STARTED");

    // Declare the node parameters
    this->declare_parameter("settings_file", "file_not_set"); // path to settings file  
    this->declare_parameter("voc_file", "file_not_set"); // Needs to be overriden with appropriate file path  
    this->declare_parameter("img0_topic", ""); // topic to receive image messages
    this->declare_parameter("img1_topic", ""); // topic to receive image messages
    this->declare_parameter("imu_topic", "");
    this->declare_parameter("dvl_topic", "");  // topic DVL messages
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
    rclcpp::Parameter enableDebugWindowParam = this->get_parameter("enable_debug_window");
    enableDebugWindow = enableDebugWindowParam.as_bool();
    rclcpp::Parameter publishTfParam = this->get_parameter("publish_tf");
    publishTf_ = publishTfParam.as_bool();
    rclcpp::Parameter manualTimeSyncParam = this->get_parameter("manual_time_sync");
    manualTimeSync = manualTimeSyncParam.as_bool();
    rclcpp::Parameter imuFromYamlParam = this->get_parameter("imu_from_yaml");
    imuFromYaml = imuFromYamlParam.as_bool();

    // Debug print: confirming parameter values
    RCLCPP_INFO(this->get_logger(), "voc_file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path: %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "img0_topic: %s", img0Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "img1_topic: %s", img1Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic: %s", imuTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "dvl_topic: %s", dvlTopic.c_str());

    // ---- SUBSCRIBERS ---- //

    // Subscribe to stereo images
    img0Sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img0Topic);
    img1Sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img1Topic);

    // Set up synchroniser for the two camera topics, define max queue length
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ImgSyncPolicy;
    sync_ = std::make_shared<message_filters::Synchronizer<ImgSyncPolicy>>(ImgSyncPolicy(10), *img0Sub_, *img1Sub_);
    sync_->registerCallback(
        std::bind(
            &InertialDvlStereoMode::StereoCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    dvlSub_ = this->create_subscription<dvl_msgs::msg::DVL>(dvlTopic, rclcpp::SensorDataQoS(), std::bind(&InertialDvlStereoMode::DvlCallback, this, std::placeholders::_1));
    imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), std::bind(&InertialDvlStereoMode::ImuCallback, this, std::placeholders::_1));

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

    InitializeSLAM();
}

// --------- DESTRUCTOR --------- //

InertialDvlStereoMode::~InertialDvlStereoMode()
{
    pAgent->Shutdown();
}

// --------- INITIALIZE SLAM --------- //

void InertialDvlStereoMode::InitializeSLAM()
{
    // Watchdog, if the paths to vocabular and settings files are still not set (DOUBLECHECK)
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 

    if (img0Topic != "" && img1Topic != "" && imuTopic != "" && dvlTopic != "")
    {
        RCLCPP_INFO(this->get_logger(), "Setting to Stereo-DVL-IMU mode");
        sensorType = ORB_SLAM3::System::IMU_STEREO; 
        //sensorType = ORB_SLAM3::System::IMU_DVL_STEREO;
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Please provide all valid message paths");       
        rclcpp::shutdown();
    }

    // Initializing System object:
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enableDebugWindow);
    RCLCPP_INFO(this->get_logger(), "AQUA-SLAM (inertial-dvl-stereo) Node initialized");
}

// --------- INITIALIZING TRANSFORM ----------- //

bool InertialDvlStereoMode::InitImuCamTransform()
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

void InertialDvlStereoMode::StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img,
                                           const sensor_msgs::msg::Image::ConstSharedPtr &right_img)
{
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
    
    if (manualTimeSync)
    {
        t = this->now().seconds();
    }

    // Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackStereo(left_cv_ptr->image, right_cv_ptr->image, t);

    // Check if tracking is successful
    if(pAgent->GetTrackingState() == ORB_SLAM3::Tracking::OK)
    {
        PublishOrbSlamOutput(Tcw, left_img, left_cv_ptr);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Error tracking");
    }
}

// --------- DVL CALLBACK --------- //

void InertialDvlStereoMode::DvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg)
{
  // TODO: Implement DVL callback
  // we can use the covariance matrix and the validity bool to decide wether we put the data in the queue
  double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  RCLCPP_INFO(this->get_logger(), "DVL Data:");
  RCLCPP_INFO(this->get_logger(), "  Time: %.6f", t);
  RCLCPP_INFO(this->get_logger(), "  Velocity Valid: %s", msg->velocity_valid ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Velocity [m/s]: [%.4f, %.4f, %.4f]",
              msg->velocity.x,
              msg->velocity.y,
              msg->velocity.z);
  RCLCPP_INFO(this->get_logger(), "  Altitude [m]: %.3f",
              msg->altitude);

  const auto & cov = msg->covariance;

  if (cov.size() == 9)
  {
    RCLCPP_INFO(this->get_logger(), "  Covariance matrix (m/s)^2:");
    RCLCPP_INFO(this->get_logger(), "    [%.6e %.6e %.6e]", cov[0], cov[1], cov[2]);
    RCLCPP_INFO(this->get_logger(), "    [%.6e %.6e %.6e]", cov[3], cov[4], cov[5]);
    RCLCPP_INFO(this->get_logger(), "    [%.6e %.6e %.6e]", cov[6], cov[7], cov[8]);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "  Covariance size is %zu (expected 9)", cov.size());
  }
}

// --------- IMU CALLBACK --------- //

void InertialDvlStereoMode::ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{   
    // Buffer IMU measurements
    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    // set/update frame ID
    imuFrameId_ = imu_msg->header.frame_id;

    // if the transformation is in the config yaml, directly use imu transformation from there
    if(imuFromYaml)
    {
        // directly use imu data 
        ORB_SLAM3::IMU::Point imu_measurement(
            imu_msg->linear_acceleration.x,
            imu_msg->linear_acceleration.y,
            imu_msg->linear_acceleration.z,
            imu_msg->angular_velocity.x,
            imu_msg->angular_velocity.y,
            imu_msg->angular_velocity.z,
            t
        );

        // pass data to ORB SLAM
        pAgent->TrackIMU(t, imu_measurement);
        return;
    }

    // init transform between IMU and camera frames
    if (!InitImuCamTransform())
    {
        return; // cannot proceed without transform
    }

    if (manualTimeSync)
    {
        t = this->now().seconds();
    }
    // transform imu data to camera frame using ROS tf2 (use frame ID from IMU and camera)
    sensor_msgs::msg::Imu transformed_imu;
    // Transform linear acceleration
    geometry_msgs::msg::Vector3Stamped acc_in, acc_out;
    acc_in.header = imu_msg->header;
    acc_in.vector = imu_msg->linear_acceleration;
    tf2::doTransform(acc_in, acc_out, transformImuCam);
    
    // Transform angular velocity
    geometry_msgs::msg::Vector3Stamped gyro_in, gyro_out;
    gyro_in.header = imu_msg->header;
    gyro_in.vector = imu_msg->angular_velocity;
    tf2::doTransform(gyro_in, gyro_out, transformImuCam);

    // Create transformed IMU message
    transformed_imu.header.stamp = imu_msg->header.stamp;
    transformed_imu.header.frame_id = cameraFrameId_;
    transformed_imu.linear_acceleration = acc_out.vector;
    transformed_imu.angular_velocity = gyro_out.vector;
    transformed_imu.orientation = imu_msg->orientation;  // Copy orientation

    ORB_SLAM3::IMU::Point imu_measurement(
        transformed_imu.linear_acceleration.x,
        transformed_imu.linear_acceleration.y,
        transformed_imu.linear_acceleration.z,
        transformed_imu.angular_velocity.x,
        transformed_imu.angular_velocity.y,
        transformed_imu.angular_velocity.z,
        t
    );

    // pass data to ORB SLAM
    pAgent->TrackIMU(t, imu_measurement);
}

// ---------- PUBLISHING ON TOPICS ---------- //

void InertialDvlStereoMode::PublishOrbSlamOutput(const Sophus::SE3f& Tcw, 
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

    // Publish tracking image
    PublishTrackingImage(cv_ptr->image, img_msg);
}

void InertialDvlStereoMode::PublishPose(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = header.stamp;
    pose_msg.header.frame_id = worldFrameOrbId_;
    
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
}

void InertialDvlStereoMode::PublishOdometry(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = worldFrameOrbId_;
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
}

void InertialDvlStereoMode::PublishPath(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = header.stamp;
    pose.header.frame_id = worldFrameOrbId_;
    
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

void InertialDvlStereoMode::PublishTF(const Sophus::SE3f& Twc, const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = img_msg->header.stamp;
    transform.header.frame_id = worldFrameOrbId_;
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

void InertialDvlStereoMode::PublishTrackingImage(const cv::Mat& image, const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    // Get tracked features from ORB-SLAM3 and draw them
    cv::Mat im_with_info = image.clone();

    // Draw tracked features, keypoints, etc.
    std::vector<cv::KeyPoint> keypoints = pAgent->GetTrackedKeyPoints();
    for (size_t i = 0; i < keypoints.size(); i++)
    {
        cv::circle(im_with_info, keypoints[i].pt, 2, cv::Scalar(0, 255, 0), -1);
    }   
    
    sensor_msgs::msg::Image::SharedPtr tracking_msg = 
        cv_bridge::CvImage(img_msg->header, img_msg->encoding, im_with_info).toImageMsg();
    
    trackingImagePub_->publish(*tracking_msg);
}