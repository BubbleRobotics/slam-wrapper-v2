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
    this->declare_parameter("is_inertial", true); // switch for inertial and non-inertial mode

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

    rclcpp::Parameter isInertialParam = this->get_parameter("is_inertial");
    isInertial = isInertialParam.as_bool();
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());\
    RCLCPP_INFO(this->get_logger(), "img_topic %s", imgTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic %s", imuTopic.c_str());

    // subscribe to the image messages
    imgMsgSub_= this->create_subscription<sensor_msgs::msg::Image>(imgTopic, rclcpp::SensorDataQoS(), std::bind(&MonocularInertialMode::Img_callback, this, _1));
    // subscribe to the imu messages (if eneabled)
    if (isInertial)
    {
        imuMsgSub_= this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), std::bind(&MonocularInertialMode::Imu_callback, this, _1));
    }

    //* Initialize the VSLAM framework
    InitializeVSLAM();
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
void MonocularInertialMode::InitializeVSLAM(){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    if (isInertial)
    {
        sensorType = ORB_SLAM3::System::IMU_MONOCULAR; 
    }
    else
    {
        sensorType = ORB_SLAM3::System::MONOCULAR; 
    }

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
    if (isInertial)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        // Get all IMU measurements between last image and current image
        for(auto it = imuBuffer_.begin(); it != imuBuffer_.end(); )
        {
            if(it->t <= t)
            {
                vImuMeas.push_back(*it);
                it = imuBuffer_.erase(it);
            }
            else
            {
                ++it;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Image timestamp: %.6f, IMU measurements: %zu", t, vImuMeas.size());
        // Track with IMU measurements
        Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, t, vImuMeas);
        // Check if tracking was successful and publish pose
        if(CheckSuccessfulTracking(Tcw))
        {
            PublishOrbSlamOutput(Twc, img_msg->header);
        }
    }
    else
    {
        // Track without IMU measurements
        Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, t);
        // Check if tracking was successful and publish pose
        if(CheckSuccessfulTracking(Tcw))
        {
            PublishOrbSlamOutput(Twc, img_msg->header);
        }
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
    imuBuffer_.push_back(imu_measurement);
}

bool MonocularInertialMode::CheckSuccessfulTracking(Sophus::SE3f Tcw)
{
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

void MonocularInertialMode::PublishOrbSlamOutput(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
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
    if (publish_tf_)
    {
        PublishTF(Twc, img_msg->header);
    }
    
    // Publish map points
    if (publish_pointcloud_)
    {
        PublishMapPoints(img_msg->header);
    }
    
    // Publish tracking image
    PublishTrackingImage(cv_ptr->image, img_msg->header);
}

void MonocularInertialMode::PublishPose(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
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
    
    pose_pub_->publish(pose_msg);
}

void MonocularInertialMode::PublishOdometry(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = worldFrameId_;
    odom_msg.child_frame_id = cameraFrameId_;
    
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    
    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();
    
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    // You can add velocity if available from SLAM
    // odom_msg.twist.twist.linear.x = vx;
    // odom_msg.twist.twist.angular.z = wz;
    
    odom_pub_->publish(odom_msg);
}

void MonocularInertialMode::PublishPath(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
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
    
    path_pub_->publish(path_);
}

void MonocularInertialMode::PublishTF(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = header.stamp;
    transform.header.frame_id = worldFrameId_;
    transform.child_frame_id = cameraFrameId_;
    
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    
    transform.transform.translation.x = t.x();
    transform.transform.translation.y = t.y();
    transform.transform.translation.z = t.z();
    
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transform);
}

void MonocularInertialMode::PublishMapPoints(const std_msgs::msg::Header& header)
{
    // Get map points from ORB-SLAM3
    std::vector<ORB_SLAM3::MapPoint*> vpMPs = mpSLAM->GetTrackedMapPoints();
    std::vector<ORB_SLAM3::MapPoint*> vpRefMPs = mpSLAM->GetReferenceMapPoints();
    
    std::set<ORB_SLAM3::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = header.stamp;
    cloud_msg.header.frame_id = worldFrameId_;
    
    cloud_msg.height = 1;
    cloud_msg.width = vpMPs.size();
    cloud_msg.is_dense = false;
    
    cloud_msg.fields.resize(3);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;
    
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;
    
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;
    
    cloud_msg.point_step = 12;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step);
    
    int idx = 0;
    for(size_t i = 0; i < vpMPs.size(); i++)
    {
        if(vpMPs[i] && !vpMPs[i]->isBad())
        {
            Eigen::Vector3f pos = vpMPs[i]->GetWorldPos();
            
            memcpy(&cloud_msg.data[idx * 12 + 0], &pos(0), sizeof(float));
            memcpy(&cloud_msg.data[idx * 12 + 4], &pos(1), sizeof(float));
            memcpy(&cloud_msg.data[idx * 12 + 8], &pos(2), sizeof(float));
            idx++;
        }
    }
    
    cloud_msg.width = idx;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step);
    
    pointcloud_pub_->publish(cloud_msg);
}

void MonocularInertialMode::PublishTrackingImage(const cv::Mat& image, const std_msgs::msg::Header& header)
{
    // Get tracked features from ORB-SLAM3 and draw them
    cv::Mat im_with_info = image.clone();
    
    // Draw tracked features, keypoints, etc.
    // This depends on your ORB-SLAM3 version and available methods
    // Example: draw current tracked points
    
    sensor_msgs::msg::Image::SharedPtr tracking_msg = 
        cv_bridge::CvImage(header, "bgr8", im_with_info).toImageMsg();
    
    tracking_image_pub_->publish(*tracking_msg);
}