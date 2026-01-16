/* *************************************************************************** */
/*                                                    ########  ########       */
/*   stereo.cpp                                       ##     ## ##     ##      */
/*                                                    ##     ## ##     ##      */
/*   By: Paul Joseph <paul@bubble-robotics.com>       ########  ########       */
/*                                                    ##     ## ##   ##        */
/*   Created: 2025/11/13 17:07:03 by Paul Joseph      ##     ## ##    ##       */
/*   Updated: 2025/11/13 17:07:03 by Paul Joseph      ########  ##     ##      */
/*                                                                             */
/* *************************************************************************** */


//* Includes
#include "ros2_orb_slam3/stereo.hpp"

//* Constructor
StereoMode::StereoMode() :Node("realsense_node"), tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 (stereo-inertial) NODE STARTED");

    this->declare_parameter("node_name", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file", "file_path_not_set"); // path to settings file  
    this->declare_parameter("img0_topic", "/camera/left/image_raw"); // topic to receive image messages
    this->declare_parameter("img1_topic", "/camera/right/image_raw"); // topic to receive image messages
    this->declare_parameter("imu_topic", "/imu/data"); // topic to receive IMU messages
    this->declare_parameter("enable_debug_window", true); // Enable debug window showing SLAM in pangolin/opencv
    this->declare_parameter("is_inertial", true); // switch for inertial and non-inertial mode
    this->declare_parameter("manual_time_sync", false); // switch for manual time synchronization
    this->declare_parameter("imu_from_yaml", false); // switch for getting imu params from yaml instead of tf2
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<bool>("publish_pointcloud", true);

    //* Populate parameter values
    rclcpp::Parameter nodeNameParam = this->get_parameter("node_name");
    nodeName = nodeNameParam.as_string();
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
    rclcpp::Parameter isInertialParam = this->get_parameter("is_inertial");
    isInertial = isInertialParam.as_bool();
    rclcpp::Parameter publishTfParam = this->get_parameter("publish_tf");
    publishTf_ = publishTfParam.as_bool();
    rclcpp::Parameter publishPointcloudParam = this->get_parameter("publish_pointcloud");
    publishPointcloud_ = publishPointcloudParam.as_bool();
    rclcpp::Parameter manualTimeSyncParam = this->get_parameter("manual_time_sync");
    manualTimeSync = manualTimeSyncParam.as_bool();
    rclcpp::Parameter imuFromYamlParam = this->get_parameter("imu_from_yaml");
    imu_from_yaml = imuFromYamlParam.as_bool();

    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "img0_topic %s", img0Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "img1_topic %s", img1Topic.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic %s", imuTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "is_inertial %b", isInertial);
    RCLCPP_INFO(this->get_logger(), "manual_time_sync %b", manualTimeSync);
    RCLCPP_INFO(this->get_logger(), "imu_from_yaml %b", imu_from_yaml);


    //set up stereo subscribers with message_filters
    img0Sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img0Topic);
    img1Sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, img1Topic);

    // sync subs for images
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ImgSyncPolicy;
    sync_ = std::make_shared<message_filters::Synchronizer<ImgSyncPolicy>>(ImgSyncPolicy(10), *img0Sub_, *img1Sub_);
    sync_->registerCallback(std::bind(&StereoMode::StereoCallback, this, std::placeholders::_1, std::placeholders::_2));

    // subscribe to the imu messages (if eneabled)
    if (isInertial)
    {
        imuSub_= this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), std::bind(&StereoMode::ImuCallback, this, _1));
    }

    // Create publishers
    posePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/camera_pose", 10);

    odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "~/odometry", 10);

    pathPub_ = this->create_publisher<nav_msgs::msg::Path>(
        "~/trajectory", 10);

    gtPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "~/ground_truth", 10);

    if (publishPointcloud_) {
        pointcloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/map_points", 10);
    }

    trackingImagePub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "~/tracking_image", 10);
    
    // TF broadcaster
    if (publishTf_) {
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    // Start of VSLAM
    InitializeVSLAM();
    has_initial_alignment_ = false;
}

//* Destructor
StereoMode::~StereoMode()
{   
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
}

//* Method to bind an initialized VSLAM framework to this node
void StereoMode::InitializeVSLAM(){
    
    // Watchdog, if the paths to vocabular and settings files are still not set (DOUBLECHECK)
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    if (isInertial)
    {
        RCLCPP_INFO(this->get_logger(), "Setting to inertial mode");
        sensorType = ORB_SLAM3::System::IMU_STEREO;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Setting to non-inertial mode");
        sensorType = ORB_SLAM3::System::STEREO;
    }

    if (enableDebugWindow)
    {
        enablePangolinWindow = true; // Shows Pangolin window output
        enableOpenCVWindow = true; // Shows OpenCV window output  
    }
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    pAgent->mpAtlas->GetCurrentMap()->SetMapTransformCallback([this](const Sophus::SE3f& T_map_new, float scale) {
        this->OnOrbMapTransformed(T_map_new, scale);
    });
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Stereo Node initialized");
}

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


//* Callback to process image messages and run Stereo-SLAM node
void StereoMode::StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img,
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

    Sophus::SE3f Tcw = pAgent->TrackStereo(left_cv_ptr->image, right_cv_ptr->image, t);
    Sophus::SE3f Twc = Tcw.inverse();

    // check if it was successful and publish data
    auto state = pAgent->GetTrackingState();
    if(state == ORB_SLAM3::Tracking::OK ||
       state == ORB_SLAM3::Tracking::RECENTLY_LOST  ||
       state == ORB_SLAM3::Tracking::NOT_INITIALIZED)
    {
        nav_msgs::msg::Odometry msg;
        
        msg.header.stamp = left_img->header.stamp; 
        msg.header.frame_id = worldFrameId_;
        msg.child_frame_id = cameraFrameId_;

        Eigen::Vector3f wtwc = Twc.translation();
        msg.pose.pose.position.x = wtwc[0];
        msg.pose.pose.position.y = wtwc[1];
        msg.pose.pose.position.z = wtwc[2];

        Eigen::Quaternionf qwc = Twc.unit_quaternion();
        msg.pose.pose.orientation.x = qwc.x();
        msg.pose.pose.orientation.y = qwc.y();
        msg.pose.pose.orientation.z = qwc.z();
        msg.pose.pose.orientation.w = qwc.w();

        odomPub_->publish(msg);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "System not in state OK");
    }
}


void StereoMode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{   
    // Buffer IMU measurements
    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    // set/update frame ID
    imuFrameId_ = imu_msg->header.frame_id;

    // if the transformation is in the config yaml, directly use imu transformation from there
    if(imu_from_yaml)
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

// --------- NOT USED IN BASIC EVALUATION ---------- //

void StereoMode::PublishOrbSlamOutput(const Sophus::SE3f& T_orbw2orbcam, 
                                      const sensor_msgs::msg::Image::ConstSharedPtr img_msg,
                                      const cv_bridge::CvImageConstPtr& cv_ptr)
{
    if (!has_initial_alignment_) {
        return;
    }

    Sophus::SE3f T_orbcam2gzbw;
    {
        std::lock_guard<std::mutex> lock(mutex_alignment_);
        T_orbcam2gzbw = T_orbw2gzbw * T_orbw2orbcam;
    }
    
    // Publish pose
    PublishPose(T_orbw2orbcam, img_msg->header);

    // Publish odometry
    PublishOdometry(T_orbcam2gzbw, img_msg->header);

    // Publish path
    PublishPath(T_orbcam2gzbw, img_msg->header);

    // Publish TF
    if (publishTf_)
    {
        rclcpp::Time stamp = img_msg->header.stamp;
        PublishWorldToOrbMapTF(stamp);
        PublishOrbMapToOrbCamTF(T_orbw2orbcam, stamp);
    }
    
    // Publish map points
    if (publishPointcloud_)
    {
        PublishMapPoints(img_msg->header);
    }
    
    // Publish tracking image
    PublishTrackingImage(cv_ptr->image, img_msg);
}

void StereoMode::PublishPose(const Sophus::SE3f& T_orbw2orbcam, const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = header.stamp;
    pose_msg.header.frame_id = worldFrameId_;
    
    Eigen::Vector3f t = T_orbw2orbcam.translation();
    Eigen::Quaternionf q = T_orbw2orbcam.unit_quaternion();
    
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    posePub_->publish(pose_msg);
}

void StereoMode::PublishOdometry(const Sophus::SE3f& T_orbcam2gzbw, const std_msgs::msg::Header& header)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id =  worldGazeboFrameId_;
    odom_msg.child_frame_id = cameraFrameOrbId_;

    Eigen::Vector3f t = T_orbcam2gzbw.translation();
    Eigen::Quaternionf q = T_orbcam2gzbw.unit_quaternion();
    
    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();
    
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    odomPub_->publish(odom_msg);
    
    try {
        // lookup latest available transform from 'map' to the realsense frame
        geometry_msgs::msg::TransformStamped tf_map_realsense =
            tf_buffer_.lookupTransform(worldGazeboFrameId_, realsenseFrameId_, tf2::TimePointZero);
         nav_msgs::msg::Odometry gt_odom;
         // Use image timestamp so SLAM outputs remain time-aligned
         gt_odom.header.stamp = header.stamp;
         gt_odom.header.frame_id = worldGazeboFrameId_;
         gt_odom.child_frame_id = realsenseFrameId_;
 
         gt_odom.pose.pose.position.x = tf_map_realsense.transform.translation.x;
         gt_odom.pose.pose.position.y = tf_map_realsense.transform.translation.y;
         gt_odom.pose.pose.position.z = tf_map_realsense.transform.translation.z;
         gt_odom.pose.pose.orientation = tf_map_realsense.transform.rotation;

         gtPub_->publish(gt_odom);
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Could not lookup map->%s transform: %s.",
            realsenseFrameId_.c_str(), ex.what());
    }
}

void StereoMode::PublishPath(const Sophus::SE3f& T_orbcam2gzbw, const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = header.stamp;
    pose.header.frame_id = worldGazeboFrameId_;
    
    Eigen::Vector3f t = T_orbcam2gzbw.translation();
    Eigen::Quaternionf q = T_orbcam2gzbw.unit_quaternion();
    
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

void StereoMode::PublishMapPoints(const std_msgs::msg::Header& header)
{
    // Get map points from ORB-SLAM3
    std::vector<ORB_SLAM3::MapPoint*> vpMPs = pAgent->GetTrackedMapPoints();
    std::vector<ORB_SLAM3::MapPoint*> vpRefMPs = pAgent->GetAllMapPoints();
    
    std::set<ORB_SLAM3::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = header.stamp;
    cloud_msg.header.frame_id = worldGazeboFrameId_;
    
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
            Eigen::Vector3f pos_map = vpMPs[i]->GetWorldPos();

            // transform to Gazebo world frame
            Eigen::Vector3f pos_world = pos_map;
            if (has_initial_alignment_) {
                Sophus::SE3f T;
                {
                    std::lock_guard<std::mutex> lock(mutex_alignment_);
                    T = T_orbw2gzbw;
                }
                pos_world = T * pos_map;
            }

            memcpy(&cloud_msg.data[idx * 12 + 0], &pos_world(0), sizeof(float));
            memcpy(&cloud_msg.data[idx * 12 + 4], &pos_world(1), sizeof(float));
            memcpy(&cloud_msg.data[idx * 12 + 8], &pos_world(2), sizeof(float));
            idx++;
        }
    }
    
    cloud_msg.width = idx;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step);

    pointcloudPub_->publish(cloud_msg);
}

void StereoMode::PublishTrackingImage(const cv::Mat& image, 
                                                const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
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

void StereoMode::OnOrbMapTransformed(const Sophus::SE3f& T, float s)
{
    {
        Eigen::Vector3f t = T.translation();
        Eigen::Matrix3f R = T.rotationMatrix();
        Eigen::Quaternionf q(R);

        RCLCPP_INFO(this->get_logger(), "OnOrbMapTransformed called. scale s = %.6f", (double)s);
        RCLCPP_INFO(this->get_logger(), "T.translation = [%.6f, %.6f, %.6f]",
                    (double)t.x(), (double)t.y(), (double)t.z());
        RCLCPP_INFO(this->get_logger(), "T.quaternion = [w: %.6f, x: %.6f, y: %.6f, z: %.6f]",
                    (double)q.w(), (double)q.x(), (double)q.y(), (double)q.z());
        RCLCPP_INFO(this->get_logger(), "T.rotation matrix row0: [%.6f, %.6f, %.6f]",
                    (double)R(0,0), (double)R(0,1), (double)R(0,2));
        RCLCPP_INFO(this->get_logger(), "T.rotation matrix row1: [%.6f, %.6f, %.6f]",
                    (double)R(1,0), (double)R(1,1), (double)R(1,2));
        RCLCPP_INFO(this->get_logger(), "T.rotation matrix row2: [%.6f, %.6f, %.6f]",
                    (double)R(2,0), (double)R(2,1), (double)R(2,2));
    }

    Sophus::SE3f T_scaled = T;
    T_scaled.translation() *= s;

    std::lock_guard<std::mutex> lock(mutex_alignment_);

    T_orbw2gzbw = T_orbw2gzbw * T_scaled.inverse();

    RCLCPP_INFO(this->get_logger(),
        "ORB map transformed. Updated world→map alignment.");
}

void StereoMode::PublishWorldToOrbMapTF(const rclcpp::Time &stamp)
{
    std::lock_guard<std::mutex> lock(mutex_alignment_);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = worldGazeboFrameId_;
    tf.child_frame_id = worldFrameId_;

    Eigen::Matrix3f R = T_orbw2gzbw.rotationMatrix();
    Eigen::Vector3f t = T_orbw2gzbw.translation();

    Eigen::Quaternionf q(R);

    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tfBroadcaster_->sendTransform(tf);
}

void StereoMode::PublishOrbMapToOrbCamTF(const Sophus::SE3f& T_orbw2orbcam, const rclcpp::Time &stamp)
{
    Eigen::Matrix3f R = T_orbw2orbcam.rotationMatrix();
    Eigen::Vector3f t = T_orbw2orbcam.translation();
    Eigen::Quaternionf q(R);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = worldFrameId_;
    tf.child_frame_id = cameraFrameOrbId_;

    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tfBroadcaster_->sendTransform(tf);
}
