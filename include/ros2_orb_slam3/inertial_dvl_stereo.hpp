
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

// ---- STANDARD ---- //

// C++ includes
#include <iostream> // The iostream library is an object-oriented library that provides input and output functionality using streams
#include <algorithm> // The header <algorithm> defines a collection of functions especially designed to be used on ranges of elements.
#include <fstream> // Input/output stream class to operate on files.
#include <chrono> // c++ timekeeper library
#include <vector> // vectors are sequence containers representing arrays that can change in size.
#include <queue>
#include <thread> // class to represent individual threads of execution.
#include <mutex> // A mutex is a lockable object that is designed to signal when critical sections of code need exclusive access, preventing other threads with the same protection from executing concurrently and access the same memory locations.
#include <cstdlib> // to find home directory
#include <filesystem> // to detect if paths are existent or not
#include <stdexcept> // to throw exceptions during building

// Manipulating strings
#include <cstring>
#include <sstream> // String stream processing functionalities

// ---- ROS ---- //

// Basic ros functionality
#include "rclcpp/rclcpp.hpp"

// Message types
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <dvl_msgs/msg/dvl.hpp>

// Synchronised subscribers
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// ---- EXTERNAL DEP ---- //

#include <cv_bridge/cv_bridge.hpp>
#include <Eigen/Dense>

// ---- SLAM ---- //

#include "System.h" // Also imports the ORB_SLAM3 namespace

class InertialDvlStereoMode : public rclcpp::Node{
    
    // Class constructor and descructor
    public:
        InertialDvlStereoMode();  // Constructor
        ~InertialDvlStereoMode();  // Destructor
    
    private:

        /*   __     __         _       _     _           
             \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___ 
              \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
               \ V / (_| | |  | | (_| | |_) | |  __/\__ \
                \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
        */
       
        // ---- ROS ---- //

        // Node paramters: strings
        std::string settingsFilePath;
        std::string vocFilePath;
        std::string img0Topic;
        std::string img1Topic;
        std::string imuTopic;
        std::string dvlTopic;

        // Node parameters: bools
        bool enableDebugWindow = false;
        bool publishTf_ = true;
        bool manualTimeSync = false;
        bool imuFromYaml = false;

        // Subscribers
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img0Sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img1Sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ImgSyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<ImgSyncPolicy>> sync_;

        rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvlSub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr trackingImagePub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
        

        // ---- PIPELINE ---- //

        // InitializeSLAM
        ORB_SLAM3::System::eSensor sensorType;
        ORB_SLAM3::System* pAgent;  // Pointer to ORB SLAM3 object

        // Frame Definitions
        std::string worldFrameOrbId_ = "mapOrb";
        std::string cameraFrameOrbId_ = "cameraOrb";
        std::string cameraFrameId_ = "";
        std::string imuFrameId_ = "";

        // Path: keeps track of poses
        nav_msgs::msg::Path path_;

        // frame transform vars
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        geometry_msgs::msg::TransformStamped transformImuCam;

        /*    _____                 _   _                  
             |  ___|   _ _ __   ___| |_(_) ___  _ __  ___  
             | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __| 
             |  _|| |_| | | | | (__| |_| | (_) | | | \__ \ 
             |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ 
        */
        void InitializeSLAM();
        bool InitImuCamTransform();

        void StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img,
                            const sensor_msgs::msg::Image::ConstSharedPtr &right_img);
        void DvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg);
        void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg); 


        // ---- ROS ---- //
        
        void PublishOrbSlamOutput(const Sophus::SE3f& Twc, 
                                  const sensor_msgs::msg::Image::ConstSharedPtr img_msg, 
                                  const cv_bridge::CvImageConstPtr& cv_ptr);

        void PublishPose(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header);

        void PublishOdometry(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header);

        void PublishPath(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header);
        
        void PublishTF(const Sophus::SE3f& Twc, const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

        void PublishTrackingImage(const cv::Mat& image, const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
};

#endif