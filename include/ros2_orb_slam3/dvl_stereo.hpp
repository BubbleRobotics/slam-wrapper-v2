
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

// ---- STANDARD ---- //

// Manipulating strings
#include <cstring>

// ---- ROS ---- //

// Basic ros functionality
#include "rclcpp/rclcpp.hpp"isInertial

// Message types
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
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

class DvlStereoMode : public rclcpp::Node{
    
    // Class constructor and descructor
    public:
        DvlStereoMode();  // Constructor
        ~DvlStereoMode();  // Destructor
    
    private:

        /*   __     __         _       _     _           
             \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___ 
              \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
               \ V / (_| | |  | | (_| | |_) | |  __/\__ \
                \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
        */
       
        // ---- ROS ---- //

        // Node paramters: strings
        std::string vocFilePath;
        std::string settingsFilePath;
        std::string img0Topic;
        std::string img1Topic;
        std::string dvlTopic;

        // Node parameters: bools
        bool isDVLUsed;
        bool enableDebugWindow;
        bool publishTf_;

        // Subscribers
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img0Sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img1Sub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ImgSyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<ImgSyncPolicy>> sync_;

        // Publishers
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr trackingImagePub_;
        
        std::string worldFrameId_ = "mapOrb";
        std::string cameraFrameOrbId = "cameraOrb";

        // ---- PIPELINE ---- //

        // InitializeSLAM
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false; // Shows OpenCV window output
        ORB_SLAM3::System* pAgent;  // Pointer to ORB SLAM3 object

        // Stereo Callback
        std::string cameraFrameId_ = "";

        // Path: keeps track of poses
        nav_msgs::msg::Path path_;

        /*    _____                 _   _                  
             |  ___|   _ _ __   ___| |_(_) ___  _ __  ___  
             | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __| 
             |  _|| |_| | | | | (__| |_| | (_) | | | \__ \ 
             |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ 
        */

        void StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img0,
                            const sensor_msgs::msg::Image::ConstSharedPtr& img1);

        void DvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg);

        void InitializeSLAM();

        // ---- ROS ---- //
        
        void PublishOrbSlamOutput(const Sophus::SE3f& Twc, 
                                  const sensor_msgs::msg::Image::ConstSharedPtr img_msg, 
                                  const cv_bridge::CvImageConstPtr& cv_ptr);

        void PublishPose(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header);

        void PublishOdometry(const Sophus::SE3f& Twc, const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

        void PublishPath(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header);
        
        void PublishTF(const Sophus::SE3f& Twc, const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
};

#endif