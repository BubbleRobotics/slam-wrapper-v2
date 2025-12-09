/* *************************************************************************** */
/*                                                    ########  ########       */
/*   mono.hpp                                         ##     ## ##     ##      */
/*                                                    ##     ## ##     ##      */
/*   By: Paul Joseph <paul@bubble-robotics.com>       ########  ########       */
/*                                                    ##     ## ##   ##        */
/*   Created: 2025/11/13 15:48:25 by Paul Joseph      ##     ## ##    ##       */
/*   Updated: 2025/11/13 15:48:25 by Paul Joseph      ########  ##     ##      */
/*                                                                             */
/* *************************************************************************** */

// Include file 
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

// C++ includes
#include <iostream> // The iostream library is an object-oriented library that provides input and output functionality using streams
#include <algorithm> // The header <algorithm> defines a collection of functions especially designed to be used on ranges of elements.
#include <fstream> // Input/output stream class to operate on files.
#include <chrono> // c++ timekeeper library
#include <vector> // vectors are sequence containers representing arrays that can change in size.
#include <queue>
#include <thread> // class to represent individual threads of execution.
#include <cstdlib> // to find home directory

#include <cstring>
#include <sstream> // String stream processing functionalities

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "rclcpp/rclcpp.hpp"

// #include "your_custom_msg_interface/msg/custom_msg_field.hpp" // Example of adding in a custom message
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using std::placeholders::_1; //* TODO why this is suggested in official tutorial

// Include Eigen
// Quick reference: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense> // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header file

// Include cv-bridge
// #include <cv_bridge/cv_bridge.h> // For humble version only 
#include <cv_bridge/cv_bridge.hpp>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // Image processing tools
#include <opencv2/highgui/highgui.hpp> // GUI tools
#include <opencv2/core/eigen.hpp>
// #include <image_transport/image_transport.h> // For humble version only
#include <image_transport/image_transport.hpp>

//* ORB SLAM 3 includes
#include "System.h" //* Also imports the ORB_SLAM3 namespace

//* Gobal defs
#define pass (void)0 // Python's equivalent of "pass" i.e. no operation


//* Node specific definitions
class MonoMode : public rclcpp::Node
{   
    public:
        //* Class constructor
        MonoMode(); // Constructor 
        ~MonoMode(); // Destructor
        
    private:
 
        //   __     __         _       _     _           
        //   \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___ 
        //    \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
        //     \ V / (_| | |  | | (_| | |_) | |  __/\__ \
        //      \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/

        // Class internal variables
        std::string OPENCV_WINDOW = ""; // Set during initialization
        std::string nodeName = ""; // Name of this node
        std::string vocFilePath = ""; // Path to ORB vocabulary provided by DBoW2 package
        std::string settingsFilePath = ""; // Path to settings file provided by ORB_SLAM3 package
        
        std::string imgTopic = ""; // Topic to subscribe to receive RGB images from a python node
        std::string imuTopic = ""; // Topic to subscribe to receive IMU data from a python node

        //* Definitions of publisher and subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub_; // Subscriber to receive image messages
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_; // Subscriber to receive IMU messages

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr trackingImagePub_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;


        //* ORB_SLAM3 related variables
        ORB_SLAM3::System* pAgent; // pointer to a ORB SLAM3 object
        ORB_SLAM3::System::eSensor sensorType;

        bool enableDebugWindow = false; // Enable debug window showing SLAM in pangolin/opencv
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false; // Shows OpenCV window output

        // switch for inertial and non-inertial mode
        bool isInertial = true;
        
        // IMU buffer 
        nav_msgs::msg::Path path_;
    
        std::string worldFrameId_ = "mapOrb";
        std::string cameraFrameOrbId = "cameraOrb";
        std::string cameraFrameId_ = "";
        std::string imuFrameId_ = "";
        bool publishTf_ = true;
        bool publishPointcloud_ = true;

        // frame transform vars
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        geometry_msgs::msg::TransformStamped transformImuCam;

        //    _____                 _   _                  
        //   |  ___|   _ _ __   ___| |_(_) ___  _ __  ___  
        //   | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __| 
        //   |  _|| |_| | | | | (__| |_| | (_) | | | \__ \ 
        //   |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ 
        //* ROS callbacks
        void ImgCallback(const sensor_msgs::msg::Image::SharedPtr img_msg); // Callback to process RGB image and semantic matrix sent by Python node
        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg); // Callback to process IMU data sent by Python node

        //* Helper functions
        void InitializeVSLAM(); //* Method to bind an initialized VSLAM framework to this node
        bool InitImuCamTransform(); //* Method to initialize the transform between IMU and camera frames
        bool CheckSuccessfulTracking(Sophus::SE3f Tcw); //* Method to check if tracking was successful and publish pose 

        // Publishers for Orb Slam Output
        void PublishOrbSlamOutput(const Sophus::SE3f& Twc, 
                                    const sensor_msgs::msg::Image::SharedPtr img_msg, 
                                    const cv_bridge::CvImageConstPtr& cv_ptr);
        void PublishPose(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header);
        void PublishOdometry(const Sophus::SE3f& Twc, const sensor_msgs::msg::Image::SharedPtr img_msg);
        void PublishPath(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header);
        void PublishTF(const Sophus::SE3f& Twc, const sensor_msgs::msg::Image::SharedPtr img_msg);
        void PublishMapPoints(const std_msgs::msg::Header& header);
        void PublishTrackingImage(const cv::Mat& image, const sensor_msgs::msg::Image::SharedPtr img_msg);

};

#endif