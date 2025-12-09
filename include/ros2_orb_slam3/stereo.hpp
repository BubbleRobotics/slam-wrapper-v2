/* *************************************************************************** */
/*                                                    ########  ########       */
/*   stereo.hpp                                       ##     ## ##     ##      */
/*                                                    ##     ## ##     ##      */
/*   By: Paul Joseph <paul@bubble-robotics.com>       ########  ########       */
/*                                                    ##     ## ##   ##        */
/*   Created: 2025/11/13 15:48:18 by Paul Joseph      ##     ## ##    ##       */
/*   Updated: 2025/11/13 15:48:18 by Paul Joseph      ########  ##     ##      */
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
#include <mutex> // A mutex is a lockable object that is designed to signal when critical sections of code need exclusive access, preventing other threads with the same protection from executing concurrently and access the same memory locations.
#include <cstdlib> // to find home directory
#include <filesystem> // to detect if paths are existent or not
#include <stdexcept> // to throw exceptions during building

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
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
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
using std::placeholders::_1;
using std::placeholders::_2;

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
class StereoMode : public rclcpp::Node
{   

    //* Class constructor
    public:
        StereoMode(); // Constructor 
        ~StereoMode(); // Destructor
        
    private:
        
        // Class internal variables
        std::string OPENCV_WINDOW = ""; // Set during initialization
        std::string nodeName = ""; // Name of this node
        std::string vocFilePath = ""; // Path to ORB vocabulary provided by DBoW2 package
        std::string settingsFilePath = ""; // Path to settings file provided by ORB_SLAM3 package

        std::string img0Topic = ""; // Topic to subscribe to receive infra1 rec images
        std::string img1Topic = ""; // Topic to subscribe to receive infra2 rec images
        std::string imuTopic = "";

        //* Definitions of publisher and subscribers
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img0Sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img1Sub_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ImgSyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<ImgSyncPolicy>> sync_;

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

        bool enableDebugWindow = false;
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false; // Shows OpenCV window output

        bool isInertial = true;
        bool manualTimeSync = false;
        bool imu_from_yaml = false;

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
        //* Helper functions
        // ORB_SLAM3::eigenMatXf convertToEigenMat(const std_msgs::msg::Float32MultiArray& msg); // Helper method, converts semantic matrix eigenMatXf, a Eigen 4x4 float matrix


        //    _____                 _   _                  
        //   |  ___|   _ _ __   ___| |_(_) ___  _ __  ___  
        //   | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __| 
        //   |  _|| |_| | | | | (__| |_| | (_) | | | \__ \ 
        //   |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ 

        void InitializeVSLAM(); //* Method to bind an initialized VSLAM framework to this node

        void StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_img,
                             const sensor_msgs::msg::Image::ConstSharedPtr &right_img);
        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg); // Callback to process IMU data sent by Python node

        bool InitImuCamTransform(); //* Method to initialize the transform between IMU and camera frames
        bool CheckSuccessfulTracking(Sophus::SE3f Tcw); //* Method to check if tracking was successful and publish pose 

        // Publishers for Orb Slam Output
        void PublishOrbSlamOutput(const Sophus::SE3f& Twc, 
                                    const sensor_msgs::msg::Image::ConstSharedPtr img_msg, 
                                    const cv_bridge::CvImageConstPtr& cv_ptr);
        void PublishPose(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header);
        void PublishOdometry(const Sophus::SE3f& Twc, const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
        void PublishPath(const Sophus::SE3f& Twc, const std_msgs::msg::Header& header);
        void PublishTF(const Sophus::SE3f& Twc, const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
        void PublishMapPoints(const std_msgs::msg::Header& header);
        void PublishTrackingImage(const cv::Mat& image, const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
};

#endif