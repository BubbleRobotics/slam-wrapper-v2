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
class MonocularInertialMode : public rclcpp::Node
{   
    //* This slam node inherits from both rclcpp and ORB_SLAM3::System classes
    //* public keyword needs to come before the class constructor and anything else
    public:
    std::string experimentConfig = ""; // String to receive settings sent by the python driver
    double timeStep; // Timestep data received from the python node
    std::string receivedConfig = "";

    //* Class constructor
    MonocularInertialMode(); // Constructor 

    ~MonocularInertialMode(); // Destructor
        
    private:
        
        // Class internal variables
        std::string OPENCV_WINDOW = ""; // Set during initialization
        std::string nodeName = ""; // Name of this node
        std::string vocFilePath = ""; // Path to ORB vocabulary provided by DBoW2 package
        std::string settingsFilePath = ""; // Path to settings file provided by ORB_SLAM3 package
        
        std::string pubconfigackName = ""; // Publisher topic name
        std::string imgTopic = ""; // Topic to subscribe to receive RGB images from a python node
        std::string imuTopic = ""; // Topic to subscribe to receive IMU data from a python node

        //* Definitions of publisher and subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgMsgSub_; // Subscriber to receive image messages
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuMsgSub_; // Subscriber to receive IMU messages

        //* ORB_SLAM3 related variables
        ORB_SLAM3::System* pAgent; // pointer to a ORB SLAM3 object
        ORB_SLAM3::System::eSensor sensorType;
        bool enableDebugWindow = false; // Enable debug window showing SLAM in pangolin/opencv
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false; // Shows OpenCV window output

        //* ROS callbacks
        void Img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg); // Callback to process RGB image and semantic matrix sent by Python node
        void Imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg); // Callback to process IMU data sent by Python node
        
        // IMU buffer 
        std::vector<ORB_SLAM3::IMU::Point> imu_buffer_;
        std::mutex imu_mutex_;

        //* Helper functions
        void initializeVSLAM(); //* Method to bind an initialized VSLAM framework to this node

};

#endif