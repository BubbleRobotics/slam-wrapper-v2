/* *************************************************************************** */
/*                                                    ########  ########       */
/*   stereo.cpp                                       ##     ## ##     ##      */
/*                                                    ##     ## ##     ##      */
/*   By: Diego Hernandez <diego@bubble-robotics.com>  ########  ########       */
/*                                                    ##     ## ##   ##        */
/*   Created: 2026/01/05 17:07:03 by Diego Hernandez  ##     ## ##    ##       */
/*   Updated: 2026/01/05 17:07:03 by Diego Hernandez  ########  ##     ##      */
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
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <dvl_msgs/msg/dvl.hpp>
using std::placeholders::_1;
using std::placeholders::_2;

class DVLMode : public rclcpp::Node{
    public:
        DVLMode(); // Constructor 
        ~DVLMode(); // Destructor

    private:
        // Class internal variables
        std::string dvlTopic = ""; // Topic to subscribe to receive DVL data
        std::string odometryEst = ""; // Topic to subscribe to receive Odometry estimation from ORB SLAM
        bool initialized_dvl = false;
        bool verbose = false;

        //* Definitions of publisher and subscribers
        rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvlSub_; // Subscriber to receive DVL messages
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_; // Subscriber to receive Odometry messages

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_; // Publisher for fused odometry

        // Client and timer for initialization service
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr init_client_;
        rclcpp::TimerBase::SharedPtr init_timer_;

        //* Callback functions
        void DvlCallback(const dvl_msgs::msg::DVL::ConstSharedPtr &msg);
        void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);

        void InitializeService();
};

#endif