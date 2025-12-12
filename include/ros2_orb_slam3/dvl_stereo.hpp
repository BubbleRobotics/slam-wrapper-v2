
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

// Manipulating strings
#include <cstring>

// ---- ROS ---- //

// Basic ros functionality
#include "rclcpp/rclcpp.hpp"
// Message types
#include "sensor_msgs/msg/image.hpp"
// Synchronised subscribers
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

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
        bool enableDebugWindow;
        bool publishTf_;

        // Subscribers
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img0Sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img1Sub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ImgSyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<ImgSyncPolicy>> sync_;

        /*    _____                 _   _                  
             |  ___|   _ _ __   ___| |_(_) ___  _ __  ___  
             | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __| 
             |  _|| |_| | | | | (__| |_| | (_) | | | \__ \ 
             |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/ 
        */

        void StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img0,
                            const sensor_msgs::msg::Image::ConstSharedPtr& img1);
};

#endif