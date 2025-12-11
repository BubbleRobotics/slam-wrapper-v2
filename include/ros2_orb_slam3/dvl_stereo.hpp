
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

#include <cstring>

#include "rclcpp/rclcpp.hpp"

class DvlStereoMode : public rclcpp::Node{
    
    // Class constructor and descructor
    public:
        DvlStereoMode();  // Constructor
        ~DvlStereoMode();  // Destructor
    
    private:

    // Member variables: ros parameters
    std::string vocFilePath;
    std::string settingsFilePath;
    std::string img0Topic;
    std::string img1Topic;
    std::string imuTopic;
    std::string dvlTopic;

    bool enableDebugWindow;
    bool publishTf_;

};

#endif