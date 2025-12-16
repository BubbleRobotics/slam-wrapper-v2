// Classes and methods used to treat DVL data.

// ----------- INCLUDES ----------- //

#ifndef DVL_TYPES_H
#define DVL_TYPES_H

#include <opencv2/core/core.hpp>

#endif

// ----------- DECLARATIONS ----------- //

namespace ORB_SLAM3 {

namespace DVL {

// Class representing a DVL measurement
// Only contains a cv::Point of 3 coordinates (the velocity measurement)
// and the the timestamp of the measurement
class Point{

    public:
        // Construct from separate floats
        Point(const float &v_x, 
              const float &v_y, 
              const float &v_z, 
              const double &timestamp);

    public:
        double t;
        Eigen::Vector3f v;
        // Ensures that instances of Point are allocated at a memory 
        // address suitable for Eigen’s SIMD operations
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

}
