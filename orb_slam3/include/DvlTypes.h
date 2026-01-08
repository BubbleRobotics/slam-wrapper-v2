#ifndef DVLTYPESH
#define DVLTYESH

#include <Eigen/Dense>

namespace ORB_SLAM3
{
namespace DVL
{

class Point{

    // A class to hold a DVL measurement with its timestamp

    public:
        Point(const float &vel_x, const float &vel_y, const float &vel_z,
              const double &timestamp)
              : v(vel_x, vel_y, vel_z), t(timestamp)
              {};

    public:
       Eigen::Vector3f v;
       double t; 
};

}

}

#endif