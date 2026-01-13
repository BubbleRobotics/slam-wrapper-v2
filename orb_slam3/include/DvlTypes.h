#ifndef DVLTYPESH
#define DVLTYPESH

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace ORB_SLAM3
{
namespace DVL
{

class Point{

    // A class to hold a DVL measurement with its timestamp

    public:
        // Default constructor -- for empty Point objects
        Point() : v(0, 0, 0), t(0) {};
        // Constructor with values
        Point(const float &vel_x, const float &vel_y, const float &vel_z,
              const double &timestamp)
              : v(vel_x, vel_y, vel_z), t(timestamp)
              {};

    public:
       Eigen::Vector3f v;
       double t; 
};

class Calib{

    // A class to hold the calibration parameters relating to the DVL:
    // R_ID
    // Sigma_D -- the DVL measurement noise

    private:

        // Standard deviation for measurement noise matrix
        static float mSigma;

        // Diagonal sensor noise covarinace meatrix: assume noise of the
        // measured velocity vector around the physical velocity is isotropic
        static Eigen::DiagonalMatrix<float, 3> mCov;

        // Rotation matrix transforming vectors in the DVL frame to vectors in the IMU body frame.
        static Sophus::SO3f mRid;

    public:

         static Sophus::SE3f Tic; 
         static Sophus::SE3f Tdc;
         static Sophus::SE3f Tid;
        
    public:

        // Default constructor
        Calib() = default;
        
        // Getters for the constants
        static const Sophus::SO3f& R_ID() {
           return mRid;
        }
        static const Eigen::DiagonalMatrix<float, 3>& Cov() {
           return mCov;
        }

        // void usless_method();

};

}

}

#endif