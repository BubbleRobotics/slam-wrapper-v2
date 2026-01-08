#ifndef DVLTYPESH
#define DVLTYPESH

#include <Eigen/Dense>

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
    // * R_ID
    // * Sigma_D -- the DVL measurement noise

    private:

        // For now: hard-code DVL parameters here
        // Average speed of BlueRov 1m/s, long term accuracy 1.01%
        float mSigma = 1.0 * 0.0101;

    public:

        // Rotation matrix transforming vectors in the DVL frame to vectors
        // in the IMU body frame. Taken from the Tank dataset parameter file 
        // for HalfTank_Easy
        const Sophus::SO3<float> mR_ID(-0.0030,    0.0292,    0.9996,
                                        0.9995,    0.0328,    0.0021,
                                        -0.0327,    0.9990,   -0.0293);
        // Diagonal sensor noise covarinace meatrix: assume noise of the 
        // measured velocity vector around the physical velocity is isotropic.
        const Eigen::DiagonalMatrix<float, 3> mCov(mSigma*mSigma, mSigma*mSigma, mSigma*mSigma);

}

}

}

#endif