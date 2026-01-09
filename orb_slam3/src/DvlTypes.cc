#include "DvlTypes.h"

namespace ORB_SLAM3{

namespace DVL {

// Standard deviation of isotropic Gaussian DVL measurement noise
float Calib::mSigma = 1.0 * 0.0101;

// Covariance matrix for DVL measurement noise
Eigen::DiagonalMatrix<float, 3> Calib::mCov(
    Calib::mSigma * Calib::mSigma, 
    Calib::mSigma * Calib::mSigma, 
    Calib::mSigma * Calib::mSigma 
);

// Matrix R_ID
// Taken from the Tank dataset parameter file for HalfTank_Easy
// Determinant 1.000036 but it needs the explicit fitToSO3 to work
Sophus::SO3f Calib::mRid = Sophus::SO3f::fitToSO3(
    Eigen::Matrix3f{
        {-0.0030f, 0.0292f, 0.9996f},
        { 0.9995f, 0.0328f, 0.0021f},
        {-0.0327f, 0.9990f,-0.0293f}
    }
);

// void Calib::usless_method(){
//     float x = 3.2;
//     float y;
//     y = 2 * x;
// };

}

}