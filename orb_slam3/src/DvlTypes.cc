#include "DvlTypes.h"

namespace ORB_SLAM3{

namespace DVL {

// Standard deviation of isotropic Gaussian DVL measurement noise
float Calib::mSigma = 0.5 * 0.0101; // 0.5m/s times the data-sheet error of 1.01%

// Covariance matrix for DVL measurement noise
Eigen::DiagonalMatrix<float, 3> Calib::mCov(
    Calib::mSigma * Calib::mSigma, 
    Calib::mSigma * Calib::mSigma, 
    Calib::mSigma * Calib::mSigma 
);

// Helper: convert a 4×4 float matrix into SE3f
Sophus::SE3f toSE3(const Eigen::Matrix4f& M) {
    Eigen::Matrix3f R = M.block<3,3>(0,0);
    Eigen::Vector3f t = M.block<3,1>(0,3);
    return Sophus::SE3f(Sophus::SO3f::fitToSO3(R), t);
}

// ----------- IMU -> Camera -----------
Sophus::SE3f Calib::Tic = toSE3(
    (Eigen::Matrix4f() <<
        0.0832135f,   -0.995332f,  -0.0488759f,  0.124137f,
        -0.0141982f,  0.0478569f,  -0.998753f,   -0.235279f,
        0.996431f,    0.0838037f,  -0.0101496f,  -0.0783153f,
        0.f,         0.f,        0.f,         1.f 
    ).finished()
);

// ----------- DVL-> Camera -----------
Sophus::SE3f Calib::Tdc = toSE3(
    (Eigen::Matrix4f() <<
        -0.0526195f,  -0.0236526f,  0.998334f,   0.157385f,
        0.998369f,    0.0209388f,   0.0531174f,  0.015834f,
        -0.0221602f,  0.999501f,    0.0225122f,  -0.192631f,
        0.f,           0.f,           0.f,          1.f 
    ).finished()
);

Sophus::SE3f Calib::Tid = Calib::Tic * Calib::Tdc.inverse();

const Sophus::SO3f Calib::R_ID(){
    return Tid.so3();
}

}

}