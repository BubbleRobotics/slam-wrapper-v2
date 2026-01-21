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
        -0.0165f,  -0.0175f,  -0.9997f,  -0.211687435f,
        -0.0106f,   0.9998f,  -0.0173f,  -0.097259169f,
         0.9998f,   0.0103f,  -0.0167f,  -0.056238089f,
         0.f,       0.f,       0.f,       1.f
    ).finished()
);

// ----------- DVL-> Camera -----------
Sophus::SE3f Calib::Tdc = toSE3(
    (Eigen::Matrix4f() <<
        -0.0030f,   0.0292f,   0.9996f,   0.132344740f,
         0.9995f,   0.0328f,   0.0021f,   0.032085643f,
        -0.0327f,   0.9990f,  -0.0293f,  -0.240100175f,
         0.f,       0.f,       0.f,       1.f
    ).finished()
);

Sophus::SE3f Calib::Tid = Calib::Tic * Calib::Tdc.inverse();

const Sophus::SO3f Calib::R_ID(){
    return Tid.so3();
}

}

}