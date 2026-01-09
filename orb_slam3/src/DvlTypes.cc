#include "DvlTypes.h"

namespace ORB_SLAM3{

namespace DVL {

Eigen::DiagonalMatrix<float, 3> Calib::mCov(1.0f, 1.0f, 1.0f);
float Calib::mSigma = 1.0 * 0.0101;
Sophus::SO3f Calib::mRid(
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