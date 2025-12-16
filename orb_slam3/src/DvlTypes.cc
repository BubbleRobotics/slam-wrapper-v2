#include "DvlTypes.h"

namespace ORB_SLAM3 {

namespace DVL {

Point::Point(const float &v_x, 
              const float &v_y, 
              const float &v_z, 
              const double &timestamp) 
              : v(v_x, v_y, v_z), t(timestamp)
              {};

}

}