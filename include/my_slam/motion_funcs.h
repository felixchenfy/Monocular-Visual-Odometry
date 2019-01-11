
#ifndef MOTION_FUNCS_H
#define MOTION_FUNCS_H

#include "my_slam/common_include.h"
#include "my_slam/frame.h"

namespace my_slam
{

Mat computeT_FromFrame1to2(const Frame::Ptr f1, const Frame::Ptr f2);


}

#endif