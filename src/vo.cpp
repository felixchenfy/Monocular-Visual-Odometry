
#include "my_slam/vo.h"

namespace my_slam
{

VisualOdometry::VisualOdometry()
{
    vo_state_ = BLANK;
    frames_.clear();

    // debug
    DEBUG_STOP_PROGRAM_=false;
}



} // namespace my_slam