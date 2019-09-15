
#include "my_slam/vo/vo_commons.h"

namespace my_slam
{
namespace vo
{

cv::Mat getMotionFromFrame1to2(const Frame::Ptr f1, const Frame::Ptr f2)
{
    const cv::Mat &T_w_to_f1 = f1->T_w_c_;
    const cv::Mat &T_w_to_f2 = f2->T_w_c_;
    cv::Mat T_f1_to_f2 = T_w_to_f1.inv() * T_w_to_f2;
    return T_f1_to_f2;
}
void getMotionFromFrame1to2(const Frame::Ptr f1, const Frame::Ptr f2, cv::Mat &R, cv::Mat &t)
{
    cv::Mat T = getMotionFromFrame1to2(f1, f2);
    basics::getRtFromT(T, R, t);
}

} // namespace vo
} // namespace my_slam