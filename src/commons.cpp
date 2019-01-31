
#include "my_slam/commons.h"

namespace my_slam
{

Mat calcMotionFromFrame1to2(const Frame::Ptr f1, const Frame::Ptr f2){
    const Mat &T_w_to_f1 = f1->T_w_c_;
    const Mat &T_w_to_f2 = f2->T_w_c_;
    Mat T_f1_to_f2 = T_w_to_f1.inv() * T_w_to_f2;
    return T_f1_to_f2;
}
void calcMotionFromFrame1to2(const Frame::Ptr f1, const Frame::Ptr f2, Mat &R, Mat &t){
    Mat T = calcMotionFromFrame1to2(f1,f2);
    getRtFromT(T, R, t);
}

} // namespace my_slam