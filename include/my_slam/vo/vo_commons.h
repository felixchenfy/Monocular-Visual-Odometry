
#ifndef MY_SLAM_COMMONS_H
#define MY_SLAM_COMMONS_H

#include "my_slam/common_include.h"
#include "my_slam/vo/frame.h"
namespace my_slam
{
namespace vo
{

cv::Mat getMotionFromFrame1to2(const Frame::Ptr f1, const Frame::Ptr f2);
void getMotionFromFrame1to2(const Frame::Ptr f1, const Frame::Ptr f2, cv::Mat &R, cv::Mat &t);

}
}
#endif