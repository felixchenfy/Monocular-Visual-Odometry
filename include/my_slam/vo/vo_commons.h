
#ifndef COMMONS_H
#define COMMONS_H

#include "my_slam/vo/common_include.h"
#include "my_slam/vo/frame.h"
namespace my_slam
{
namespace vo
{
using namespace std; 
using namespace cv;

cv::Mat getMotionFromFrame1to2(const Frame::Ptr f1, const Frame::Ptr f2);
void getMotionFromFrame1to2(const Frame::Ptr f1, const Frame::Ptr f2, cv::Mat &R, cv::Mat &t);

}
}
#endif