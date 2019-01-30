
/* This script is mainly copied and then modified from Chapter 7 of Dr. Xiang Gao's book. Link is here:
https://github.com/gaoxiang12/slambook/blob/master/ch7/pose_estimation_3d2d.cpp
*/

#ifndef G2O_BA_H
#define G2O_BA_H


#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>


namespace my_optimization
{
using namespace std;
using namespace cv;

void bundleAdjustment(
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat &K,
    Mat &T_world_to_cam_cv);
} // namespace my_optimization
#endif