
/* This script is mainly copied and then modified from Chapter 7 of Dr. Xiang Gao's book. Link is here:
https://github.com/gaoxiang12/slambook/blob/master/ch7/pose_estimation_3d2d.cpp
*/

#ifndef G2O_BA_H
#define G2O_BA_H


#include <iostream>
#include <vector>
#include <unordered_map>
#include <opencv2/core/core.hpp>


namespace my_optimization
{
using namespace std;
using namespace cv;

void optimizeSingleFrame(
    const vector<Point2f> &points_2d,
    const Mat &K,
    vector<Point3f*> &points_3d,
    Mat &cam_pose_in_world,
    bool fix_map_pts, bool update_map_pts);

void bundleAdjustment(
    const vector<vector<Point2f*>> &v_pts_2d,
    const vector<vector<int>> &v_pts_2d_to_3d_idx,
    const Mat& K,
    unordered_map<int, Point3f*> &pts_3d,
    vector<Mat*> &v_camera_g2o_poses,
    bool fix_map_pts, bool update_map_pts);

} // namespace my_optimization
#endif