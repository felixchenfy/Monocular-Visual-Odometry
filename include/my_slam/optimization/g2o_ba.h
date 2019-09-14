
/* This script is mainly copied and then modified from Chapter 7 of Dr. Xiang Gao's book. Link is here:
https://github.com/gaoxiang12/slambook/blob/master/ch7/pose_estimation_3d2d.cpp
*/

#ifndef G2O_BA_H
#define G2O_BA_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <opencv2/core/core.hpp>

namespace my_slam
{
namespace optimization
{
using namespace std;
using namespace cv;

void optimizeSingleFrame(
    const vector<cv::Point2f *> &points_2d,
    const cv::Mat &K,
    vector<cv::Point3f *> &points_3d,
    cv::Mat &cam_pose_in_world,
    bool fix_map_pts, bool update_map_pts);

void bundleAdjustment(
    const vector<vector<cv::Point2f *>> &v_pts_2d,
    const vector<vector<int>> &v_pts_2d_to_3d_idx,
    const cv::Mat &K,
    unordered_map<int, cv::Point3f *> &pts_3d,
    vector<cv::Mat *> &v_camera_g2o_poses,
    const cv::Mat &information_matrix,
    bool fix_map_pts = false, bool update_map_pts = true);

} // namespace optimization
} // namespace my_slam
#endif