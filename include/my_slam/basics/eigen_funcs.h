
// Eigen geometrical transformations and datatype conversions to/from OpenCV

#ifndef MY_SLAM_EIGEN_FUNCS_H
#define MY_SLAM_EIGEN_FUNCS_H

#include "my_slam/common_include.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>

#include <opencv2/core/eigen.hpp>

namespace my_slam
{
namespace basics
{

// --------------  Eigen --------------

// Get Affine3d using pos (x,y,z) and axis-rotation (rot_axis_x, rot_axis_y, rot_axis_z) with maginitude
Eigen::Affine3d getAffine3d(double x, double y, double z, double rot_axis_x, double rot_axis_y, double rot_axis_z);

// -------------- CV <--> Eigen --------------

// Convert cv::Mat R and t --> Eigen Affine3d
Eigen::Affine3d transT_CVRt_to_EigenAffine3d(const cv::Mat &R, const cv::Mat &t);

// -------------- CV <--> Sophus --------------
Sophus::SE3 transT_cv2sophus(const cv::Mat &T_cv);
cv::Mat transT_sophus2cv(const Sophus::SE3 &T_sophus);

} // namespace basics
} // namespace my_slam

#endif