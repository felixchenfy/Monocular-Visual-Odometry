
// Eigen geometrical transformations and datatype conversions to/from OpenCV

#ifndef EIGEN_TRANS_H
#define EIGEN_TRANS_H
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for cv::Rodrigues

namespace my_basics
{

// convert cv::Mat R and t --> Affine3d SE3
Eigen::Affine3d transCVMatRt2Affine3d(const cv::Mat &R0, const cv::Mat &t);

// get Affine3d using pos (x,y,z) and euler angles (ea_x, ea_y, ea_z)
Eigen::Affine3d getAffine3d(double x, double y, double z, double ea_x, double ea_y, double ea_z); 

}

#endif