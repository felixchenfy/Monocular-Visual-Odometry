
// Eigen geometrical transformations and datatype conversion with OpenCV

#ifndef EIGEN_TRANS_H
#define EIGEN_TRANS_H

// This is for testing basic functions/datatypes in Eigen.
// 1. Datatype conversions between:
//      OpenCV and Matrix3d/Vector3d/Isometry3d/Affine3d.

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for cv::Rodrigues

using namespace std;
using namespace cv;
using namespace Eigen;

namespace my{

// Combine cv::Mat R and t --> Affine3d SE3
Eigen::Affine3d transCVMatRt2Affine3d(const Mat &R0, const Mat &t);

}

#endif