
#ifndef CAMERA_H
#define CAMERA_H

#include "my_geometry/common_include.h"

namespace my_geometry
{
// ---------------- transformations (by OpenCV) ----------------
Point2f pixel2camNormPlane(const Point2f &p, const Mat &K);
Point3f pixel2cam(const Point2f &p, const Mat &K, double depth = 1);
Point2f cam2pixel(const Point3f &p, const Mat &K);
Point2f cam2pixel(const Mat &p, const Mat &K);
Mat world2camera(const Point3f &p, const Mat &T_world_to_cam);


// ---------------- transformations (by Eigen && Sophus; For g2o optimization) ----------------
// ------------ !!! Copy from Dr. Xiang Gao's book !!! --------------
// TO DO

// ---------------- Class ----------------
class Camera /*This is defined but not acctually used*/
{
public:
  typedef std::shared_ptr<Camera> Ptr;
  double fx_, fy_, cx_, cy_;
  Mat K_;

public:
  Camera(double fx, double fy, double cx, double cy) : fx_(fx), fy_(fy), cx_(cx), cy_(cy)
  {
    K_ = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  }
  Camera(Mat K)
  {
    fx_ = K.at<double>(0, 0);
    fy_ = K.at<double>(1, 1);
    cx_ = K.at<double>(0, 2);
    cy_ = K.at<double>(1, 2);
    K_=K;
  }

};
} // namespace my_slam
#endif
