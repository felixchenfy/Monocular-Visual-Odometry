
#ifndef CAMERA_H
#define CAMERA_H

#include "my_geometry/common_include.h"

namespace my_geometry
{
// ---------------- transformation ----------------
Point2f pixel2camNormPlane(const Point2f &p, const Mat &K);
Point3f pixel2cam(const Point2f &p, const Mat &K, double depth = 1);
Point2f cam2pixel(const Point3f &p, const Mat &K);
Point2f cam2pixel(const Mat &p, const Mat &K);

// ---------------- Class ----------------
class Camera
{
public:
  typedef std::shared_ptr<Camera> Ptr;
  double fx_, fy_, cx_, cy_;

public:
  Camera(double fx, double fy, double cx, double cy) : fx_(fx), fy_(fy), cx_(cx), cy_(cy)
  {
    Mat K = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  }
  Camera(Mat K)
  {
    fx_ = K.at<double>(0, 0);
    fy_ = K.at<double>(1, 1);
    cx_ = K.at<double>(0, 2);
    cy_ = K.at<double>(1, 2);
  }

  //   // coordinate transform: world, camera, pixel
  //   // p_w -> p_c
  //   Vector3d world2camera(const Vector3d &p_w, const SE3 &T_c_w);

  //   // p_c -> p_w
  //   Vector3d camera2world(const Vector3d &p_c, const SE3 &T_c_w);

  //   // p_c -> (u, v)
  //   Vector2d camera2pixel(const Vector3d &p_c);

  //   // (u, v) -> p_c
  //   Vector3d pixel2camera(const Vector2d &p_p, double depth);

  //   // p_w -> (u, v)
  //   Vector2d world2pixel(const Vector3d &p_w, const SE3 &T_c_w);

  //   // (u, v) -> p_w
  //   Vector3d pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth);

};
} // namespace my_slam
#endif
