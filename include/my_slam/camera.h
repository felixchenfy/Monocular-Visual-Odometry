
#ifndef CAMERA_H
#define CAMERA_H

#include "my_slam/common_include.h"

namespace my_slam
{

// Pinhole RGBD camera model
class Camera
{
public:
  typedef std::shared_ptr<Camera> Ptr;
  double fx_, fy_, cx_, cy_;
  double depth_scale_; // This is for rgb-d camera. Not used here.
  // double k1_, k2_, p1_, p2_, k3_; // Suppose image has been undistorted

public:
  Camera(
      double fx, double fy, double cx, double cy,
      //double k1=0,double k2=0,double p1=0,double p2=0,double k3=0,
      double depth_scale = 0) : fx_(fx), fy_(fy), cx_(cx), cy_(cy),
                                //k1_(k1),k2_(k2),p1_(p1),p2_(p2),k3_(k3),
                                depth_scale_(depth_scale)
  {
  }

  // coordinate transform: world, camera, pixel
  // p_w -> p_c
  Vector3d world2camera(const Vector3d &p_w, const SE3 &T_c_w);

  // p_c -> p_w
  Vector3d camera2world(const Vector3d &p_c, const SE3 &T_c_w);

  // p_c -> (u, v)
  Vector2d camera2pixel(const Vector3d &p_c);

  // (u, v) -> p_c
  Vector3d pixel2camera(const Vector2d &p_p, double depth);

  // p_w -> (u, v)
  Vector2d world2pixel(const Vector3d &p_w, const SE3 &T_c_w);

  // (u, v) -> p_w
  Vector3d pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth);
};

} // namespace my_slam
#endif
