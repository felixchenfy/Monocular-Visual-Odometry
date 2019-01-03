

#ifndef FRAME_H
#define FRAME_H

#include "my_slam/common_include.h"
#include "my_geometry/camera.h"

using namespace std;
using namespace cv;

namespace my_slam
{

class Frame
{
private:
public:
  typedef std::shared_ptr<Frame> Ptr;
  static unsigned long factory_id_;

public:
  unsigned long id_;  // id of this frame
  double time_stamp_; // when it is recorded

  // image features
  Mat rgb_img_;
  vector<KeyPoint> keypoints;
  vector<DMatch> matches;
  Mat descriptors;

  // pose
  my_geometry::Camera::Ptr camera_;
  Mat T_c_w_; // transform from camera to world

public:
  Frame() {}
  ~Frame() {}
  static Frame::Ptr createFrame(Mat rgb_img, my_geometry::Camera::Ptr camera, double time_stamp = -1);

public: // Member functions

};

} // namespace my_slam

#endif // FRAME_H
