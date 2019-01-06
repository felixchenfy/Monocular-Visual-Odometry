

#ifndef FRAME_H
#define FRAME_H

#include "my_slam/common_include.h"
#include "my_geometry/camera.h"
#include "my_geometry/feature_match.h"

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
  vector<KeyPoint> keypoints_;
  Mat descriptors_;
  vector<DMatch> matches_; // matches with the previous frame
  // vector<int> inliers_of_matches_; // inliers index with respect to the matches
  vector<int> inliers_of_all_pts_; // inliers index with respect to all the points
  // vector<DMatch> inliers_matches; // not used
  vector<Point3f> inliers_pts3d_; // matches with the previous frame

  // -- Other commonly used points
  // vector<Point2f> pts_matched_;

  // pose
  my_geometry::Camera::Ptr camera_;
  Mat T_w_c_; // transform from camera to world
  Mat R_curr_to_prev_;
  Mat t_curr_to_prev_;

public:
  Frame() {}
  ~Frame() {}
  static Frame::Ptr createFrame(Mat rgb_img, my_geometry::Camera::Ptr camera, double time_stamp = -1);

public: // Member functions
   void extractKeyPoints() { 
      my_geometry::extractKeyPoints(rgb_img_, keypoints_);
    }
    void computeDescriptors(){
      my_geometry::computeDescriptors(rgb_img_, keypoints_, descriptors_);
    };
    void matchFeatures(Frame::Ptr prev_frame){
      my_geometry::matchFeatures(
        descriptors_, prev_frame->descriptors_,
        matches_,
        true // print result
        );
    }
};

} // namespace my_slam

#endif // FRAME_H
