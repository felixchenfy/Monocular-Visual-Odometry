

#ifndef FRAME_H
#define FRAME_H

#include "my_slam/common_include.h"
#include "my_basics/opencv_funcs.h"
#include "my_geometry/camera.h"
#include "my_geometry/feature_match.h"

namespace my_slam
{
using namespace std;
using namespace cv;
using namespace my_basics;
using namespace my_geometry;

typedef struct PtConn_
{
  int pt_curr_idx;
  int pt_ref_idx;
  int pt_map_idx;
} PtConn;

class Frame
{
private:
public:
  typedef std::shared_ptr<Frame> Ptr;
  static int factory_id_;

public:
  int id_;  // id of this frame
  double time_stamp_; // when it is recorded

  // -- image features
  Mat rgb_img_;
  vector<KeyPoint> keypoints_;
  Mat descriptors_;
  vector<vector<unsigned char>> kpts_colors_; // rgb colors

  // -- Matches with reference frame
  //  for (1) E/H at initialization stage and (2) triangulating 3d points at all stages.
  vector<DMatch> matches_with_ref_;         // matches with reference frame
  vector<DMatch> inliers_matches_with_ref_; // matches that satisify E or H's constraints, and
  vector<DMatch> inliers_matches_for_3d_;   // matches whose triangulation result is good.
  vector<Point3f> inliers_pts3d_;           // 3d points triangulated from inliers_matches_for_3d_

  // -- Matches with map points (for PnP)
  vector<DMatch> matches_with_map_; // inliers matches index with respect to all the points

  // -- Information for keypoints' association with map points (for bundle adjustment)
  // Whether a keypoint has associatation with a map point. -1 means no. If yes, the value is the index.
  vector<bool> vb_is_mappoint_; // size = keypoints_.size()
  vector<int> vi_mappoint_idx_; // size = keypoints_.size()
  // Store the keypoint that is in curr and ref frame, as well as being a mappoint.
  // Store these points' indices in curr, ref, and map.
  vector<PtConn> inliers_connections_; // size = inliers_pts3d_.size()

  // -- Camera
  my_geometry::Camera::Ptr camera_;

  // -- Current pose
  Mat T_w_c_; // transform from camera to world

public:
  Frame() {}
  ~Frame() {}
  static Frame::Ptr createFrame(Mat rgb_img, my_geometry::Camera::Ptr camera, double time_stamp = -1);

public: // Below are deprecated. These were used in the two-frame-matching vo.
  void extractKeyPoints()
  {
    my_geometry::extractKeyPoints(rgb_img_, keypoints_);
  }
  void computeDescriptors()
  {
    my_geometry::computeDescriptors(rgb_img_, keypoints_, descriptors_);
    kpts_colors_.clear();
    for (KeyPoint kpt : keypoints_)
    {
      int x = floor(kpt.pt.x), y = floor(kpt.pt.y);
      kpts_colors_.push_back(getPixelAt(rgb_img_, x, y));
    }
    vb_is_mappoint_ = vector<bool>(keypoints_.size(), false);
    vi_mappoint_idx_ = vector<int>(keypoints_.size(), -1);
    
  };
  void matchFeatures(Frame::Ptr prev_frame)
  {
    my_geometry::matchFeatures(
        // descriptors_, prev_frame->descriptors_,
        prev_frame->descriptors_, descriptors_,
        matches_with_ref_,
        true // print result
    );
  }
  bool isInFrame(const Point3f &p_world);
  bool isInFrame(const Mat &p_world);
  Mat getCamCenter();
};

} // namespace my_slam

#endif // FRAME_H
