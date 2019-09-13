

#ifndef FRAME_H
#define FRAME_H

#include "my_slam/vo/common_include.h"
#include "my_slam/basics/opencv_funcs.h"
#include "my_slam/geometry/camera.h"
#include "my_slam/geometry/feature_match.h"

namespace vo
{
using namespace std;
using namespace cv;
using namespace basics;
using namespace geometry;

typedef struct PtConn_
{
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

  // -- Matches with reference keyframe (for E/H or PnP)
  //  for (1) E/H at initialization stage and (2) triangulating 3d points at all stages.
  vector<DMatch> matches_with_ref_;         // matches with reference frame
  vector<DMatch> inliers_matches_with_ref_; // matches that satisify E or H's constraints, and
  
  // -- vectors for triangulation
  vector<double> triangulation_angles_of_inliers_;
  vector<DMatch> inliers_matches_for_3d_;   // matches whose triangulation result is good.
  vector<Point3f> inliers_pts3d_;           // 3d points triangulated from inliers_matches_for_3d_
  unordered_map<int, PtConn> inliers_to_mappt_connections_; // curr idx -> idx in ref, and map
  
  // -- Matches with map points (for PnP)
  vector<DMatch> matches_with_map_; // inliers matches index with respect to all the points

  // -- Camera
  geometry::Camera::Ptr camera_;

  // -- Current pose
  Mat T_w_c_; // transform from world to camera

public:
  Frame() {}
  ~Frame() {}
  static Frame::Ptr createFrame(Mat rgb_img, geometry::Camera::Ptr camera, double time_stamp = -1);

public: // Below are deprecated. These were used in the two-frame-matching vo.
  void clearNoUsed(){
    // rgb_img_.release();
    kpts_colors_.clear();
    matches_with_ref_.clear();
    inliers_matches_with_ref_.clear();
    inliers_matches_for_3d_.clear();
    matches_with_map_.clear();
  }
  void extractKeyPoints()
  {
    geometry::extractKeyPoints(rgb_img_, keypoints_);
  }
  void computeDescriptors()
  {
    geometry::computeDescriptors(rgb_img_, keypoints_, descriptors_);
    kpts_colors_.clear();
    for (KeyPoint kpt : keypoints_)
    {
      int x = floor(kpt.pt.x), y = floor(kpt.pt.y);
      kpts_colors_.push_back(getPixelAt(rgb_img_, x, y));
    }
  };
  void matchFeatures(Frame::Ptr prev_frame)
  {
    geometry::matchFeatures(
        // descriptors_, prev_frame->descriptors_,
        prev_frame->descriptors_, descriptors_,
        matches_with_ref_,
        true // print result
    );
  }
  bool isInFrame(const Point3f &p_world);
  bool isInFrame(const Mat &p_world);
  bool isMappoint(int idx){
    bool not_find = inliers_to_mappt_connections_.find(idx)==inliers_to_mappt_connections_.end();
    return !not_find;
  }
  Mat getCamCenter();
};

} // namespace vo

#endif // FRAME_H
