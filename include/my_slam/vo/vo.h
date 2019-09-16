

#ifndef MY_SLAM_VO_H
#define VO_H

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// my

#include "my_slam/basics/basics.h"
#include "my_slam/vo/vo_io.h"
#include "my_slam/basics/config.h"
#include "my_slam/basics/opencv_funcs.h"

#include "my_slam/geometry/camera.h"
#include "my_slam/geometry/feature_match.h"
#include "my_slam/geometry/motion_estimation.h"

#include "my_slam/common_include.h"
#include "my_slam/vo/frame.h"
#include "my_slam/vo/map.h"
#include "my_slam/vo/mappoint.h"
#include "my_slam/vo/vo_commons.h"

namespace my_slam
{
namespace vo
{

class VisualOdometry
{

public:
  typedef std::shared_ptr<VisualOdometry> Ptr;
  VisualOdometry();

  void addFrame(vo::Frame::Ptr frame); // Add a new frame to the visual odometry system and compute its pose.

  bool isInitialized();                         // Is visual odometry initialized.
  Frame::Ptr getPrevRef() { return prev_ref_; } // for run_vo.cpp to draw result
  Map::Ptr getMap() { return map_; }            // for run_vo.cpp to draw result

private:
  enum VOState
  {
    BLANK,
    DOING_INITIALIZATION,
    DOING_TRACKING,
    LOST
  };
  VOState vo_state_;

private:
  // Frame
  Frame::Ptr curr_ = nullptr;         // current frame
  Frame::Ptr prev_ = nullptr;         // previous frame
  Frame::Ptr ref_ = nullptr;          // reference keyframe
  Frame::Ptr prev_ref_ = nullptr;     // set prev_ref_ as ref_ at the beginning of addFrame (only for displaying purpose)
  Frame::Ptr newest_frame_ = nullptr; // temporarily store the newest frame
  cv::Mat prev_T_w_c_;                // pos of previous frame
  std::deque<Frame::Ptr> frames_buff_;

  // Map
  Map::Ptr map_;

  // Map features
  vector<cv::KeyPoint> keypoints_curr_;
  cv::Mat descriptors_curr_;

  vector<cv::Point3f> matched_pts_3d_in_map_;
  vector<int> matched_pts_2d_idx_;

  // Parameters
  const int kBuffSize_ = 20; // How much prev frames to store.

private: // functions
  // Push a frame to the buff.
  void pushFrameToBuff_(Frame::Ptr frame)
  {
    frames_buff_.push_back(frame);
    if (frames_buff_.size() > kBuffSize_)
      frames_buff_.pop_front();
  }

  // Initialization
  void estimateMotionAnd3DPoints_();

  // Check if visual odmetry is good to be initialized.
  bool isVoGoodToInit_();

  // Remove bad triangulation points
  // Change "pts3d_in_curr", return a new "inlier_matches".
  void retainGoodTriangulationResult_();

public: // ------------------------------- Tracking -------------------------------
  bool checkLargeMoveForAddKeyFrame_(Frame::Ptr curr, Frame::Ptr ref);
  void optimizeMap_();
  bool poseEstimationPnP_();

public: // ------------------------------- Mapping -------------------------------
  void addKeyFrame_(Frame::Ptr keyframe);
  void pushCurrPointsToMap_();
  void getMappointsInCurrentView_(
      vector<MapPoint::Ptr> &candidate_mappoints_in_map,
      cv::Mat &corresponding_mappoints_descriptors);
  double getViewAngle_(Frame::Ptr frame, MapPoint::Ptr point);

public: // ------------------------------- BundleAdjustment -------------------------------
  void callBundleAdjustment_();
};

} // namespace vo
} // namespace my_slam
#endif // FRAME_H
