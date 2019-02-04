

#ifndef VO_H
#define VO_H

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// my

#include "my_basics/basics.h"
#include "my_basics/io.h"
#include "my_basics/config.h"
#include "my_basics/opencv_funcs.h"

#include "my_geometry/camera.h"
#include "my_geometry/feature_match.h"
#include "my_geometry/motion_estimation.h"

#include "my_slam/common_include.h"
#include "my_slam/frame.h"
#include "my_slam/map.h"
#include "my_slam/mappoint.h"
#include "my_slam/commons.h"

namespace my_slam
{
using namespace std;
using namespace cv;
using namespace my_geometry;

class VisualOdometry
{

public: // ------------------------------- Member variables -------------------------------
  typedef shared_ptr<VisualOdometry> Ptr;

  enum VOState
  {
    BLANK,
    INITIALIZATION,
    OK,
    LOST
  };
  VOState vo_state_;

  // Frame
  Frame::Ptr curr_;
  Frame::Ptr ref_;
  Frame::Ptr newest_frame_; // temporarily store the newest frame
  Mat prev_T_w_c_;          // pos of previous frame
  deque<Frame::Ptr> frames_buff_;

  // Map
  Map::Ptr map_;

  // Map features
  vector<KeyPoint> keypoints_curr_;
  Mat descriptors_curr_;
  
  vector<Point3f> matched_pts_3d_in_map_;
  vector<int> matched_pts_2d_idx_;

// ================================ Functions ================================
public: // basics
  VisualOdometry();
  void addFrame(my_slam::Frame::Ptr frame);
  void pushFrameToBuff(Frame::Ptr frame){
    const int BUFF_SIZE=20;
    frames_buff_.push_back(frame);
    if(frames_buff_.size()>BUFF_SIZE)
      frames_buff_.pop_front();
  }
public: // ------------------------------- Initialization -------------------------------
  void estimateMotionAnd3DPoints();
  bool checkIfVoGoodToInit(int checkIfVoGoodToInit);
  bool isInitialized();

public: // ------------------------------- Triangulation -------------------------------
  // 1. motion_estimation.h: helperTriangulatePoints
  // 2. Some triangulated points have a small viewing angle between two frames.
  //    Remove these points: change "pts3d_in_curr", return a new "inlier_matches"
  vector<DMatch> retainGoodTriangulationResult(Frame::Ptr ref, Frame::Ptr curr,
    const vector<DMatch> inlier_matches, vector<Point3f> pts3d_in_curr);

public: // ------------------------------- Tracking -------------------------------
  // void find3Dto2DCorrespondences()
  bool checkLargeMoveForAddKeyFrame(Frame::Ptr curr, Frame::Ptr ref);
  void optimizeMap();
  void poseEstimationPnP();

public: // ------------------------------- Mapping -------------------------------
  void addKeyFrame(Frame::Ptr frame);
  void getMappointsInCurrentView(
      vector<MapPoint::Ptr> &candidate_mappoints_in_map,
      Mat &corresponding_mappoints_descriptors);
  void pushCurrPointsToMap();
  double getViewAngle(Frame::Ptr frame, MapPoint::Ptr point);
  
public: // ------------------------------- BundleAdjustment -------------------------------
  void callBundleAdjustment();
};

} // namespace my_slam

#endif // FRAME_H
