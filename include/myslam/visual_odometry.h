
#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include <opencv2/features2d/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"

namespace myslam
{

class VisualOdometry
{

  private:
  public: // variables
    typedef shared_ptr<VisualOdometry> Ptr;

    enum VOState
    {
        EMPTY = 0,
        INITIALIZING = 1,
        OK = 2,
        LOST = 3
    };
    VOState state_; // current VO status

    Map::Ptr map_; // Pointer to the map.
    // This is for:
    //      map_->map_points_

    Frame::Ptr ref_;  // reference frame (Currently, it's the previous frame)
    Frame::Ptr curr_; // current frame

    // image local features
    cv::Ptr<cv::ORB> orb_;
    vector<cv::KeyPoint> keypoints_curr_;
    Mat descriptors_curr_;

    // feature matching
    cv::FlannBasedMatcher matcher_flann_;      // flann matcher
    vector<MapPoint::Ptr> match_3d_MapPoints_; // matched 3d points
    vector<int> match_2d_keypts_index_;        // matched 2d pixels (index of kp_curr)

    // SE3 T_c_w_estimated_;    // the estimated pose of current frame
    // int num_inliers_;        // number of inlier features in icp
    // int num_lost_;           // number of lost times

    // --------------- parameters ---------------

    // matching
    double match_ratio_; // used here: double threshold_dis = max<double>(min_dis * match_ratio_, 30.0);

    // ORB
    int num_of_features_; // number of features
    double scale_factor_; // scale in image pyramid
    int level_pyramid_;   // number of pyramid levels

    

  public: // functions
    VisualOdometry();
    ~VisualOdometry(){}
    bool addFrame(Frame::Ptr frame); // add a new frame

  protected:
    // inner operation
    void extractKeyPoints() { orb_->detect(curr_->rgb_img_, keypoints_curr_); }
    void computeDescriptors() { orb_->compute(curr_->rgb_img_, keypoints_curr_, descriptors_curr_); }
    // void featureMatching();

  // ------------------ IO -------------------
  public: 
  vector<cv::KeyPoint>& getCurrentKeypoints(){return keypoints_curr_;};

};

} // namespace myslam
#endif