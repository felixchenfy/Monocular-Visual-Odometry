
#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include <opencv2/features2d/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"
#include "myslam/feature_match.h"

namespace myslam
{

class Buff_FramesWithFeatures{
public:
   deque<Frame::Ptr> buff_frames_; // use deque for const time of pop_front
   deque<vector<cv::KeyPoint>> buff_keypoints_;
   deque<cv::Mat> buff_descriptors_;
  void push_back(Frame::Ptr frame, vector<cv::KeyPoint> keypoints, cv::Mat descriptors){
    buff_frames_.push_back(frame);
    buff_keypoints_.push_back(keypoints);
    buff_descriptors_.push_back(descriptors);
  }
  void pop_ith(int i){
    buff_frames_.erase(buff_frames_.begin()+i);
    buff_keypoints_.erase(buff_keypoints_.begin()+i);
    buff_descriptors_.erase(buff_descriptors_.begin()+i);
  }
  void pop_front(){pop_ith(0);}
  int size(){return buff_descriptors_.size();}
};

class VisualOdometry
{

  private:
  public: // variables
    typedef shared_ptr<VisualOdometry> Ptr;

    enum VOState
    {
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
    vector<MapPoint::Ptr> matched_3d_MapPoints_; // matched 3d points
    vector<int> matched_2d_keypts_index_;        // matched 2d pixels (index of kp_curr)

    // SE3 T_c_w_estimated_;    // the estimated pose of current frame
    // int num_inliers_;        // number of inlier features in icp
    // int num_lost_;           // number of lost times

    // --------------- parameters ---------------
    // ORB
    int num_of_features_; // number of features
    double scale_factor_; // scale in image pyramid
    int level_pyramid_;   // number of pyramid levels

    

  public: // functions
    VisualOdometry();
    ~VisualOdometry(){}
    bool addFrame(Frame::Ptr frame); // add a new frame
    


  protected: // inner operation
    void extractKeyPoints() { orb_->detect(curr_->rgb_img_, keypoints_curr_); }
    void computeDescriptors() { orb_->compute(curr_->rgb_img_, keypoints_curr_, descriptors_curr_); }

    // // Currently not used !!! This will be used after triangulation !!!
    // void matchFeatures_withMapPoints(); 

  // ------------------ IO -------------------
  public: 
  vector<cv::KeyPoint>& getCurrentKeypoints(){return keypoints_curr_;};

};

} // namespace myslam
#endif