
#include <boost/timer.hpp>

#include "myslam/visual_odometry.h"
#include "myslam/config.h"

namespace myslam
{

VisualOdometry::VisualOdometry() : 
    state_(EMPTY), ref_(nullptr), curr_(nullptr), map_(new Map), 
    matcher_flann_(new cv::flann::LshIndexParams(5, 10, 2))
{

    // max_num_lost_       = Config::get<float> ( "max_num_lost" );
    // min_inliers_        = Config::get<int> ( "min_inliers" );
    // key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    // key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    // map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );

    // Set for matching descriptors
    match_ratio_ = Config::get<float>("match_ratio");
    // num_lost_(0), num_inliers_(0)


    // Set for ORB properties
    num_of_features_ = Config::get<int>("number_of_features");
    scale_factor_ = Config::get<double>("scale_factor");
    level_pyramid_ = Config::get<int>("level_pyramid");
    orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{

    boost::timer timer;
    bool res = true;
    switch (state_)
    {
    case EMPTY:
    {
        // state_ = INITIALIZING;
        curr_ = ref_ = frame;
        // extract features from first frame and add them into map
        extractKeyPoints();
        computeDescriptors();
        break;
    }
        // case INITIALIZING:
        // {
        //     state_ = OK;
        //     curr_ = ref_ = frame;
        //     // extract features from first frame and add them into map
        //     extractKeyPoints();
        //     computeDescriptors();
        //     addKeyFrame(); // the first frame is a key-frame
        //     break;
        // }
    }

    // Print result
    cout << endl;
    cout << "Add frame " << frame->id_ << endl;
    cout << "time cost = " << timer.elapsed()*1000 << "ms" << endl;
    cout << "number of keypoints = " << keypoints_curr_.size() << endl;      
    return res;
}
}