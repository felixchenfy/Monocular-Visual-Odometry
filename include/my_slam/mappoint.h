
#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "my_slam/common_include.h"
#include "my_slam/frame.h"

namespace my_slam
{
using namespace std; 
using namespace cv;

// class Frame;
class MapPoint
{
public: // Variables
    typedef shared_ptr<MapPoint> Ptr;

    static unsigned long factory_id_; 
    unsigned long id_;
    Mat pos_;
    Mat norm_; // Vector pointing from camera center to the point
    vector<unsigned char> color_; // r,g,b
    Mat descriptor_; // Descriptor for matching 

public: // Properties for constructing local mapping

    bool        good_;      // wheter a good point 
    int         matched_times_;     // being an inliner in pose estimation
    int         visible_times_;     // being visible in current frame 
    
    // TO DO: Add property of which frame this mappoint is as inlier. This is for optimization.
    vector<Frame::Ptr>    matched_frames_;
    vector<Point2f>    uv_in_matched_frames_;


public: // Constructor
    MapPoint (const Mat& pos,  const Mat& descriptor, const Mat& norm,
        unsigned char r=0, unsigned char g=0, unsigned char b=0);

public: // Functions


    // list<Frame*>    observed_frames_;   // key-frames that can observe this point 
    
    // MapPoint();
    // MapPoint( 
    //     unsigned long id, 
    //     const Vector3d& position, 
    //     const Vector3d& norm, 
    //     Frame* frame=nullptr, 
    //     const Mat& descriptor=Mat() 
    // );
    
    // inline cv::Point3f getPositionCV() const {
    //     return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    // }
    
    // static MapPoint::Ptr createMapPoint();
    // static MapPoint::Ptr createMapPoint( 
    //     const Vector3d& pos_world, 
    //     const Vector3d& norm_,
    //     const Mat& descriptor,
    //     Frame* frame );
};
}

#endif // MAPPOINT_H
