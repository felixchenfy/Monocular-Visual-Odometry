
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
public: // Basics Properties
    typedef shared_ptr<MapPoint> Ptr;

    static int factory_id_; 
    int id_;
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


public: // Functions
    MapPoint (const Mat& pos,  const Mat& descriptor, const Mat& norm,
        unsigned char r=0, unsigned char g=0, unsigned char b=0);
    void resetPos(Point3f pos);

};
}

#endif // MAPPOINT_H
