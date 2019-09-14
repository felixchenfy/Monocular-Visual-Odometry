
#ifndef MY_SLAM_MAPPOINT_H
#define MY_SLAM_MAPPOINT_H

#include "my_slam/common_include.h"
#include "my_slam/vo/frame.h"

namespace my_slam
{
namespace vo
{
using namespace std;
using namespace cv;

// class Frame;
class MapPoint
{
public: // Basics Properties
    typedef std::shared_ptr<MapPoint> Ptr;

    static int factory_id_;
    int id_;
    cv::Point3f pos_;
    cv::Mat norm_;                    // Vector pointing from camera center to the point
    vector<unsigned char> color_; // r,g,b
    cv::Mat descriptor_;              // Descriptor for matching

public:                 // Properties for constructing local mapping
    bool good_;         // TODO: determine wheter a good point
    int matched_times_; // being an inliner in pose estimation
    int visible_times_; // being visible in current frame

public: // Functions
    MapPoint(const cv::Point3f &pos, const cv::Mat &descriptor, const cv::Mat &norm,
             unsigned char r = 0, unsigned char g = 0, unsigned char b = 0);
    // void resetPos(cv::Point3f pos);
};

} // namespace vo
} // namespace my_slam

#endif // MAPPOINT_H
