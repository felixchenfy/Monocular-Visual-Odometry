
// Functions and a class "PclViewer"
//      for displaying 3D points and camera using PCL library.

/*
The reason I make a class for this pcl's displaying class:
PCL compiles really really slow. 
If I put pcl codes in my "main.cpp", then it will be tooooo slow to debug.
So I put them in a class like this, 
    and i was hoping that there are less things to compile.
However, it's not as fast as I expected -- The compiling and linking still take like 10 seconds.
*/

#ifndef PCL_DISPLAY_H
#define PCL_DISPLAY_H

#include <memory>
#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for cv::Rodrigues

#include "my_basics/eigen_funcs.h"

using namespace std;

namespace my_display
{

void setPointColor(pcl::PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b);
void setPointPos(pcl::PointXYZRGB &point, float x, float y, float z);
void setPointPos(pcl::PointXYZRGB &point, double x, double y, double z);
void setPointPos(pcl::PointXYZRGB &point, cv::Mat p);

// initialize the viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer>
initPointCloudViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                const string &viewer_name, const string &cloud_name, const string &camera_frame_name_);

class PclViewer
{

  public:
    // ----------------- variables -----------------
    typedef shared_ptr<PclViewer> Ptr;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    string viewer_name_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_;
    string point_cloud_name_;

    string camera_frame_name_;
    cv::Mat cam_R_vec_;
    cv::Mat cam_t_;

  public:
    // ----------------- constructor -----------------
    PclViewer(const string &viewer_name);
    void createViewer(const string &viewer_name);

  public:
    void updateCameraPose(const cv::Mat &R_vec, const cv::Mat &t);
    void addPoint(const cv::Mat pt_3d_pos_in_world, uint8_t r, uint8_t g, uint8_t b);
    void update();
    void spinOnce(unsigned int millisecond);
    bool wasStopped();
};

} // namespace my_display

#endif