
// Functions for displaying 3D points and camera using PCL library.



#ifndef PCL_VISUALIZATIONS_H
#define PCL_VISUALIZATIONS_H

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

// set the initial viewing angle (Copied from pcl website)
void  setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

// set the initial viewing angle (Overload by using pos and euler angles)
void  setViewerPose (pcl::visualization::PCLVisualizer& viewer,
    double x, double y, double z, double ea_x, double ea_y, double ea_z);

} // namespace my_display

#endif