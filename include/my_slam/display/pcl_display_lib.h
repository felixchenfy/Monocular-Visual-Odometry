
/* @brief
 * Functions for displaying 3D points and camera position by using PCL library.
 */

#ifndef MY_SLAM_PCL_DISPLAY_LIB_H
#define MY_SLAM_PCL_DISPLAY_LIB_H

#include "my_slam/common_include.h"
#include "my_slam/basics/eigen_funcs.h"

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace my_slam
{
namespace display
{

void setPointColor(pcl::PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b);
void setPointPos(pcl::PointXYZRGB &point, float x, float y, float z);
void setPointPos(pcl::PointXYZRGB &point, double x, double y, double z);
void setPointPos(pcl::PointXYZRGB &point, cv::Mat p);
void setPointPos(pcl::PointXYZ &point, float x, float y, float z);
void setPointPos(pcl::PointXYZ &point, double x, double y, double z);
void setPointPos(pcl::PointXYZ &point, cv::Mat p);

// Initialize the viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer> initPointCloudViewer(const string &viewer_name);

// Add a cloud to the viewer, and returned the cloud ptr.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr addPointCloud(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
    const string cloud_name, int POINT_SIZE = 3);

// Set the initial viewing angle (Copied from pcl website)
void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose);

// Set the initial viewing angle (Overload by using pos and euler angles)
void setViewerPose(pcl::visualization::PCLVisualizer &viewer,
                   double x, double y, double z, double rot_axis_x, double rot_axis_y, double rot_axis_z);

} // namespace display
} // namespace my_slam

#endif