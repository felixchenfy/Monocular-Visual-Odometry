
// Functions for displaying 3D points and camera using PCL library.



#ifndef pcl_display_LIB_H
#define pcl_display_LIB_H

#include <memory>
#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for cv::Rodrigues

#include "my_basics/eigen_funcs.h"

namespace my_display
{
using namespace std;
using namespace pcl;

typedef PointCloud<PointXYZRGB>::Ptr CloudPtr;

void setPointColor(PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b);
void setPointPos(PointXYZRGB &point, float x, float y, float z);
void setPointPos(PointXYZRGB &point, double x, double y, double z);
void setPointPos(PointXYZRGB &point, cv::Mat p);
void setPointPos(PointXYZ &point, float x, float y, float z);
void setPointPos(PointXYZ &point, double x, double y, double z);
void setPointPos(PointXYZ &point, cv::Mat p);

// initialize the viewer
boost::shared_ptr<visualization::PCLVisualizer> initPointCloudViewer(const string &viewer_name);

// Add a cloud to the viewer, and returned the cloud ptr.
PointCloud<PointXYZRGB>::Ptr addPointCloud(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
    const string cloud_name, int POINT_SIZE = 3);


// set the initial viewing angle (Copied from pcl website)
void setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

// set the initial viewing angle (Overload by using pos and euler angles)
void setViewerPose (visualization::PCLVisualizer& viewer,
    double x, double y, double z, double rot_axis_x, double rot_axis_y, double rot_axis_z);

} // namespace my_display

#endif