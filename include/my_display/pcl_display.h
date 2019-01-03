
// Functions for displaying 3D points and camera using PCL library.

#ifndef PCL_DISPLAY_H
#define PCL_DISPLAY_H

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for cv::Rodrigues

namespace my_display{

void setPointColor(pcl::PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b);
void setPointPos(pcl::PointXYZRGB &point, float x, float y, float z);
void setPointPos(pcl::PointXYZRGB &point, double x, double y, double z);
void setPointPos(pcl::PointXYZRGB &point, cv::Mat p);

}

#endif