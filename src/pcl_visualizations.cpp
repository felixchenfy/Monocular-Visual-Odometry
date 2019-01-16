
#include "my_display/pcl_visualizations.h"
#include "my_basics/eigen_funcs.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace my_display
{


// initialize the viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer>
initPointCloudViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                     const string &viewer_name, const string &cloud_name, const string &camera_frame_name)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer(viewer_name));

    // Add a RGB point cloud
    const int POINT_SIZE = 3;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_setting(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, color_setting, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, cloud_name);

    // Add Coordinate system
    viewer->addCoordinateSystem(1.0, camera_frame_name);

    // Other properties
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    return (viewer);
}

void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

void setViewerPose(pcl::visualization::PCLVisualizer& viewer,
    double x, double y, double z, double rotaxis_x, double rotaxis_y, double rotaxis_z){
    Eigen::Affine3d T = my_basics::getAffine3d(x, y, z, rotaxis_x, rotaxis_y, rotaxis_z); 
    setViewerPose(viewer, T.cast<float>());
}


void setPointColor(pcl::PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    point.rgb = *reinterpret_cast<float *>(&rgb);
}

void setPointPos(pcl::PointXYZRGB &point, float x, float y, float z)
{
    point.x = x;
    point.y = y;
    point.z = z;
}
void setPointPos(pcl::PointXYZRGB &point, double x, double y, double z)
{
    setPointPos(point, (float)x, float(y), float(z));
}
void setPointPos(pcl::PointXYZRGB &point, cv::Mat p)
{
    setPointPos(point, p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
}
void setPointPos(pcl::PointXYZ &point, float x, float y, float z)
{
    point.x = x;
    point.y = y;
    point.z = z;
}
void setPointPos(pcl::PointXYZ &point, double x, double y, double z)
{
    setPointPos(point, (float)x, float(y), float(z));
}
void setPointPos(pcl::PointXYZ &point, cv::Mat p)
{
    setPointPos(point, p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
}
} // namespace my_display
