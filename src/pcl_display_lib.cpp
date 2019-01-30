
#include "my_display/pcl_display_lib.h"
#include "my_basics/eigen_funcs.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace my_display
{



// initialize the viewer
boost::shared_ptr<visualization::PCLVisualizer>
initPointCloudViewer(const string &viewer_name)
{
    boost::shared_ptr<visualization::PCLVisualizer> viewer(
        new visualization::PCLVisualizer(viewer_name));

    // Other properties
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    return (viewer);
}

PointCloud<PointXYZRGB>::Ptr addPointCloud(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, 
    const string cloud_name, int POINT_SIZE)
{
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    // cloud.reset(new PointCloud<PointXYZRGB>);
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> color_setting(cloud);
    viewer->addPointCloud<PointXYZRGB>(cloud, color_setting, cloud_name);
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, cloud_name);
    return cloud;
}


void setViewerPose(visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

void setViewerPose(visualization::PCLVisualizer& viewer,
    double x, double y, double z, double rot_axis_x, double rot_axis_y, double rot_axis_z){
    Eigen::Affine3d T = my_basics::getAffine3d(x, y, z, rot_axis_x, rot_axis_y, rot_axis_z); 
    setViewerPose(viewer, T.cast<float>());
}


void setPointColor(PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    point.rgb = *reinterpret_cast<float *>(&rgb);
}

void setPointPos(PointXYZRGB &point, float x, float y, float z)
{
    point.x = x;
    point.y = y;
    point.z = z;
}
void setPointPos(PointXYZRGB &point, double x, double y, double z)
{
    setPointPos(point, (float)x, float(y), float(z));
}
void setPointPos(PointXYZRGB &point, cv::Mat p)
{
    setPointPos(point, p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
}
void setPointPos(PointXYZ &point, float x, float y, float z)
{
    point.x = x;
    point.y = y;
    point.z = z;
}
void setPointPos(PointXYZ &point, double x, double y, double z)
{
    setPointPos(point, (float)x, float(y), float(z));
}
void setPointPos(PointXYZ &point, cv::Mat p)
{
    setPointPos(point, p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
}
} // namespace my_display
