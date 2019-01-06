
#include "my_display/pcl_viewer.h"
#include "my_display/pcl_visualizations.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace my_pcl_viewer
{
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_;
} // namespace my_pcl_viewer

using namespace my_pcl_viewer;
namespace my_display
{

// --------------------- Class definition -----------------------

// constructor
PclViewer::PclViewer(const string &viewer_name,
                     double x, double y, double z,
                     double ea_x, double ea_y, double ea_z)
{
    // names
    viewer_name_ = viewer_name;
    point_cloud_name_ = "the 0th point cloud";
    camera_frame_name_ = "the 0th camera";

    // pointer
    point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewer_ = initPointCloudViewer(
        point_cloud_ptr_, viewer_name_, point_cloud_name_, camera_frame_name_);
    viewer_->addCoordinateSystem(1.0, "fixed world frame");

    // set viewer angle
    setViewerPose(*viewer_, x, y, z, ea_x, ea_y, ea_z);
}

void PclViewer::updateCameraPose(const cv::Mat &R_vec, const cv::Mat &t)
{
    cam_R_vec_ = R_vec.clone();
    cam_t_ = t.clone();
}

void PclViewer::addPoint(const cv::Mat pt_3d_pos_in_world, uchar r, uchar g, uchar b)
{
    pcl::PointXYZRGB point;
    setPointPos(point, pt_3d_pos_in_world);
    setPointColor(point, r, g, b);
    point_cloud_ptr_->points.push_back(point);
}

void PclViewer::update()
{
    // Update camera
    Eigen::Affine3f T_affine = my_basics::transCVMatRt2Affine3d(cam_R_vec_, cam_t_).cast<float>();
    viewer_->removeCoordinateSystem(camera_frame_name_);
    viewer_->addCoordinateSystem(1.0, T_affine, camera_frame_name_, 0);

    // Update point
    // viewer_->updatePointCloud(point_cloud_ptr_, point_cloud_name_);
}
void PclViewer::spinOnce(unsigned int millisecond)
{
    viewer_->spinOnce(millisecond);
}
bool PclViewer::wasStopped()
{
    return viewer_->wasStopped();
}

} // namespace my_display