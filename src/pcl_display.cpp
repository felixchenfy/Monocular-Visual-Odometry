
#include "my_display/pcl_display.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace my_display
{

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

// initialize the viewer
boost::shared_ptr<pcl::visualization::PCLVisualizer>
initPointCloudViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
        const string &viewer_name, const string &cloud_name, const string &camera_frame_name)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer(viewer_name));

    // Add a RGB point cloud
    const int POINT_SIZE = 3;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, cloud_name);

    // Add Coordinate system
    viewer->addCoordinateSystem(1.0, camera_frame_name);

    // Other properties
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    return (viewer);
}

// --------------------- Class definition -----------------------

// constructor
PclViewer::PclViewer(const string &viewer_name): 
    point_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
{

    viewer_name_ = viewer_name;
    point_cloud_name_ = "the 0th point cloud";
    camera_frame_name_ = "the 0th camera";

    viewer_ = initPointCloudViewer(
            point_cloud_ptr_, viewer_name_, point_cloud_name_, camera_frame_name_);
}

void PclViewer::updateCameraPose(const cv::Mat &R_vec, const cv::Mat &t){
    cam_R_vec_ = R_vec;
    cam_t_ = t;
}

void PclViewer::addPoint(const cv::Mat pt_3d_pos_in_world, uint8_t r, uint8_t g, uint8_t b){
    pcl::PointXYZRGB point;
    setPointPos(point, pt_3d_pos_in_world);
    setPointColor(point, r, g, b);
    point_cloud_ptr_->points.push_back(point);
}

void PclViewer::update(){
    // Update camera
    Eigen::Affine3f T_affine = my_basics::transCVMatRt2Affine3d(cam_R_vec_, cam_t_).cast<float>();
    viewer_->removeCoordinateSystem(camera_frame_name_);
    viewer_->addCoordinateSystem(1.0, T_affine, camera_frame_name_, 0);

    // Update point
    viewer_->updatePointCloud(point_cloud_ptr_, point_cloud_name_);
}
void PclViewer::spinOnce(unsigned int millisecond){
    viewer_->spinOnce(millisecond);
}
bool PclViewer::wasStopped(){
    return viewer_ ->wasStopped();
}


} // namespace my_display
