
#include "my_display/pcl_viewer.h"
#include "my_display/pcl_visualizations.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace my_pcl_viewer
{ // store pcl related class members here instead of .h, in order to speed up compiling.
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_keypoints_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_cam_traj_;
} // namespace my_pcl_viewer

using namespace my_pcl_viewer;
namespace my_display
{

// --------------------- Class definition -----------------------

// constructor
PclViewer::PclViewer(const string &viewer_name,
                     double x, double y, double z,
                     double rotaxis_x, double rotaxis_y, double rotaxis_z)
{
    // Set names
    viewer_name_ = viewer_name;
    point_cloud_name_ = "cloud: keypoints";
    camera_frame_name_ = "Coord: camera";
    camera_traj_name_ = "cloud: camera traj";

    // Set viewer
    cloud_ptr_keypoints_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewer_ = initPointCloudViewer(
        cloud_ptr_keypoints_, viewer_name_, point_cloud_name_, camera_frame_name_);
    viewer_->addCoordinateSystem(1.0, "fixed world frame");

    // Add another point cloud for showing camera's trajectory
    cloud_ptr_cam_traj_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_setting(cloud_ptr_cam_traj_);
    viewer_->addPointCloud<pcl::PointXYZRGB>(cloud_ptr_cam_traj_, color_setting, camera_traj_name_);
    const int POINT_SIZE = 3;
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, camera_traj_name_);

    // Set viewer angle
    setViewerPose(*viewer_, x, y, z, rotaxis_x, rotaxis_y, rotaxis_z);
}

void PclViewer::updateCameraPose(const cv::Mat &R_vec, const cv::Mat &t)
{
    static int cnt_cam = 0;
    // Before update camera pos, add a line and a point for display
    if (!cam_t_.empty())
    {
        // add a line between current and old camera pos
        pcl::PointXYZ cam_pos_old, cam_pos_new;
        setPointPos(cam_pos_old, cam_t_);
        setPointPos(cam_pos_new, t);
        viewer_->addLine<pcl::PointXYZ>(cam_pos_old, cam_pos_new, "line-cam" + to_string(cnt_cam++));

        // add a point of the new camera pos
        pcl::PointXYZRGB point;
        setPointPos(point, t);
        unsigned char r = 255, g = 255, b = 255;
        setPointColor(point, r, g, b);
        cloud_ptr_cam_traj_->points.push_back(point);
    }
    cam_R_vec_ = R_vec.clone();
    cam_t_ = t.clone();
}

void PclViewer::addPoint(const cv::Mat pt_3d_pos_in_world, uchar r, uchar g, uchar b)
{
    pcl::PointXYZRGB point;
    setPointPos(point, pt_3d_pos_in_world);
    setPointColor(point, r, g, b);
    cloud_ptr_keypoints_->points.push_back(point);
}
void PclViewer::deletePoints(){
    cloud_ptr_keypoints_->points.clear();
}
void PclViewer::update()
{
    static int cnt_frame = 0;
    // Update camera
    Eigen::Affine3f T_affine = my_basics::transCVMatRt2Affine3d(cam_R_vec_, cam_t_).cast<float>();
    viewer_->removeCoordinateSystem(camera_frame_name_);
    viewer_->addCoordinateSystem(1.0, T_affine, camera_frame_name_, 0);

    // Update point
    viewer_->updatePointCloud(cloud_ptr_keypoints_, point_cloud_name_);
    viewer_->updatePointCloud(cloud_ptr_cam_traj_, camera_traj_name_);

    // Update text
    int xpos = 20, ypos = 30;
    char str[512];
    sprintf(str, "frame id: %03d", cnt_frame);
    // sprintf(str, "frame id: %03dth\ntime: %.1fs", cnt_frame, cnt_frame/30.0);
    double r = 1, g = 1, b = 1;
    string text = str, str_id = "text display";
    int viewport = 0, fontsize = 20;
    if (cnt_frame == 0)
        viewer_->addText(text, xpos, ypos, fontsize, r, g, b, str_id, viewport);
    else
        viewer_->updateText(text, xpos, ypos, str_id);
    cnt_frame++;
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