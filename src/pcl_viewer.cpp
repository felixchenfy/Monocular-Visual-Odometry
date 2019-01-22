
#include "my_display/pcl_viewer.h"
#include "my_display/pcl_visualizations.h"

using namespace std;
using namespace cv;
using namespace Eigen;

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtr;

namespace my_display_private
{ // store pcl related class members here instead of .h, in order to speed up compiling.

ViewerPtr viewer_;

CloudPtr pc_cam_traj_ptr;
string pc_cam_traj_name = "pc_cam_traj_name";

CloudPtr pc_pts_map_ptr;
string pc_pts_map_name = "pc_pts_map_name";

CloudPtr pc_pts_in_view_ptr;
string pc_pts_in_view_name = "pc_pts_in_view_name";

CloudPtr pc_pts_curr_ptr;
string pc_pts_curr_name = "pc_pts_curr_name";

// Store the above 4 into 1 array
const int NUM_PC = 4;
CloudPtr pc_array_ptr[NUM_PC] = {pc_cam_traj_ptr, pc_pts_map_ptr, pc_pts_in_view_ptr, pc_pts_curr_ptr};
string pc_array_name[NUM_PC] = {pc_cam_traj_name, pc_pts_map_name, pc_pts_in_view_name, pc_pts_curr_name};

} // namespace my_display_private

namespace my_display
{
using namespace my_display_private;

void addPointCloud(ViewerPtr &viewer, CloudPtr &cloud, string cloud_name, int POINT_SIZE = 3)
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_setting(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, color_setting, pc_cam_traj_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, pc_cam_traj_name);
}

// --------------------- Class definition -----------------------

// constructor
PclViewer::PclViewer(const string &viewer_name,
                     double x, double y, double z,
                     double rotaxis_x, double rotaxis_y, double rotaxis_z)
{
    // Set names
    viewer_name_ = viewer_name;
    camera_frame_name_ = "Coord: camera";

    // Set viewer
    pc_pts_curr_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewer_ = initPointCloudViewer(
        pc_pts_curr_ptr, viewer_name_, pc_pts_curr_name, camera_frame_name_);
    viewer_->addCoordinateSystem(1.0, "fixed world frame");

    // Add another point cloud for showing camera's trajectory
    for (int i = 0; i < NUM_PC; i++)
    {
        addPointCloud(viewer_, pc_array_ptr[i], pc_array_name[i]);
    }

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
        pc_cam_traj_ptr->points.push_back(point);
    }
    cam_R_vec_ = R_vec.clone();
    cam_t_ = t.clone();
}

// -- Insert points ---------------------------------------------------------------------

void addPoint(CloudPtr &cloud, const cv::Mat pt_3d_pos_in_world, uchar r, uchar g, uchar b)
{
    pcl::PointXYZRGB point;
    setPointPos(point, pt_3d_pos_in_world);
    setPointColor(point, r, g, b);
    pc_pts_curr_ptr->points.push_back(point);
}
void deletePoints(CloudPtr &cloud)
{
    cloud->points.clear();
}

// void PclViewer::updatePtsMap();
// void PclViewer::updatePtsInView();
// void PclViewer::updatePtsCurr();

// -- Update ---------------------------------------------------------------------

void PclViewer::update()
{
    static int cnt_frame = 0;
    // Update camera
    Eigen::Affine3f T_affine = my_basics::transCVMatRt2Affine3d(cam_R_vec_, cam_t_).cast<float>();
    viewer_->removeCoordinateSystem(camera_frame_name_);
    viewer_->addCoordinateSystem(1.0, T_affine, camera_frame_name_, 0);

    // Update point
    for (int i = 0; i < NUM_PC; i++)
    {
        viewer_->updatePointCloud(pc_array_ptr[i], pc_array_name[i]);
    }

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