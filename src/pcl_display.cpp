
#include "my_display/pcl_display.h"
#include "my_display/pcl_display_lib.h"
#include <unordered_map>

using namespace std;
using namespace cv;
using namespace Eigen;

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPtr;

namespace my_display_private
{ // store pcl related class members here instead of .h, in order to speed up compiling.

ViewerPtr viewer_;
const double LEN_COORD_AXIS=0.1;

const int NUM_PC = 4;
string pc_cam_traj = "pc_cam_traj";
string pc_pts_map = "pc_pts_map";
string pc_pts_in_view = "pc_pts_in_view";
string pc_pts_curr = "pc_pts_curr";
vector<string> point_clouds_names = {pc_cam_traj, pc_pts_map, pc_pts_in_view, pc_pts_curr};
unordered_map<string, CloudPtr> point_clouds;

void resetPoints(CloudPtr cloud, const vector<cv::Point3f> &vec_pos, const vector<vector<unsigned char>> &vec_color);

// Set keyboard event
bool bKeyPressed=false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void)
{
  pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *> (viewer_void);
  if (event.keyDown())
//   if (event.getKeySym () == "r" && event.keyDown ())
  {
    bKeyPressed=true;
  }
}

} // namespace my_display_private

// --------------------- Class definition -----------------------

namespace my_display
{
using namespace my_display_private;

// constructor
PclViewer::PclViewer(double x, double y, double z,
                     double rotaxis_x, double rotaxis_y, double rotaxis_z)
{
    // Set names
    viewer_name_ = "viewer_name_";
    camera_frame_name_ = "camera_frame_name_";

    // Set viewer
    viewer_ = initPointCloudViewer(viewer_name_, camera_frame_name_, LEN_COORD_AXIS);
    viewer_->addCoordinateSystem(LEN_COORD_AXIS, "fixed world frame");

    // Add point clouds to viewer, and stored the cloud ptr into the hash
    point_clouds[pc_cam_traj] = addPointCloud(viewer_, pc_cam_traj, 3); // last param is point size
    point_clouds[pc_pts_map] = addPointCloud(viewer_, pc_pts_map, 4);
    point_clouds[pc_pts_in_view] = addPointCloud(viewer_, pc_pts_in_view, 8);
    point_clouds[pc_pts_curr] = addPointCloud(viewer_, pc_pts_curr, 20);

    // Set viewer angle
    setViewerPose(*viewer_, x, y, z, rotaxis_x, rotaxis_y, rotaxis_z);

    // Set keyboard event
    viewer_->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer_);
}

bool PclViewer::checkKeyPressed(){
    if(bKeyPressed){
        bKeyPressed=false;
        return true;
    }else{
        return false;
    }
}

// -- Update camera pose ---------------------------------------------------------------------
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
        point_clouds[pc_cam_traj]->points.push_back(point);
    }
    cam_R_vec_ = R_vec.clone();
    cam_t_ = t.clone();
}

// -- Insert points ---------------------------------------------------------------------

void resetPoints(CloudPtr cloud, const vector<cv::Point3f> &vec_pos, const vector<vector<unsigned char>> &vec_color)
{
    cloud->points.clear();
    assert(vec_pos.size() == vec_color.size());
    int N = vec_pos.size();
    pcl::PointXYZRGB point;
    for (int i = 0; i < N; i++)
    {
        point.x = vec_pos[i].x;
        point.y = vec_pos[i].y;
        point.z = vec_pos[i].z;
        point.r = vec_color[i][0];
        point.g = vec_color[i][1];
        point.b = vec_color[i][2];
        cloud->points.push_back(point);
    }
}

void PclViewer::updateMapPoints(const vector<cv::Point3f> &vec_pos, const vector<vector<unsigned char>> &vec_color)
{
    CloudPtr cloud = point_clouds[pc_pts_map];
    resetPoints(cloud, vec_pos, vec_color);
}
void PclViewer::updateCurrPoints(const vector<cv::Point3f> &vec_pos, const vector<vector<unsigned char>> &vec_color)
{
    CloudPtr cloud = point_clouds[pc_pts_curr];
    resetPoints(cloud, vec_pos, vec_color);
}
void PclViewer::updatePointsInView(const vector<cv::Point3f> &vec_pos, const vector<vector<unsigned char>> &vec_color)
{
    CloudPtr cloud = point_clouds[pc_pts_in_view];
    resetPoints(cloud, vec_pos, vec_color);
}

// // Add point to cloud by name. Not used
// void addPointToCloud(const string &cloud_name, const cv::Mat &pt_3d_pos_in_world, uchar r, uchar g, uchar b)
// {
//     pcl::PointXYZRGB point;
//     setPointPos(point, pt_3d_pos_in_world);
//     setPointColor(point, r, g, b);
//     assert(point_clouds.find(cloud_name)!=point_clouds.end());
//     point_clouds[cloud_name]->points.push_back(point);
// }
// void deletePointsOfCloud(const string &cloud_name)
// {
//     assert(point_clouds.find(cloud_name)!=point_clouds.end());
//     point_clouds[cloud_name]->points.clear();
// }

// -- Update ---------------------------------------------------------------------

void PclViewer::update()
{
    static int cnt_frame = 0;
    // Update camera
    Eigen::Affine3f T_affine = my_basics::transCVMatRt2Affine3d(cam_R_vec_, cam_t_).cast<float>();
    viewer_->removeCoordinateSystem(camera_frame_name_);
    viewer_->addCoordinateSystem(LEN_COORD_AXIS, T_affine, camera_frame_name_, 0);

    // Update point
    for (auto cloud_info : point_clouds)
    {
        string name = cloud_info.first;
        CloudPtr cloud = cloud_info.second;
        viewer_->updatePointCloud(cloud, name);
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