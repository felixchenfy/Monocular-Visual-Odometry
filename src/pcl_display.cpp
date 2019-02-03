
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
const double LEN_COORD_AXIS = 0.1;
const double LEN_COORD_AXIS_TRUTH_TRAJ = 0.05;

// -- Point clouds to display

// 1. camera truth trajectory
string pc_cam_traj = "pc_cam_traj";
const unsigned char pc_cam_traj_color[3] = {255, 255, 255};

// 2. camera ground truth trajectory
string pc_cam_traj_ground_truth = "pc_cam_traj_ground_truth";
const unsigned char pc_cam_traj_ground_truth_color[3] = {0, 255, 0};

// 3. map points
string pc_pts_map = "pc_pts_map";

// 4. newly triangulated points by current frame
string pc_pts_curr = "pc_pts_curr";

// Store the above into a vector and a unordered_map
vector<string> point_clouds_names = {pc_cam_traj, pc_cam_traj_ground_truth, pc_pts_map, pc_pts_curr};
unordered_map<string, CloudPtr> point_clouds;

// -- Functions

// Set cloud points as the vector of pos and color
void setCloudPoints(CloudPtr cloud, const vector<cv::Point3f> &vec_pos, const vector<vector<unsigned char>> &vec_color);

// Set keyboard event
bool bKeyPressed = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void)
{
    pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr *>(viewer_void);
    if (event.keyDown())
    //   if (event.getKeySym () == "r" && event.keyDown ())
    {
        bKeyPressed = true;
    }
}

} // namespace my_display_private

// --------------------- Class definition -----------------------

namespace my_display
{
using namespace my_display_private;

// constructor
PclViewer::PclViewer(double x, double y, double z,
                     double rot_axis_x, double rot_axis_y, double rot_axis_z)
{
    // Set names
    viewer_name_ = "Trajectory{WHITE: estimated, RED: Keyframe; GREEN: truth}, Points{RED: new points} ";
    camera_frame_name_ = "camera_frame_name_";
    truth_camera_frame_name_ = "truth_camera_frame_name_";

    // Set viewer
    viewer_ = initPointCloudViewer(viewer_name_);
    viewer_->addCoordinateSystem(LEN_COORD_AXIS, "fixed world frame");
    viewer_->addCoordinateSystem(LEN_COORD_AXIS, camera_frame_name_);
    viewer_->addCoordinateSystem(LEN_COORD_AXIS_TRUTH_TRAJ, truth_camera_frame_name_);

    // Add point clouds to viewer, and stored the cloud ptr into the hash. (Last param is point size.)
    point_clouds[pc_cam_traj] = addPointCloud(viewer_, pc_cam_traj, 7);
    point_clouds[pc_cam_traj_ground_truth] = addPointCloud(viewer_, pc_cam_traj_ground_truth, 5);
    point_clouds[pc_pts_map] = addPointCloud(viewer_, pc_pts_map, 5);
    point_clouds[pc_pts_curr] = addPointCloud(viewer_, pc_pts_curr, 5);

    // Set viewer angle
    setViewerPose(*viewer_, x, y, z, rot_axis_x, rot_axis_y, rot_axis_z);

    // Set keyboard event
    viewer_->registerKeyboardCallback(keyboardEventOccurred, (void *)&viewer_);

    // Camera pose
    cam_R_vec_ = cv::Mat::eye(3, 3, CV_64F);
    cam_t_ = cv::Mat::zeros(3, 1, CV_64F);
    truth_cam_R_vec_ = cv::Mat::eye(3, 3, CV_64F);
    truth_cam_t_ = cv::Mat::zeros(3, 1, CV_64F);

}

bool PclViewer::checkKeyPressed()
{
    if (bKeyPressed)
    {
        bKeyPressed = false;
        return true;
    }
    else
    {
        return false;
    }
}

// -- Update camera pose ---------------------------------------------------------------------
void addNewCameraPoseToTraj(const cv::Mat &R_vec_new, const cv::Mat &t_new,
                            ViewerPtr viewer, cv::Mat &R_vec_curr, cv::Mat &t_curr,
                            const string cam_traj_name, const unsigned char color[3], 
                            int cnt_cam, int line_width, double line_color[3])
{
    // Add a line between new_pos and old_pos. Also, add a point.
    if (cnt_cam > 0)
    {
        // add a line between current and old camera pos
        pcl::PointXYZ cam_pos_old, cam_pos_new;
        setPointPos(cam_pos_old, t_curr);
        setPointPos(cam_pos_new, t_new);
        string line_name = cam_traj_name + "-line" + to_string(cnt_cam);
        viewer_->addLine<pcl::PointXYZ>(
            cam_pos_old, cam_pos_new, line_color[0], line_color[1], line_color[2], line_name);
        viewer_->setShapeRenderingProperties (
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, line_name);
        
        // add a point of the new camera pos
        pcl::PointXYZRGB point;
        setPointPos(point, t_new);
        setPointColor(point, color[0], color[1], color[2]);
        point_clouds[cam_traj_name]->points.push_back(point);
    }
    // Update new camera pose
    R_vec_curr = R_vec_new.clone();
    t_curr = t_new.clone();
}
void copyColor(const unsigned char c1[3], double c2[3]){
    for(int i=0;i<3;i++)
        c2[i]=c1[i]/255.0;    
}
void PclViewer::updateCameraPose(const cv::Mat &R_vec, const cv::Mat &t, int is_keyframe)
{
    static int cnt_cam = 0;
    const int line_width = 3;
    double line_color[3];
    unsigned char point_color[3]={pc_cam_traj_color[0], pc_cam_traj_color[1], pc_cam_traj_color[2]};
    if(is_keyframe==true){
        point_color[0]=255;
        point_color[1]=0;
        point_color[2]=0;
    }
    copyColor(pc_cam_traj_color, line_color);
    addNewCameraPoseToTraj(
        R_vec, t, viewer_, cam_R_vec_, cam_t_,
        pc_cam_traj, point_color, cnt_cam++, line_width, line_color);
}
void PclViewer::updateCameraTruthPose(const cv::Mat &R_vec, const cv::Mat &t)
{
    static int cnt_cam = 0;
    const int line_width = 1;
    double line_color[3]; 
    copyColor(pc_cam_traj_ground_truth_color, line_color);
    addNewCameraPoseToTraj(
        R_vec, t, viewer_, truth_cam_R_vec_, truth_cam_t_,
        pc_cam_traj_ground_truth, pc_cam_traj_ground_truth_color, cnt_cam++, line_width, line_color);
}

// -- Insert points ---------------------------------------------------------------------

void setCloudPoints(CloudPtr cloud, const vector<cv::Point3f> &vec_pos, const vector<vector<unsigned char>> &vec_color)
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
    setCloudPoints(cloud, vec_pos, vec_color);
}
void PclViewer::updateCurrPoints(const vector<cv::Point3f> &vec_pos, const vector<vector<unsigned char>> &vec_color)
{
    CloudPtr cloud = point_clouds[pc_pts_curr];
    setCloudPoints(cloud, vec_pos, vec_color);
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
    Eigen::Affine3f T_affine = my_basics::transT_CVRt_to_EigenAffine3d(cam_R_vec_, cam_t_).cast<float>();
    viewer_->removeCoordinateSystem(camera_frame_name_);
    viewer_->addCoordinateSystem(LEN_COORD_AXIS, T_affine, camera_frame_name_, 0);

    // Update truth camera
    Eigen::Affine3f T_affine_truth = my_basics::transT_CVRt_to_EigenAffine3d(truth_cam_R_vec_, truth_cam_t_).cast<float>();
    viewer_->removeCoordinateSystem(truth_camera_frame_name_);
    viewer_->addCoordinateSystem(LEN_COORD_AXIS_TRUTH_TRAJ, T_affine_truth, truth_camera_frame_name_, 0);

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