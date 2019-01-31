

// std
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <deque>
#include <sstream>
#include <iomanip>
#include <unistd.h>

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// my
#include "my_basics/io.h"
#include "my_basics/config.h"
#include "my_basics/basics.h"

#include "my_geometry/motion_estimation.h"
#include "my_slam/frame.h"
#include "my_slam/vo.h"

// display
#include "my_display/pcl_display.h"
using namespace my_display;

using namespace std;
using namespace cv;
using namespace my_geometry;
using namespace my_slam;

// functions for this script
bool checkInputArguments(int argc, char **argv);

const string IMAGE_WINDOW_NAME = "Green: keypoints; Red: inlier matches with map points";
bool drawResultByOpenCV(const cv::Mat &rgb_img, const my_slam::Frame::Ptr frame, const my_slam::VisualOdometry::Ptr vo);

PclViewer::Ptr setUpPclDisplay();
bool drawResultByPcl(const my_slam::VisualOdometry::Ptr vo, my_slam::Frame::Ptr frame, 
    PclViewer::Ptr pcl_displayer, bool DRAW_GROUND_TRUTH_TRAJ);
void waitPclKeyPress(PclViewer::Ptr pcl_displayer);

const bool DEBUG_MODE = false;

int main(int argc, char **argv)
{
    // -- Read in image filenames and camera prameters.
    assert(checkInputArguments(argc, argv));
    const string CONFIG_FILE = argv[1];
    const bool PRINT_RES = false;
    vector<string> image_paths;
    if (DEBUG_MODE)
    {
        string folder = "/home/feiyu/Desktop/slam/my_vo/my2/data/test_data/";
        vector<string> tmp{
            "image0001.jpg", "image0013.jpg", "image0015.jpg"};
        for (string &filename : tmp)
            filename = folder + filename;
        image_paths = tmp;
    }
    else
    {
        image_paths = my_basics::readImagePaths(CONFIG_FILE, PRINT_RES);
    }
    cv::Mat K = my_basics::readCameraIntrinsics(CONFIG_FILE); // camera intrinsics

    // Init a camera class to store K, and might be used to provide common transformations
    my_geometry::Camera::Ptr camera(new my_geometry::Camera(K));

    // Just to remind to set this config file. Following algorithms will read from it for setting params.
    my_basics::Config::setParameterFile(CONFIG_FILE);

    // -- Prepare Pcl display
    PclViewer::Ptr pcl_displayer = setUpPclDisplay(); // Prepare pcl display
    bool DRAW_GROUND_TRUTH_TRAJ =  my_basics::Config::get<int>("DRAW_GROUND_TRUTH_TRAJ")==1;

    // -- Prepare opencv display
    cv::namedWindow(IMAGE_WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(IMAGE_WINDOW_NAME, 500, 50);

    // -- Setup for vo
    my_slam::VisualOdometry::Ptr vo(new my_slam::VisualOdometry);

    // -- Iterate through images
    int MAX_NUM_IMAGES = my_basics::Config::get<int>("MAX_NUM_IMAGES");
    vector<cv::Mat> cam_pose_history;
    for (int img_id = 0; img_id < min(MAX_NUM_IMAGES, (int)image_paths.size()); img_id++)
    {

        // Read image.
        cv::Mat rgb_img = cv::imread(image_paths[img_id]);
        if (rgb_img.data == nullptr)
        {
            cout << "The image file <<" << image_paths[img_id] << "<< is empty. Finished." << endl;
            break;
        }

        // run vo
        my_slam::Frame::Ptr frame = my_slam::Frame::createFrame(rgb_img, camera);
        vo->addFrame(frame);

        // Display
        bool cv2_draw_good = drawResultByOpenCV(rgb_img, frame, vo);
        bool pcl_draw_good = drawResultByPcl(vo, frame, pcl_displayer, DRAW_GROUND_TRUTH_TRAJ);
        static const int PCL_WAIT_FOR_KEY_PRESS = my_basics::Config::get<int>("PCL_WAIT_FOR_KEY_PRESS");
        if (PCL_WAIT_FOR_KEY_PRESS == 1)
            waitPclKeyPress(pcl_displayer);

        // Return
        // cout << "Finished an image" << endl;
        cam_pose_history.push_back(vo->curr_->T_w_c_.clone());
        // if (img_id == 100 || vo->vo_state_ == VisualOdometry::OK)
        //     break;
    }
   
    // Save camera trajectory
    const string STORE_CAM_TRAJ= my_basics::Config::get<string>("STORE_CAM_TRAJ");
    writePoseToFile(STORE_CAM_TRAJ, cam_pose_history);
    
    // Wait for user close
    while (!pcl_displayer->wasStopped())
        pcl_displayer->spinOnce(10);
    cv::destroyAllWindows();
}

bool checkInputArguments(int argc, char **argv)
{
    // The only argument is Path to the configuration file, which stores the dataset_dir and camera_info
    const int NUM_ARGUMENTS = 1;
    if (argc - 1 != NUM_ARGUMENTS)
    {
        cout << "Lack arguments: Please input the path to the .yaml config file" << endl;
        return false;
    }
    return true;
}

PclViewer::Ptr setUpPclDisplay()
{
    double view_point_dist = 0.3;
    double x = 0.5 * view_point_dist,
           y = -1.0 * view_point_dist,
           z = -1.0 * view_point_dist;
    double rot_axis_x = -0.5, rot_axis_y = 0, rot_axis_z = 0;
    PclViewer::Ptr pcl_displayer(
        new PclViewer(x, y, z, rot_axis_x, rot_axis_y, rot_axis_z));
    return pcl_displayer;
}

bool drawResultByOpenCV(const cv::Mat &rgb_img, const my_slam::Frame::Ptr frame, const my_slam::VisualOdometry::Ptr vo)
{
    cv::Mat img_show = rgb_img.clone();
    const int img_id = frame->id_;
    if (1) // draw keypoints
    {
        cv::Scalar color_g(0, 255, 0), color_b(255, 0, 0), color_r(0, 0, 255);
        vector<KeyPoint> inliers_kpt;
        for (auto &m : frame->matches_)
            inliers_kpt.push_back(frame->keypoints_[m.trainIdx]);
        cv::drawKeypoints(img_show, frame->keypoints_, img_show, color_g);
        cv::drawKeypoints(img_show, inliers_kpt, img_show, color_r);
    }
    else if (0 && img_id != 0) // draw matches
    {
        drawMatches(vo->ref_->rgb_img_, vo->ref_->keypoints_,
                    frame->rgb_img_, frame->keypoints_,
                    // frame->matches_,
                    frame->inlier_matches_,
                    img_show);
    }
    cv::imshow(IMAGE_WINDOW_NAME, img_show);
    waitKey(20);

    // save to file
    if (0)
    {
        string str_img_id = my_basics::int2str(img_id, 4);
        imwrite("result/" + str_img_id + ".png", img_show);
    }

    return true;
}

bool drawResultByPcl(const my_slam::VisualOdometry::Ptr vo, my_slam::Frame::Ptr frame, 
    PclViewer::Ptr pcl_displayer, bool DRAW_GROUND_TRUTH_TRAJ)
{

    // -- Update camera pose
    Mat R, R_vec, t;
    getRtFromT(frame->T_w_c_, R, t);
    Rodrigues(R, R_vec);
    pcl_displayer->updateCameraPose(R_vec, t);
    
    // -- Update truth camera pose
    if(DRAW_GROUND_TRUTH_TRAJ){
        static string GROUND_TRUTH_TRAJ_FILENAME =  my_basics::Config::get<string>("GROUND_TRUTH_TRAJ_FILENAME");
        static vector<cv::Mat> truth_poses= readPoseToFile(GROUND_TRUTH_TRAJ_FILENAME);
        cv::Mat truth_T = truth_poses[frame->id_], truth_R_vec, truth_t;
        getRtFromT(truth_T, truth_R_vec, truth_t);
        
        // Start drawing only when visual odometry has been initialized. (i.e. The first few frames are not drawn.)
        // The reason is: we need scale the truth pose to be same as estiamted pose, so we can make comparison between them. 
        // (And the underlying reason is that Mono SLAM cannot estiamte depth of point.)
        static bool is_made_the_same_scale=false;
        static double scale;
        if(is_made_the_same_scale==false && vo->isInitialized()){
            scale = calcMatNorm(truth_t)/calcMatNorm(t);
            is_made_the_same_scale = true;
        }
        if(vo->isInitialized()){
            truth_t/=scale;
            pcl_displayer->updateCameraTruthPose(truth_R_vec, truth_t);
        }else{
            // not draw truth camera pose
        }

    }
    // ------------------------------- Update points ----------------------------------------

    vector<Point3f> vec_pos;
    vector<vector<unsigned char>> vec_color;
    unsigned char r, g, b;
    vector<unsigned char> color(3, 0);

    if (1)
    {
        // -- Draw map points
        vec_pos.clear();
        vec_color.clear();
        for (auto &iter_map_point : vo->map_->map_points_)
        {
            const MapPoint::Ptr &p = iter_map_point.second;
            vec_pos.push_back(Mat_to_Point3f(p->pos_));
            vec_color.push_back(p->color_);
        }
        pcl_displayer->updateMapPoints(vec_pos, vec_color);
    }
    if (1 && vo->map_->checkKeyFrame(frame->id_) == true) 
    {
        // --  If frame is a keyframe, Draw newly triangulated points with color
        // cout << "number of current triangulated points:"<<frame->inliers_pts3d_.size()<<endl;

        vec_pos.clear();
        vec_color.clear();
        color[0] = 255;
        color[1] = 0;
        color[2] = 0;
        for (const Point3f &pt3d : frame->inliers_pts3d_)
        {
            vec_pos.push_back(transCoord(pt3d, R, t));
            vec_color.push_back(color);
        }
        pcl_displayer->updateCurrPoints(vec_pos, vec_color);
    }

    // -----------------------------------------------------------------------
    // -- Display
    pcl_displayer->update();
    pcl_displayer->spinOnce(10);
    if (pcl_displayer->wasStopped())
        return false;
    else
        return true;
}

void waitPclKeyPress(PclViewer::Ptr pcl_displayer)
{
    while (!pcl_displayer->checkKeyPressed())
    {
        pcl_displayer->spinOnce(10);
    }
}
