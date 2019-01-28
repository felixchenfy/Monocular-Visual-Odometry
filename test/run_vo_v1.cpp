

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

const string IMAGE_WINDOW_NAME = "window name";
bool drawResultByOpenCV(const cv::Mat &rgb_img, const my_slam::Frame::Ptr frame, const my_slam::VisualOdometry::Ptr vo);

PclViewer::Ptr setUpPclDisplay();
bool drawResultByPcl(const my_slam::VisualOdometry::Ptr vo, my_slam::Frame::Ptr frame, PclViewer::Ptr pcl_displayer);
void holdOnPclViewer(PclViewer::Ptr pcl_displayer);

int main(int argc, char **argv)
{
    // -- Read in image filenames and camera prameters.
    assert(checkInputArguments(argc, argv));
    const string CONFIG_FILE = argv[1];
    const bool PRINT_RES = false;

    vector<string> image_paths = my_basics::readImagePaths(CONFIG_FILE, 150, PRINT_RES);

    cv::Mat K = my_basics::readCameraIntrinsics(CONFIG_FILE); // camera intrinsics

    // Init a camera class to store K, and might be used to provide common transformations
    my_geometry::Camera::Ptr camera(new my_geometry::Camera(K));

    // Just to remind to set this config file. Following algorithms will read from it for setting params.
    my_basics::Config::setParameterFile(CONFIG_FILE);

    // -- Prepare Pcl display
    PclViewer::Ptr pcl_displayer = setUpPclDisplay(); // Prepare pcl display

    // -- Prepare opencv display
    cv::namedWindow(IMAGE_WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(IMAGE_WINDOW_NAME, 500, 50);

    // -- Setup for vo
    my_slam::VisualOdometry::Ptr vo(new my_slam::VisualOdometry);

    // -- Iterate through images
    int max_num_images = my_basics::Config::get<int>("max_num_images");
    for (int img_id = 0; img_id < min(max_num_images, (int)image_paths.size()); img_id++)
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
        bool pcl_draw_good = drawResultByPcl(vo, frame, pcl_displayer);

        // Return
        cout << "Finished an image" << endl;
        if (img_id == 150)
            break;
        if (vo->DEBUG_STOP_PROGRAM_)
        {
            break;
        }
    }
    holdOnPclViewer(pcl_displayer);
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
    double view_point_dist = 3;
    double x = 0.5 * view_point_dist,
           y = -1.0 * view_point_dist,
           z = -1.0 * view_point_dist;
    double rotaxis_x = -0.5, rotaxis_y = 0, rotaxis_z = 0;
    PclViewer::Ptr pcl_displayer(
        new PclViewer(x, y, z, rotaxis_x, rotaxis_y, rotaxis_z));
    return pcl_displayer;
}

bool drawResultByOpenCV(const cv::Mat &rgb_img,
                        const my_slam::Frame::Ptr frame, const my_slam::VisualOdometry::Ptr vo)
{
    cv::Mat img_show = rgb_img.clone();
    const int img_id = frame->id_;
    const int CV_WAIT_KEY_TIME = 10;
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
        my_slam::Frame::Ptr prev_frame = vo->keyframes_[vo->keyframes_.size() - 2]; // the last 2nd frame
        drawMatches(prev_frame->rgb_img_, prev_frame->keypoints_,
                    frame->rgb_img_, frame->keypoints_, frame->matches_, img_show);
    }
    cv::imshow(IMAGE_WINDOW_NAME, img_show);
    waitKey(CV_WAIT_KEY_TIME);

    // save to file
    string str_img_id = my_basics::int2str(img_id, 4);
    imwrite("result/" + str_img_id + ".png", img_show);

    return true;
}

bool drawResultByPcl(const my_slam::VisualOdometry::Ptr vo, my_slam::Frame::Ptr frame, PclViewer::Ptr pcl_displayer)
{

    // -- Update camera pose
    Mat R, R_vec, t;
    getRtFromT(frame->T_w_c_, R, t);
    Rodrigues(R, R_vec);
    pcl_displayer->updateCameraPose(R_vec, t);

    // ------------------------------- Update points ----------------------------------------
    vector<Point3f> vec_pos;
    vector<vector<unsigned char>> vec_color;
    unsigned char r, g, b;
    vector<unsigned char> color(3, 0);

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

    // -- Draw newly inserted points with specified color
    vec_pos.clear();
    vec_color.clear();
    color[0] = 255;
    color[1] = 0;
    color[2] = 0;
    // cout << "number of current triangulated points:"<<frame->inliers_pts3d_.size()<<endl;
    for (const Point3f &pt3d : frame->inliers_pts3d_)
    {
        vec_pos.push_back(transCoord(pt3d, R, t));
        vec_color.push_back(color);
    }
    pcl_displayer->updateCurrPoints(vec_pos, vec_color);

    // -----------------------------------------------------------------------
    // -- Update and display
    pcl_displayer->update();
    pcl_displayer->spinOnce(100);
    if (pcl_displayer->wasStopped())
        return false;
    else
        return true;
}
void holdOnPclViewer(PclViewer::Ptr pcl_displayer)
{

    while (!pcl_displayer->wasStopped())
    {
        // pcl_displayer->update();
        pcl_displayer->spinOnce(10);
    }
}