

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
#include "my_slam/common_include.h"

#include "my_slam/vo/vo_io.h"
#include "my_slam/basics/config.h"
#include "my_slam/basics/yaml.h"
#include "my_slam/basics/basics.h"

#include "my_slam/geometry/motion_estimation.h"
#include "my_slam/vo/frame.h"
#include "my_slam/vo/vo.h"

// display
#include "my_slam/display/pcl_display.h"

using namespace my_slam;

// =========================================
// =============== Functions ===============
// =========================================

bool checkInputArguments(int argc, char **argv);

const string IMAGE_WINDOW_NAME = "Green: keypoints; Red: inlier matches with map points";
bool drawResultByOpenCV(const cv::Mat &rgb_img, const vo::Frame::Ptr frame, const vo::VisualOdometry::Ptr vo);

display::PclViewer::Ptr setUpPclDisplay();
bool drawResultByPcl(basics::Yaml config_dataset,
                     const vo::VisualOdometry::Ptr vo,
                     vo::Frame::Ptr frame,
                     display::PclViewer::Ptr pcl_displayer);
void waitPclKeyPress(display::PclViewer::Ptr pcl_displayer);

// ========================================
// =============== Settings ===============
// ========================================

const bool IS_DEBUGGING = false;

// ========================================
// ================= Main =================
// ========================================

int main(int argc, char **argv)
{
    // -- Set configuration file
    assert(checkInputArguments(argc, argv));
    const string kConfigFile = argv[1];
    basics::Yaml config(kConfigFile);              // Use Yaml to read .yaml
    basics::Config::setParameterFile(kConfigFile); // Use Config to read .yaml
    const string dataset_name = config.get<string>("dataset_name");
    basics::Yaml config_dataset = config.get(dataset_name);

    // -- Read image filenames
    vector<string> image_paths;
    if (IS_DEBUGGING) // Read certain images specified below.
    {
        string folder = "/home/feiyu/Documents/Projects/EECS432_CV_VO/data/test_data/";
        vector<string> tmp{
            "image0001.jpg", "image0013.jpg", "image0015.jpg"};
        for (string &filename : tmp)
            filename = folder + filename;
        image_paths = tmp;
    }
    else // Read the dataset configured in config.yaml file
    {
        const string dataset_dir = config_dataset.get<string>("dataset_dir");
        const int num_images = config_dataset.get<int>("num_images");
        constexpr bool is_print_res = false;
        const string image_formatting = "/rgb_%05d.png";
        image_paths = vo::readImagePaths(dataset_dir, num_images, image_formatting, is_print_res);
    }

    // -- Read camera prameters.
    cv::Mat K = vo::readCameraIntrinsics(config_dataset); // camera intrinsics
    // Init a camera class to store K, and might be used to provide common transformations
    geometry::Camera::Ptr camera(new geometry::Camera(K));

    // -- Prepare PCL and CV display
    display::PclViewer::Ptr pcl_displayer = setUpPclDisplay(); // PCL display
    cv::namedWindow(IMAGE_WINDOW_NAME, cv::WINDOW_AUTOSIZE);   // CV display
    cv::moveWindow(IMAGE_WINDOW_NAME, 500, 50);

    // -- Setup for vo
    vo::VisualOdometry::Ptr vo(new vo::VisualOdometry);

    // -- Main loop: Iterate through images
    int max_num_imgs_to_proc = basics::Config::get<int>("max_num_imgs_to_proc");
    vector<cv::Mat> cam_pose_history;
    for (int img_id = 0; img_id < std::min(max_num_imgs_to_proc, (int)image_paths.size()); img_id++)
    {

        // Read image.
        cv::Mat rgb_img = cv::imread(image_paths[img_id]);
        if (rgb_img.data == nullptr)
        {
            cout << "The image file " << image_paths[img_id] << " is empty. Finished." << endl;
            break;
        }

        // run vo
        vo::Frame::Ptr frame = vo::Frame::createFrame(rgb_img, camera);
        vo->addFrame(frame); // This is the core of my VO !!!

        // Display
        bool cv2_draw_good = drawResultByOpenCV(rgb_img, frame, vo);
        bool pcl_draw_good = drawResultByPcl(config_dataset, vo, frame, pcl_displayer);
        static const bool is_pcl_wait_for_keypress = basics::Config::getBool("is_pcl_wait_for_keypress");
        if (is_pcl_wait_for_keypress)
            waitPclKeyPress(pcl_displayer);

        // if (img_id == 1){ // wait for user's keypress to start
        //     cv::imshow(IMAGE_WINDOW_NAME, rgb_img);
        //     waitKey(100);
        //     waitPclKeyPress(pcl_displayer);
        // }

        // Return
        // cout << "Finished an image" << endl;
        cam_pose_history.push_back(frame->T_w_c_.clone());
        frame->clearNoUsed();
        // if (img_id == 10+3)
        //     break;
    }

    // Save camera trajectory
    const string save_predicted_traj_to = basics::Config::get<string>("save_predicted_traj_to");
    vo::writePoseToFile(save_predicted_traj_to, cam_pose_history);

    // Wait for user close
    while (!pcl_displayer->isStopped())
        pcl_displayer->spinOnce(10);
    cv::destroyAllWindows();
}

// ===========================================================
// ================= Definition of functions =================
// ===========================================================

bool checkInputArguments(int argc, char **argv)
{
    // The only argument is Path to the configuration file, which stores the dataset_dir and camera_info
    const int kNumArguments = 1;
    if (argc - 1 != kNumArguments)
    {
        cout << "Lack arguments: Please input the path to the .yaml config file" << endl;
        return false;
    }
    return true;
}

display::PclViewer::Ptr setUpPclDisplay()
{
    double view_point_dist = 0.3;
    double x = 0.5 * view_point_dist,
           y = -1.0 * view_point_dist,
           z = -1.0 * view_point_dist;
    double rot_axis_x = -0.5, rot_axis_y = 0, rot_axis_z = 0;
    display::PclViewer::Ptr pcl_displayer(
        new display::PclViewer(x, y, z, rot_axis_x, rot_axis_y, rot_axis_z));
    return pcl_displayer;
}

bool drawResultByOpenCV(const cv::Mat &rgb_img, const vo::Frame::Ptr frame, const vo::VisualOdometry::Ptr vo)
{
    cv::Mat img_show = rgb_img.clone();
    const int img_id = frame->id_;
    static bool is_vo_initialized_in_prev_frame = false;
    bool first_time_vo_init = vo->isInitialized() && !is_vo_initialized_in_prev_frame;

    if (1) // draw all & inlier keypoints in the current frame
    {
        cv::Scalar color_g(0, 255, 0), color_b(255, 0, 0), color_r(0, 0, 255);
        vector<cv::KeyPoint> inliers_kpt;
        if (!vo->isInitialized() || first_time_vo_init)
        {
            for (auto &m : frame->matches_with_ref_)
                inliers_kpt.push_back(frame->keypoints_[m.trainIdx]);
        }
        else
        {
            for (auto &m : frame->matches_with_map_)
                inliers_kpt.push_back(frame->keypoints_[m.trainIdx]);
        }
        cv::drawKeypoints(img_show, frame->keypoints_, img_show, color_g);
        cv::drawKeypoints(img_show, inliers_kpt, img_show, color_r);
    }
    else if (img_id != 0 && // draw matches during initialization stage
             (!vo->isInitialized() || first_time_vo_init))
    {
        drawMatches(vo->ref_->rgb_img_, vo->ref_->keypoints_, // keywords: feature matching / matched features
                    frame->rgb_img_, frame->keypoints_,
                    frame->inliers_matches_for_3d_,
                    img_show);
    }
    is_vo_initialized_in_prev_frame = vo->isInitialized();
    cv::imshow(IMAGE_WINDOW_NAME, img_show);
    cv::waitKey(1);

    // save to file
    if (0)
    {
        string str_img_id = basics::int2str(img_id, 4);
        imwrite("result/" + str_img_id + ".png", img_show);
    }

    return true;
}

bool drawResultByPcl(basics::Yaml config_dataset,
                     const vo::VisualOdometry::Ptr vo,
                     vo::Frame::Ptr frame,
                     display::PclViewer::Ptr pcl_displayer)
{

    // -- Update camera pose
    cv::Mat R, R_vec, t;
    basics::getRtFromT(frame->T_w_c_, R, t);
    Rodrigues(R, R_vec);
    pcl_displayer->updateCameraPose(R_vec, t,
                                    vo->map_->hasKeyFrame(frame->id_)); // If it's keyframe, draw a red dot. Otherwise, white dot.

    // -- Update truth camera pose
    static const bool is_draw_true_traj = config_dataset.getBool("is_draw_true_traj");
    if (is_draw_true_traj)
    {
        static const string true_traj_filename = config_dataset.get<string>("true_traj_filename");
        static const vector<cv::Mat> truth_poses = vo::readPoseFromFile(true_traj_filename);
        cv::Mat truth_T = truth_poses[frame->id_], truth_R_vec, truth_t;
        basics::getRtFromT(truth_T, truth_R_vec, truth_t);

        // Start drawing only when visual odometry has been initialized. (i.e. The first few frames are not drawn.)
        // The reason is: we need scale the truth pose to be same as estiamted pose, so we can make comparison between them.
        // (And the underlying reason is that Mono SLAM cannot estiamte depth of point.)
        static bool is_made_the_same_scale = false;
        static double scale;
        if (is_made_the_same_scale == false && vo->isInitialized())
        {
            scale = basics::calcMatNorm(truth_t) / basics::calcMatNorm(t);
            is_made_the_same_scale = true;
        }
        if (vo->isInitialized())
        {
            truth_t /= scale;
            pcl_displayer->updateCameraTruthPose(truth_R_vec, truth_t);
        }
        else
        {
            // not draw truth camera pose
        }
    }
    // ------------------------------- Update points ----------------------------------------

    vector<cv::Point3f> vec_pos;
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
            const vo::MapPoint::Ptr &p = iter_map_point.second;
            vec_pos.push_back(p->pos_);
            vec_color.push_back(p->color_);
        }
        pcl_displayer->updateMapPoints(vec_pos, vec_color);
    }
    if (1 && vo->map_->hasKeyFrame(frame->id_) == true)
    {
        // --  If frame is a keyframe, Draw newly triangulated points with color
        // cout << "number of current triangulated points:"<<frame->inliers_pts3d_.size()<<endl;

        vec_pos.clear();
        vec_color.clear();
        color[0] = 255;
        color[1] = 0;
        color[2] = 0;
        for (const cv::Point3f &pt3d : frame->inliers_pts3d_)
        {
            vec_pos.push_back(basics::transCoord(pt3d, R, t));
            vec_color.push_back(color);
        }
        pcl_displayer->updateCurrPoints(vec_pos, vec_color);
    }

    // -----------------------------------------------------------------------
    // -- Display
    pcl_displayer->update();
    pcl_displayer->spinOnce(10);
    if (pcl_displayer->isStopped())
        return false;
    else
        return true;
}

void waitPclKeyPress(display::PclViewer::Ptr pcl_displayer)
{
    while (!pcl_displayer->isKeyPressed())
    {
        pcl_displayer->spinOnce(10);
    }
}
