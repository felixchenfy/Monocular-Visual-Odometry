

// std
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <deque>
#include <sstream>
#include <iomanip>

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// my
#include "my_basics/io.h"
#include "my_basics/config.h"
// #include "my_geometry/feature_match.h"
// #include "my_geometry/epipolar_geometry.h"
#include "my_geometry/motion_estimation.h"
#include "my_slam/frame.h"

// display
#include "my_display/pcl_viewer.h"
using namespace my_display;
const bool ENABLE_PCL_DISPLAY = true;

using namespace std;
using namespace cv;
using namespace my_geometry;

bool checkInputArguments(int argc, char **argv);
int main(int argc, char **argv)
{
    // Read in image filenames and camera prameters.
    assert(checkInputArguments(argc, argv));
    const string config_file = argv[1];
    bool print_res = false;
    vector<string> image_paths = my_basics::readImagePaths(config_file, 150, print_res);
    cv::Mat K = my_basics::readCameraIntrinsics(config_file); // camera intrinsics
    my_geometry::Camera::Ptr camera(                          // init a camera class with common transformations
        new my_geometry::Camera(K));
    my_basics::Config::setParameterFile( // just to remind to set this config file.
        config_file);                    // Following algorithms will read from it for setting params.

    // Prepare pcl display
    double view_point_dist = 3;
    double x = 0.5 * view_point_dist,
           y = -1.0 * view_point_dist,
           z = -1.0 * view_point_dist;
    double ea_x = -0.5, ea_y = 0, ea_z = 0;
    string viewer_name = "my pcl viewer";
    my_display::PclViewer::Ptr pcl_displayer(
        new my_display::PclViewer(
            viewer_name, x, y, z, ea_x, ea_y, ea_z));
    
    // prepare opencv display
    const string IMAGE_WINDOW_NAME="scene";
    cv::namedWindow(IMAGE_WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    cv::moveWindow(IMAGE_WINDOW_NAME, 500, 50);

    // Setup for vo
    deque<my_slam::Frame::Ptr> frames;
    enum VO_STATE
    {
        BLANK,
        INITIALIZATION,
        OK
        // LOST
    };
    VO_STATE vo_state = INITIALIZATION;
    const int FRAME_FOR_FIRST_ESSENTIAL=14;
    const int LENGHTH_UNIT=20; // num units per meter. (10 for dm, 100 for cm, 1000 for mmm)

    // Iterate through images
    for (int img_id = 0; img_id < (int)image_paths.size(); img_id++)
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
        for (int vo_null_loop = 0; vo_null_loop == 0; vo_null_loop++)
        {
            printf("\n\n=============================================\n");
            printf("=============================================\n");
            printf("=============================================\n");
            printf("Start processing the %dth image.\n", img_id);

            // Init a frame. Extract keypoints and descriptors.
            frame->extractKeyPoints();
            frame->computeDescriptors();
            cout << "Number of keypoints: " << frame->keypoints_.size() << endl;

            // vo_state: BLANK -> INITIALIZATION
            if (img_id == 0)
            {
                frame->T_w_c_ = Mat::eye(4, 4, CV_64F);
                frame->R_curr_to_prev_=Mat::eye(3, 3, CV_64F);
                frame->t_curr_to_prev_=Mat::zeros(3, 1, CV_64F);
                vo_state = INITIALIZATION;
                frames.push_back(frame);
                break;
            }

            // Match features
            my_slam::Frame::Ptr prev_frame = frames.back();
            frame->matchFeatures(prev_frame);

            // -- Estimation motion by Essential && Homography matrix and get inlier points
            vector<Mat> list_R, list_t, list_normal;
            vector<vector<DMatch>> list_matches; // these are the inliers matches
            vector<vector<Point3f>> sols_pts3d_in_cam1_by_triang;
            const bool print_res = false, is_frame_cam2_to_cam1 = true;
            const bool compute_homography = true;
            helperEstimatePossibleRelativePosesByEpipolarGeometry(
                /*Input*/
                prev_frame->keypoints_, frame->keypoints_, frame->matches_, K,
                /*Output*/
                list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
                /*settings*/
                print_res, compute_homography, is_frame_cam2_to_cam1);

            // -- Compute errors of results of E/H estimation:
            // [epipolar error] and [trigulation error on norm plane]
            // for the 3 solutions of (E, H1, H2)/
            // Choosing a good solution might based on these criterias.
            vector<double> list_error_epipolar;
            vector<double> list_error_triangulation;
            helperEvaluateEstimationsError(
                prev_frame->keypoints_, frame->keypoints_, list_matches,
                sols_pts3d_in_cam1_by_triang, list_R, list_t, list_normal, K,
                /*output*/
                list_error_epipolar, list_error_triangulation,
                true); // print result

            if (vo_state == INITIALIZATION)
            {
                // Check initialization condition
                {    
                    // Method 1: manually set an image id
                    const bool VO_INIT_FLAG_MANUAL=img_id >= FRAME_FOR_FIRST_ESSENTIAL;

                    // Method 2: check the distance between inlier points, run vo until there is a large movement.
                    vector<double> dists_between_kpts;
                    for (const DMatch &d:list_matches[0]){
                        Point2f p1=prev_frame->keypoints_[d.queryIdx].pt;
                        Point2f p2=frame->keypoints_[d.trainIdx].pt;
                        dists_between_kpts.push_back(calcDist(p1,p2));                        
                    }
                    double mean_dist=0;
                    for(double d:dists_between_kpts)mean_dist+=d;
                    mean_dist/=dists_between_kpts.size();
                    const bool VO_INIT_FLAG_KPT_DIST=mean_dist>50;

                    if (VO_INIT_FLAG_KPT_DIST){
                        cout<<"Large movement detected at frame "<<img_id<<". Start initialization"<<endl;
                    }else{ // skip this frame
                        frame->T_w_c_ =prev_frame->T_w_c_;
                        frame->R_curr_to_prev_=prev_frame->R_curr_to_prev_;
                        frame->t_curr_to_prev_=prev_frame->t_curr_to_prev_;
                        break;
                    }
                }

           

                // -- Choose 1 solution from the 3 solutions.
                //      Results: R, t, inlier_matches, pts_3d in cam1 and cam2
                // Currently, I'll just simply choose the result from essential matrix.
                //      Need to read papers such as ORB-SLAM2.
                // === analyzeError(list_error_epipolar, list_error_triangulation) ===
                const int SOL_IDX = 0; // 0 corresponds to Essential matrix
                vector<DMatch> &inlier_matches = list_matches[SOL_IDX];
                Mat &R = list_R[SOL_IDX], &t = list_t[SOL_IDX];
                vector<Point3f> &pts3d_in_cam1 = sols_pts3d_in_cam1_by_triang[SOL_IDX];
                vector<Point3f> pts3d_in_cam2;
                for (const Point3f &p1 : pts3d_in_cam1)
                    pts3d_in_cam2.push_back(transCoord(p1, R, t));
                const int num_inlier_pts = pts3d_in_cam2.size();

                // -- Normalize the mean depth of points to be 1m
                double mean_depth = 0;
                for (const Point3f &p : pts3d_in_cam2)
                    mean_depth += p.z;
                mean_depth /= num_inlier_pts;
                mean_depth = mean_depth / LENGHTH_UNIT;
                t /= mean_depth;
                for (Point3f &p : pts3d_in_cam2)
                {
                    p.x /= mean_depth;
                    p.y /= mean_depth;
                    p.z /= mean_depth;
                }

                // -- Update current camera pos
                Mat T_curr_to_prev = transRt2T(R, t);
                frame->T_w_c_ = prev_frame->T_w_c_ * T_curr_to_prev.inv();
                frame->R_curr_to_prev_ = R;
                frame->t_curr_to_prev_ = t;
                frame->inliers_matches_ = inlier_matches;
                frame->inliers_pts3d_ = pts3d_in_cam2;

                // --Update vo state
                vo_state = OK;
                frames.push_back(frame);
            }
            else if (vo_state == OK)
            {

                // -- Estimate Essential matrix to find the inliers
                // vector<DMatch> inliers_matches; // matches, that are inliers
                // Mat dummy_R, dummy_t;
                // helperEstiMotionByEssential(
                //     prev_frame->keypoints_, frame->keypoints_,
                //     frame->matches_, K,
                //     dummy_R, dummy_t, inliers_matches);
                // cout << "Number of inliers by E: "<<inliers_matches.size()<<endl;
                // frame->inliers_matches_ = inliers_matches;

                vector<DMatch> &inliers_matches=list_matches[0];
                frame->inliers_matches_ = inliers_matches;

                // --  Find the intersection between [DMatches_curr] and [DMatches_prev],
                // --  and 3d-2d correspondance
                vector<Point3f> pts_3d; // a point's 3d pos in cam1 frame
                vector<Point2f> pts_2d; // a point's 2d pos in image2 pixel frame
                helperFind3Dto2DCorrespondences(
                    frame->inliers_matches_,
                    frame->keypoints_,
                    prev_frame->inliers_matches_,
                    prev_frame->inliers_pts3d_,
                    pts_3d, pts_2d);
                
                // cout <<"DEBUG 3d-2d cor:"<<endl;
                // int cnt=0;
                // for(auto m:prev_frame->inliers_matches_)
                //     cout<<cnt++<<" prev frame: "<<m.trainIdx<<endl;
                // cnt=0;
                // for(auto m:frame->inliers_matches_)
                //     cout<<cnt++<<"curr frame: "<<m.queryIdx<<endl;
                cout << "Number of 3d-2d pairs: " << pts_3d.size() << endl;

                // -- Solve PnP, get T_cam1_to_cam2
                Mat R_vec, R, t;
                solvePnPRansac(pts_3d, pts_2d, K, Mat(), R_vec, t, false);
                Rodrigues(R_vec, R);

                // --Triangulate points
                vector<Point3f> pts_3d_in_curr;
                helperTriangulatePoints(
                    prev_frame->keypoints_, frame->keypoints_,
                    frame->inliers_matches_, R, t, K,
                    pts_3d_in_curr);
                frame->inliers_pts3d_ = pts_3d_in_curr;

                // -- Update current camera pos
                Mat T_curr_to_prev = transRt2T(R, t);
                frame->T_w_c_ = prev_frame->T_w_c_ * T_curr_to_prev.inv();
                frame->R_curr_to_prev_ = R;
                frame->t_curr_to_prev_ = t;
                
                
                //DEBUG
                invRt(R,t);
                cout<<"\nprev_to_curr displaycement: "<<t.t()<<endl;

                // --Update vo state
                vo_state = vo_state; // still OK
                frames.push_back(frame);
            }
        } // This is a dummy loop

        // ------------------------Complete-------------------------------
        if (vo_state==OK)
        { // Display image by opencv
            cv::Mat img_show = rgb_img.clone();
            std::stringstream ss;
            ss << std::setw(4) << std::setfill('0') << img_id;
            string str_img_id = ss.str();
            const int CV_WAIT_KEY_TIME=10;
            if (1 || img_id == 0) // draw keypoints
            {
                cv::Scalar color_g(0, 255, 0),color_b(255,0,0),color_r(0, 0, 255);
                // cv::Scalar color2 = cv::Scalar::all(-1);
                string window_name="img "+str_img_id;
                vector<KeyPoint> inliers_kpt;
                for(auto &m:frame->matches_)
                    inliers_kpt.push_back(frame->keypoints_[m.trainIdx]);
                cv::drawKeypoints(img_show, frame->keypoints_, img_show, color_g);
                cv::drawKeypoints(img_show, inliers_kpt, img_show, color_r);
                // cv::destroyAllWindows();
                // cv::imshow(window_name, img_show);
                cv::imshow(IMAGE_WINDOW_NAME, img_show);
                waitKey(CV_WAIT_KEY_TIME);
            }
            if(0 && img_id!=0) // draw matches
            {
                my_slam::Frame::Ptr prev_frame = frames[frames.size()-2];// the last 2nd frame
                string window_name = "Image " + str_img_id + ", matched keypoints";                
                drawMatches(prev_frame->rgb_img_, prev_frame->keypoints_,
                            frame->rgb_img_, frame->keypoints_, frame->matches_, img_show);
                cv::namedWindow(window_name, WINDOW_AUTOSIZE);
                cv::imshow(window_name, img_show);
                waitKey(CV_WAIT_KEY_TIME);
            }
            imwrite("result/" + str_img_id + ".png", img_show);
        }
        printf("\n\n--Printing frame %d pose:\n", img_id);
        if (1 && ENABLE_PCL_DISPLAY)
        {
            Mat R, R_vec, t;
            getRtFromT(frame->T_w_c_, R, t);
            Rodrigues(R, R_vec);

            // cout << endl;
            // cout << "R_world_to_camera:\n"
            //      << R << endl;
            // cout << "t_world_to_camera:\n"
            //      << t.t() << endl;

            pcl_displayer->updateCameraPose(R_vec, t);
            unsigned char r = 255, g = 0, b = 0;
            pcl_displayer->deletePoints();
            for(const Point3f &pt3d: frame->inliers_pts3d_){
                Mat kpt_3d_pos_in_world = R * Point3f_to_Mat(pt3d) + t;
                pcl_displayer->addPoint(kpt_3d_pos_in_world, r, g, b);
            }

            pcl_displayer->update();
            pcl_displayer->spinOnce(100);
            if (pcl_displayer->wasStopped())
                break;
        }
        // Save to buff.
        if (frames.size() > 10) frames.pop_front();

        // Print
        // cout << "R_curr_to_prev_: " << frame->R_curr_to_prev_ << endl;
        // cout << "t_curr_to_prev_: " << frame->t_curr_to_prev_ << endl;

        cout << "Finished an image" << endl;

        // Return
        if (img_id == FRAME_FOR_FIRST_ESSENTIAL+150)
            break;
    }
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