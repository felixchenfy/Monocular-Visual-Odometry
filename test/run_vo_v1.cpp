

// std
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <deque>

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
    vector<string> image_paths = my_basics::readImagePaths(config_file, print_res);
    cv::Mat K = my_basics::readCameraIntrinsics(config_file); // camera intrinsics
    my_geometry::Camera::Ptr camera(                          // init a camera class with common transformations
        new my_geometry::Camera(K));
    my_basics::Config::setParameterFile( // just to remind to set this config file.
        config_file);                    // Following algorithms will read from it for setting params.

    // read in image
    deque<my_slam::Frame::Ptr> frames;
    enum VO_STATE
    {
        INITIALIZATION,
        OK,
        LOST
    };
    VO_STATE vo_state = INITIALIZATION;
    // double mean_depth = 0; // This is the scale factor. Will be estiamted in 1st frame.
    for (int img_id = 0; img_id < (int)image_paths.size(); img_id++)
    {

        // Read image.
        cv::Mat rgb_img = cv::imread(image_paths[img_id]);
        if (rgb_img.data == nullptr)
            break;
        printf("\n\n=============================================\n");
        printf("=============================================\n");
        printf("=============================================\n");
        printf("Start processing the %dth image.\n", img_id);

        // Init a frame. Extract keypoints and descriptors.
        my_slam::Frame::Ptr frame = my_slam::Frame::createFrame(rgb_img, camera);
        frame->extractKeyPoints();
        frame->computeDescriptors();

        // Add a new frame into vo.
        vector<KeyPoint> keypoints;
        vector<DMatch> matches;
        Mat descriptors;
        if (img_id == 0)
        {
            Mat T_w_to_c_0 = Mat::eye(4, 4, CV_64F);
            frame->T_w_c_ = T_w_to_c_0;
            vo_state = INITIALIZATION;
        }
        else
        {
            my_slam::Frame::Ptr prev_frame = frames.back();
            frame->matchFeatures(prev_frame);
            if (vo_state == INITIALIZATION)
            { // initiliazation stage

                // -- Estimation motion
                vector<Mat> list_R, list_t, list_normal;
                vector<vector<DMatch>> list_matches; // these are the inliers matches
                vector<vector<Point3f>> sols_pts3d_in_cam1_by_triang;
                helperEstimatePossibleRelativePosesByEpipolarGeometry(
                    /*Input*/
                    prev_frame->keypoints_, frame->keypoints_,
                    prev_frame->descriptors_, frame->descriptors_,
                    frame->matches_,
                    K,
                    /*Output*/
                    list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
                    false); // print result
                cout << "DEBUG: Before PnP: test if list_R[1] is the same:" << list_R[1]<<endl;
                
                // -- Compute [epipolar error] and [trigulation error on norm plane] for the 3 solutions (E, H1, H2)
                vector<double> list_error_epipolar;
                vector<double> list_error_triangulation;
                helperEvaluateEstimationsError(
                    prev_frame->keypoints_, frame->keypoints_,list_matches,
                    sols_pts3d_in_cam1_by_triang, list_R, list_t, list_normal, K,
                    /*output*/
                    list_error_epipolar, list_error_triangulation,
                    false); // print result

                // -- Choose 1 solution from the 3 solutions.
                // Currently, I'll just simply choose the result from essential matrix.
                //      Need to read papers such as ORB-SLAM2.
                const int sol_idx = 0;
                Mat R = list_R[sol_idx], t = list_t[sol_idx];
                vector<Point3f> pts3d = sols_pts3d_in_cam1_by_triang[sol_idx];
                const int num_inlier_pts = pts3d.size();

                // -- Normalize the mean depth of points to be 1m
                double mean_depth = 0;
                for (const Point3f &p : pts3d)
                    mean_depth += p.z;
                mean_depth /= num_inlier_pts;
                t /= mean_depth;
                for (Point3f &p : pts3d)
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
                // --Update other values
                // frame->inliers_matches_ = list_matches[sol_idx]; // I don't think this is needed
                vo_state = OK;
            }
            else if (vo_state == OK)
            {
                // Now, we have the pose of prev two images and their keypoints matches.

                // Only the 2nd image has computed the inliers,
                //      so for all other images, we still need to find the inliers again.)

                const int frames_size = frames.size();
                my_slam::Frame::Ptr frame1 = frames[frames_size - 2];
                my_slam::Frame::Ptr frame2 = frames[frames_size - 1];

                if(1){ // -- Find their inliers by checking epipolar constraints
                    
                    // -- Extract some commonly used points
                    vector<Point2f> pts_img1, pts_img2; // matched points
                    extractPtsFromMatches(frame1->keypoints_, frame2->keypoints_, frame2->matches_, pts_img1, pts_img2);
                    vector<Point2f> pts_on_np1, pts_on_np2; // matched points on camera normalized plane
                    for (const Point2f &pt : pts_img1)
                        pts_on_np1.push_back(pixel2camNormPlane(pt, K));
                    for (const Point2f &pt : pts_img2)
                        pts_on_np2.push_back(pixel2camNormPlane(pt, K));

                    
                    // -- Here cam1 is the prev2 image, cam2 is the prev1 image.
                    const double ERR_EPPI_TRESH = 0.1;
                    Mat R = frame2->R_curr_to_prev_, t = frame2->t_curr_to_prev_;
                    const int num_matched_pts = pts_img1.size();
                    vector<int> inliers;
                    for (int i = 0; i < num_matched_pts; i++)
                    {
                        const Point2f &p1 = pts_img1[i], &p2 = pts_img2[i];
                        double err = abs(computeEpipolarConsError(p1, p2, R, t, K));
                        if (err < ERR_EPPI_TRESH)
                        {
                            inliers.push_back(i);
                        }
                    }

                    // -- Use triangulation to find these points pos in prev image's frame
                    vector<Point3f> pts3d_in_cam1, pts3d_in_cam2;
                    cout<<pts_on_np1.size()<<endl;
                    cout<<pts_on_np2.size()<<endl;
                    cout<<R<<endl;
                    cout<<t<<endl;
                    cout<<inliers.size()<<endl;
                    doTriangulation(pts_on_np1, pts_on_np2, R, t, inliers, pts3d_in_cam1);
                    for (const Point3f &pt3d : pts3d_in_cam1)
                        pts3d_in_cam2.push_back(transCoord(pt3d, R, t));

                    // -- update values
                    // frame2->inliers_of_matches_ = inliers;
                    vector<int> inliers_of_all_pts; // inliers index with respect to all the points
                    for (int idx : inliers)
                        inliers_of_all_pts.push_back(frame2->matches_[idx].trainIdx);
                    frame2->inliers_of_all_pts_ = inliers_of_all_pts;
                    frame2->inliers_pts3d_ = pts3d_in_cam2;
                }else{ // When there is pure rotation, epipolar results are bad. 
                    // use helperEstimatePossibleRelativePosesByEpipolarGeometry
                    vector<Mat> list_R, list_t, list_normal;
                    vector<vector<DMatch>> list_matches; // these are the inliers matches
                    vector<vector<Point3f>> sols_pts3d_in_cam1_by_triang;
                    helperEstimatePossibleRelativePosesByEpipolarGeometry(
                        /*Input*/
                        frame1->keypoints_, frame2->keypoints_,
                        frame1->descriptors_, frame2->descriptors_,
                        frame2->matches_,
                        K,
                        /*Output*/
                        list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
                    false); // print result
                    cout << "DEBUG: PnP: test if list_R[1] is the same:" << list_R[1]<<endl;

                    // -- update values
                    int sol_idx_for_inlier=0;
                    vector<int> inliers_of_all_pts; // inliers index with respect to all the points
                    for (const DMatch &m : list_matches[sol_idx_for_inlier])
                        inliers_of_all_pts.push_back(m.trainIdx);
                    frame2->inliers_of_all_pts_ = inliers_of_all_pts;

                    vector<Point3f> &pts3d_in_cam1 = sols_pts3d_in_cam1_by_triang[sol_idx_for_inlier];
                    vector<Point3f> pts3d_in_cam2;
                    for (const Point3f &pt3d : pts3d_in_cam1)
                        pts3d_in_cam2.push_back(transCoord(pt3d, frame2->R_curr_to_prev_, frame2->t_curr_to_prev_));
                    frame2->inliers_pts3d_ = pts3d_in_cam2;

                }

                // --  Find the intersection between [DMatches_curr] and [DMatches_prev],
                // --  and 3d-2d correspondance
                vector<Point3f> pts_3d; // a point's 3d pos in cam1 frame
                vector<Point2f> pts_2d; // a point's 2d pos in image2 pixel frame
                helperFind3Dto2DCorrespondences(frame->matches_, frame->keypoints_,
                                                frame2->inliers_of_all_pts_, frame2->inliers_pts3d_,
                                                pts_3d, pts_2d);
                cout << "Number of 3d-2d pairs: " << pts_3d.size() << endl;

                // -- Solve PnP, get T_cam1_to_cam2
                Mat R_vec, R, t;
                solvePnPRansac(pts_3d, pts_2d, K, Mat(), R_vec, t, false);
                Rodrigues(R_vec, R);

                // -- Update current camera pos
                Mat T_curr_to_prev = transRt2T(R, t);
                frame->T_w_c_ = prev_frame->T_w_c_ * T_curr_to_prev.inv();
                frame->R_curr_to_prev_ = R;
                frame->t_curr_to_prev_ = t;

                // -- Update
                vo_state = OK;
            }
        }
        // Display
        // cv::Mat img_show=rgb_img.clone();
        // cv::Scalar color(0,255,0);
        // cv::Scalar color2= cv::Scalar::all(-1);
        // cv::drawKeypoints(img_show, vo->getCurrentKeypoints(), img_show, color);
        // cv::imshow ( "rgb_img", img_show );
        // cv::waitKey (1000);

        // Save to buff.
        frames.push_back(frame);
        if (frames.size() > 10)
            frames.pop_front();

        // Print
        printf("\n\n-----Printing frame %d results:---------\n", img_id);
        cout << "R_curr_to_prev_: " << frame->R_curr_to_prev_ << endl;
        cout << "t_curr_to_prev_: " << frame->t_curr_to_prev_ << endl;
        // Return
        if (img_id == 5)
            break;
    }
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