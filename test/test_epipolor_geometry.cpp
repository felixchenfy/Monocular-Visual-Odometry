// Test functions in "include/geometry", including:
//  * cv::KeyPoint extraction and feature matching.
//  * Estimate camera motion by Essential/Homography matrix (totally 1+2=3 solutions). All returned in a vector.
//  * Triangulation.
//  * Compute epipolar error and triangulation error in pixel.

/*
How to run:
bin/test_epipolor_geometry image0001.jpg image0002.jpg
bin/test_epipolor_geometry image0001.jpg image0015.jpg
*/

#include <iostream>
#include <algorithm> // std::min
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "my_slam/geometry/feature_match.h"
#include "my_slam/geometry/epipolar_geometry.h"
#include "my_slam/geometry/motion_estimation.h"
#include "my_slam/basics/config.h"

using namespace std;
using namespace my_slam;

int main(int argc, char **argv)
{

    // camera intrinsics
    cv::Mat K_fr1 = (cv::Mat_<double>(3, 3) << 517.3, 0, 325.1, 0, 516.5, 249.7, 0, 0, 1); // fr1 dataset
    cv::Mat K_fr2 = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1); // fr2 dataset
    cv::Mat K_mtb = (cv::Mat_<double>(3, 3) << 615, 0, 320, 0, 615, 240, 0, 0, 1);         // fr2 dataset
    cv::Mat K;

    // read in images
    string img_file1, img_file2;
    string folder = "data/test_data/";
    int IDX_TEST_CASE = 1;

    if (argc - 1 == 2)
    {
        // bin/test_epipolor_geometry rgb_00000.png rgb_00001.png # inliers = 90+
        // bin/test_epipolor_geometry rgb_00003.png rgb_00004.png # inliers = 35
        // bin/test_epipolor_geometry rgb_00004.png rgb_00005.png # inliers = 90+
        // bin/test_epipolor_geometry image0001.jpg image0015.jpg # Mean Score: E=11.0, H=9.1
        // bin/test_epipolor_geometry image0001.jpg image0002.jpg # Mean Score: E=11.3, H=11.0
        IDX_TEST_CASE = -1;
        img_file1 = argv[1];
        img_file2 = argv[2];
        K = K_mtb;
    }

    else if (IDX_TEST_CASE == 1) // keypoints are not on the same plane.
    {
        img_file1 = "fr1_1_1.png";
        img_file2 = "fr1_1_2.png";
        K = K_fr1;
    }
    else if (IDX_TEST_CASE == 2) // keypoints are almost on the same plane.
    {
        img_file1 = "fr1_2_1.png";
        img_file2 = "fr1_2_1.png";
        K = K_fr1;
    }
    else if (IDX_TEST_CASE == 3) // keypoints are almost on the same plane.
    {
        img_file1 = "fr2_1_1.png";
        img_file2 = "fr2_1_2.png";
        K = K_fr2;
    }

    //====================================== Main program starts =========================================

    // Read image
    cv::Mat img_1 = imread(folder + img_file1);
    cv::Mat img_2 = imread(folder + img_file2);

    // Extract keypoints and features. Match keypoints
    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    vector<cv::DMatch> matches;
    cv::Mat descriptors_1, descriptors_2;
    // doFeatureMatching(img_1, img_2, keypoints_1, keypoints_2, descriptors_1, descriptors_2, matches);

    bool is_print_res = true, SET_PARAM_BY_YAML = false;
    if (0)
    {                                                            // use default settings
        geometry::calcKeyPoints(img_1, keypoints_1, SET_PARAM_BY_YAML); // Choose the config file before running this
        geometry::calcKeyPoints(img_2, keypoints_2, SET_PARAM_BY_YAML);
        cout << "Number of keypoints: " << keypoints_1.size() << ", " << keypoints_2.size() << endl;
        geometry::calcDescriptors(img_1, keypoints_1, descriptors_1, SET_PARAM_BY_YAML);
        geometry::calcDescriptors(img_2, keypoints_2, descriptors_2, SET_PARAM_BY_YAML);
        geometry::matchFeatures(descriptors_1, descriptors_2, matches, is_print_res, SET_PARAM_BY_YAML);
    }
    else
    { // use settings in .yaml file
        string filename = "config/config.yaml";
        basics::Config::setParameterFile(filename);
        geometry::calcKeyPoints(img_1, keypoints_1); // Choose the config file before running this
        geometry::calcKeyPoints(img_2, keypoints_2);
        cout << "Number of keypoints: " << keypoints_1.size() << ", " << keypoints_2.size() << endl;
        geometry::calcDescriptors(img_1, keypoints_1, descriptors_1);
        geometry::calcDescriptors(img_2, keypoints_2, descriptors_2);
        geometry::matchFeatures(descriptors_1, descriptors_2, matches, is_print_res);
        printf("Number of matches: %d\n", (int)matches.size());

    }

    // Estimation motion
    vector<cv::Mat> list_R, list_t, list_normal;
    vector<vector<cv::DMatch>> list_matches;
    vector<vector<cv::Point3f>> sols_pts3d_in_cam1_by_triang;
    const bool is_print_res = false, is_calc_homo = true, is_frame_cam2_to_cam1 = true;
    int best_sol = geometry::helperEstimatePossibleRelativePosesByEpipolarGeometry(
        /*Input*/
        keypoints_1, keypoints_2, matches, K,
        /*Output*/
        list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
        /*settings*/
        is_print_res, is_calc_homo, is_frame_cam2_to_cam1);
    cout << "Best solution is: " << best_sol << endl;

    // Compute [epipolar error] and [trigulation error on norm plane] for the 3 solutions (E, H1, H2)
    geometry::helperEvalEppiAndTriangErrors(
        keypoints_1, keypoints_2, list_matches,
        sols_pts3d_in_cam1_by_triang,
        list_R, list_t, list_normal,
        K,
        false); // print result

    // plot image
    cv::Mat Idst;
    string window_name;

    window_name = "Detected and matched keypoints";
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, Idst);
    cv::namedWindow(window_name, WINDOW_AUTOSIZE);
    cv::imshow(window_name, Idst);
    cv::imwrite(window_name + ".png", Idst);
    cv::waitKey(1);

    window_name = "Detected and inliers keypoints";
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, list_matches[0], Idst);
    cv::namedWindow(window_name, WINDOW_AUTOSIZE);
    cv::imshow(window_name, Idst);
    cv::imwrite(window_name + ".png", Idst);
    cv::waitKey(1);

    // -- I spot some wrong inliers, I'm going to show it.
    // int cnt=0;
    // for (const cv::DMatch &d:list_matches[0]){
    //     cv::Point2f p1=keypoints_1[d.queryIdx].pt;
    //     cv::Point2f p2=keypoints_2[d.trainIdx].pt;
    //     if(abs(p1.x-p2.x)>20){
    //         cout << endl;
    //         printf("The %dth kpt, p1(%.2f, %.2f), p2(%.2f, %.2f)\n",
    //             cnt, p1.x, p1.y, p2.x, p2.y);
    //         double error=computeEpipolarConsError(p1,p2,list_R[0],list_t[0],K);
    //         printf("Print its epi error:%.6f", error);
    //         cout<<endl;
    //     }
    //     cnt++;
    // }
    // return
    cv::waitKey();
    cv::destroyAllWindows();
    return 0;
}
