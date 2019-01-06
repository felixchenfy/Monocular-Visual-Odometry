// Test functions in "include/my_geometry", including:
//  * Keypoint extraction and feature matching.
//  * Estimate camera motion by Essential/Homography matrix (totally 1+2=3 solutions). All returned in a vector.
//  * Triangulation.
//  * Compute epipolar error and triangulation error in pixel.

#include <iostream>
#include <algorithm> // std::min
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "my_geometry/feature_match.h"
#include "my_geometry/epipolar_geometry.h"
#include "my_geometry/motion_estimation.h"
#include "my_basics/config.h"

using namespace std;
using namespace cv;
using namespace my_geometry;

int main(int argc, char **argv)
{

    // camera intrinsics
    Mat K_fr1 = (Mat_<double>(3, 3) << 517.3, 0, 325.1, 0, 516.5, 249.7, 0, 0, 1); // fr1 dataset
    Mat K_fr2 = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1); // fr2 dataset
    Mat K;

    // read in images
    string img_file1, img_file2;
    string folder = "test_data/";
    const int IDX_TEST_CASE = 1;

    if (IDX_TEST_CASE == 1) // keypoints are not on the same plane.
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
    
    Mat img_1 = imread(folder + img_file1);
    Mat img_2 = imread(folder + img_file2);

    // Extract keypoints and features. Match keypoints
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    Mat descriptors_1, descriptors_2;
    // doFeatureMatching(img_1, img_2, keypoints_1, keypoints_2, descriptors_1, descriptors_2, matches);

    bool PRINT_RES = true, SET_PARAM_BY_YAML = false;
    if (0)
    {                                                            // use default settings
        extractKeyPoints(img_1, keypoints_1, SET_PARAM_BY_YAML); // Choose the config file before running this
        extractKeyPoints(img_2, keypoints_2, SET_PARAM_BY_YAML);
        cout << "Number of keypoints: " << keypoints_1.size() << ", " << keypoints_2.size() << endl;
        computeDescriptors(img_1, keypoints_1, descriptors_1, SET_PARAM_BY_YAML);
        computeDescriptors(img_2, keypoints_2, descriptors_2, SET_PARAM_BY_YAML);
        matchFeatures(descriptors_1, descriptors_2, matches, PRINT_RES, SET_PARAM_BY_YAML);
    }
    else
    { // use settings in .yaml file
        string filename = "config/default.yaml";
        my_basics::Config::setParameterFile(filename);
        extractKeyPoints(img_1, keypoints_1); // Choose the config file before running this
        extractKeyPoints(img_2, keypoints_2);
        cout << "Number of keypoints: " << keypoints_1.size() << ", " << keypoints_2.size() << endl;
        computeDescriptors(img_1, keypoints_1, descriptors_1);
        computeDescriptors(img_2, keypoints_2, descriptors_2);
        matchFeatures(descriptors_1, descriptors_2, matches, PRINT_RES);
    }

    // Estimation motion
    vector<Mat> list_R, list_t, list_normal;
    vector<vector<DMatch>> list_matches;
    vector<vector<Point3f>> sols_pts3d_in_cam1_by_triang;
    helperEstimatePossibleRelativePosesByEpipolarGeometry(
        /*Input*/
        keypoints_1, keypoints_2, descriptors_1, descriptors_2, matches, K,
        /*Output*/
        list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
        false); // print result

    // Compute [epipolar error] and [trigulation error on norm plane] for the 3 solutions (E, H1, H2)
    vector<double> list_error_epipolar;
    vector<double> list_error_triangulation;
    helperEvaluateEstimationsError(
        keypoints_1, keypoints_2, list_matches,
        sols_pts3d_in_cam1_by_triang,
        list_R, list_t, list_normal,
        K,
        /*output*/
        list_error_epipolar, list_error_triangulation,
        true); // print result

    // plot image
    Mat Idst;
    string window_name;

    window_name = "Detected and matched keypoints";
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, Idst);
    cv::namedWindow(window_name, WINDOW_AUTOSIZE);
    cv::imshow(window_name, Idst);
    imwrite(window_name + ".png", Idst);
    waitKey(1);

    window_name = "Detected and inliers keypoints";
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, list_matches[0], Idst);
    cv::namedWindow(window_name, WINDOW_AUTOSIZE);
    cv::imshow(window_name, Idst);
    imwrite(window_name + ".png", Idst);
    waitKey(1);

    // return
    waitKey();
    cv::destroyAllWindows();
    return 0;
}
