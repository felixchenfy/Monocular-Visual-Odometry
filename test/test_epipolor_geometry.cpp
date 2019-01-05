// Test functions in "include/my_geometry", including:
//  * Keypoint extraction and feature matching.
//  * Estimate camera motion by Essential/Homography matrix and triangulation (1+2=3 solutions).
// The error of the estimated motions are displayed.
// (As for which to choose from the 3 solutions, I might do it in "test_vo.cpp") ### <-- delete this later

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
    const int IDX_TEST_CASE = 2;
    if (IDX_TEST_CASE == 1) // keypoints are on the same plane.
    {
        img_file1 = "fr2_1.png";
        img_file2 = "fr2_2.png";
        K = K_fr2;
    }
    else if (IDX_TEST_CASE == 2) // keypoints are not on the same plane.
    {
        img_file1 = "fr1_1.png";
        img_file2 = "fr1_2.png";
        K = K_fr1;
    }
    else
    {
        img_file1 = "fr1_3.png";
        img_file2 = "fr1_4.png";
        K = K_fr1;
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

    // estimation motion
    vector<Mat> list_R, list_t, list_normal;
    vector<vector<DMatch>> list_matches;
    vector<vector<Point3f>> sols_pts3d_in_cam1_by_triang;
    helperEstimatePossibleRelativePosesByEpipolarGeometry(
        /*Input*/
        keypoints_1, keypoints_2, matches, descriptors_1, descriptors_2, K,
        /*Output*/
        list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
        false); // print result

    // Debug 2:
    // Print the error of the above three methods
    int num_solutions=list_R.size();
    printf("\n------------------------------------\n");
    printf("Print the mean error of each E/H method by using the inlier points.");
    for (int i = 0; i < num_solutions; i++)
    {
        Mat &R = list_R[i], &t = list_t[i];
        vector<DMatch> &matches=list_matches[i];
        vector<Point3f> &pts3d = sols_pts3d_in_cam1_by_triang[i];
        vector<Point2f> inlpts1, inlpts2;
        extractPtsFromMatches(keypoints_1, keypoints_2, matches, inlpts1 ,inlpts2);
        
        // epipolar error
        double err_epi = computeEpipolarConsError(inlpts1, inlpts2, R, t, K);

        // In image frame,  the error between triangulation and real
        double err_triang = 0;
        int num_inlier_pts = inlpts1.size();
        for (int j = 0; j < num_inlier_pts; j++)
        {
            Point2f &p1 = inlpts1[j], &p2 = inlpts2[j];
            // print triangulation result
            Mat pts3dc1 = Point3f_to_Mat(sols_pts3d_in_cam1_by_triang[i][j]); // 3d pos in camera 1
            Mat pts3dc2 = R * pts3dc1 + t;
            Point2f pts2dc1 = cam2pixel(pts3dc1, K);
            Point2f pts2dc2 = cam2pixel(pts3dc2, K);
            err_triang = err_triang + calcError(p1, pts2dc1) + calcError(p2, pts2dc2);
        }
        if (num_inlier_pts == 0)
            err_triang = 9999999999;
        else
            err_triang = sqrt(err_triang / 2.0 / num_inlier_pts);

        // print
        printf("\n---------------\n");
        printf("Solution %d, num inliers = %d \n", i, num_inlier_pts);
        print_R_t(R, t);
        if (i >= 1)
            cout << "norm is:" << (list_normal[i]).t() << endl;
        printf("-- Epipolar cons error = %f \n", err_epi);
        printf("-- Triangulation error = %f \n", err_triang); // the error on the normalized image plane
    }

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
