#include <iostream>
#include <algorithm> // std::min
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "mygeometry/epipolar_geometry.h"
#include "mygeometry/feature_match.h"
#include "myslam/config.h"

using namespace std;
using namespace cv;
using namespace mygeometry;

// void doFeatureMatching(
//     Mat &img_1, Mat &img_2,
//     vector<KeyPoint> &keypoints_1, vector<KeyPoint> &keypoints_2,
//     Mat &descriptors_1, Mat &descriptors_2,
//     vector<DMatch> matches);

int main(int argc, char **argv)
{

    // camera intrinsics
    Mat K_fr1 = (Mat_<double>(3, 3) << 517.3, 0, 325.1, 0, 516.5, 249.7, 0, 0, 1); // fr1 dataset
    Mat K_fr2 = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1); // fr2 dataset
    Mat K;

    // read in images
    string img_file1, img_file2;
    string folder = "test/";
    const int IDX_TEST_CASE=1;
    if (IDX_TEST_CASE==1) // keypoints are on the same plane.
    {
        img_file1 = "fr2_1.png";
        img_file2 = "fr2_2.png";
        K = K_fr2;
    }
    else if (IDX_TEST_CASE==2) // keypoints are not on the same plane.
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
        myslam::Config::setParameterFile(filename);
        extractKeyPoints(img_1, keypoints_1); // Choose the config file before running this
        extractKeyPoints(img_2, keypoints_2);
        cout << "Number of keypoints: " << keypoints_1.size() << ", " << keypoints_2.size() << endl;
        computeDescriptors(img_1, keypoints_1, descriptors_1);
        computeDescriptors(img_2, keypoints_2, descriptors_2);
        matchFeatures(descriptors_1, descriptors_2, matches, PRINT_RES);
    }

    // get point2f
    vector<Point2f> pts_img1, pts_img2;
    extractPtsFromMatches(keypoints_1, keypoints_2, matches, pts_img1, pts_img2);

    // Estiamte motion by Essential Matrix
    Mat R_e, t_e;
    vector<int> inliers_index_e;          // index of the inliers
    vector<Point2f> inlpts1_e, inlpts2_e; // inlier points

    estiMotionByEssential(pts_img1, pts_img2, K,
                          R_e, t_e, inliers_index_e, inlpts1_e, inlpts2_e);

    // Estiamte motion by Homography Matrix
    vector<Mat> R_h_list, t_h_list, normal_list;
    vector<int> inliers_index_h;          // index of the inliers
    vector<Point2f> inlpts1_h, inlpts2_h; // inlier points

    estiMotionByHomography(pts_img1, pts_img2, K,
                           R_h_list, t_h_list, normal_list,
                           inliers_index_h, inlpts1_h, inlpts2_h);

    // Combine the motions from Essential/Homography
    vector<Mat> list_R, list_t;
    vector<vector<Point2f>> list_inlpts1, list_inlpts2;
    list_R.push_back(R_e);
    list_t.push_back(t_e);
    list_inlpts1.push_back(inlpts1_e);
    list_inlpts2.push_back(inlpts2_e);
    for (int i = 0; i < R_h_list.size(); i++)
    {
        list_R.push_back(R_h_list[i]);
        list_t.push_back(t_h_list[i]);
        list_inlpts1.push_back(inlpts1_h);
        list_inlpts2.push_back(inlpts2_h);
    }
    int num_solutions = list_R.size();

    // get intersection of inliers
    vector<int> inliers_common =
        getIntersection(inliers_index_e, inliers_index_h);
    int num_common = inliers_common.size();

    // Triangulation for all 3 solutions
    vector<vector<Point3f>> sols_pts3d_in_cam1;
    for (int i = 0; i < num_solutions; i++)
    {
        vector<Point3f> pts3d_in_cam1;
        triangulation(pts_img1, pts_img2, list_R[i], list_t[i], K, pts3d_in_cam1);
        sols_pts3d_in_cam1.push_back(pts3d_in_cam1);
    }

    // Check [Epipoloar error] and [Triangulation res pos on Normalized Plane] for each feature point
    cout << "\n---------------------------------------" << endl;
    cout << "Check [Epipoloar error] and [Triangulation res pos on Normalized Plane]" << endl;
    const int MAX_TO_CHECK = 1;
    cout << "for the first " << MAX_TO_CHECK << " points:";
    int cnt = 0;
    for (int iii = 0; iii < min(num_common, MAX_TO_CHECK); iii++)
    {
        int i = inliers_common[iii];
        cout << "\n--------------" << endl;
        cout << "Printing the " << i << "th point:" << endl;
        Point2f p1 = pts_img1[i], p2 = pts_img2[i];
        Point2f p_cam1 = pixel2camNormPlane(p1, K); // point's pos in the camera 1 frame, with depth=1
        Point2f p_cam2 = pixel2camNormPlane(p2, K);
        cout << "cam1, pixel on norm plane by inv(K)*(u,v): " << p_cam1.x << ", " << p_cam1.y << endl;
        cout << "cam2, pixel on norm plane by inv(K)*(u,v): " << p_cam2.x << ", " << p_cam2.y << endl;

        for (int j = 0; j < num_solutions; j++)
        {
            // print epipolar error
            double err_epi = computeEpipolarConsError(p1, p2, list_R[j], list_t[j], K);
            cout << "===solu " << j << ": epipolar error is " << err_epi * 1e6 << endl;

            // print triangulation result
            double depth1, depth2;
            Point2f pts3dc1_norm, pts3dc2_norm;
            Point3f pts3dc1 = sols_pts3d_in_cam1[j][i]; // 3d points pos in camera 1
            ptPosInNormPlane(pts3dc1, list_R[j], list_t[j],
                             pts3dc1_norm, depth1, pts3dc2_norm, depth2);

            cout << "By triangulation, the point pos on normalized plane is:" << endl;
            cout << "-- On cam1, pos: " << pts3dc1_norm << ". Depth:" << depth1 << endl;
            cout << "-- On cam2, pos: " << pts3dc2_norm << ". Depth:" << depth2 << endl;
        }

        cout << endl;
    }

    // Print the error of the above three methods
    printf("\n------------------------------------\n");
    printf("Print the mean error of each E/H method by using the inlier points.");
    for (int j = 0; j < num_solutions; j++)
    {
        Mat &R = list_R[j], &t = list_t[j];
        vector<Point2f> &inlpts1 = list_inlpts1[j];
        vector<Point2f> &inlpts2 = list_inlpts2[j];
        vector<Point3f> &pts3d = sols_pts3d_in_cam1[j];

        // epipolar error
        double err_epi = computeEpipolarConsError(inlpts1, inlpts2, list_R[j], list_t[j], K);

        // On camera normalized plane,
        // the error between triangulation and real
        double err_triang = 0;
        int num_pts = pts_img1.size();
        for (int i = 0; i < num_pts; i++)
        {
            Point2f p1 = pts_img1[i], p2 = pts_img2[i];
            Point2f p_cam1 = pixel2camNormPlane(p1, K); // point's pos in the camera 1 frame, with depth=1
            Point2f p_cam2 = pixel2camNormPlane(p2, K);

            // print triangulation result
            double depth1, depth2;
            Point2f pts3dc1_norm, pts3dc2_norm;
            Point3f pts3dc1 = sols_pts3d_in_cam1[j][i]; // 3d points pos in camera 1
            ptPosInNormPlane(pts3dc1, list_R[j], list_t[j],
                             pts3dc1_norm, depth1, pts3dc2_norm, depth2);

            double err_x = p_cam1.x - pts3dc1_norm.x;
            double err_y = p_cam2.y - pts3dc2_norm.y;
            double err = err_x * err_x + err_y * err_y;
            err_triang += err;
        }
        if (num_pts == 0)
            err_triang = 9999999999;
        else
            err_triang = sqrt(err_triang / num_pts);

        // print
        printf("\n---------------\n");
        printf("Solution %d, num inliers = %d \n", j, (int)inlpts1.size());
        print_R_t(R, t);
        printf("-- Eppipolar cons error = %f \n", err_epi);
        printf("-- Triangulation error = %f \n", err_triang); // the error on the normalized image plane
    }

    // plot image
    Mat Idst;
    string window_name;
    
    window_name = "Matched keypoints";
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, Idst);
    cv::namedWindow(window_name, WINDOW_AUTOSIZE);
    cv::imshow(window_name, Idst);
    imwrite(window_name+".png", Idst); 
    waitKey(1);

    window_name = "inliers keypoints";
    vector<KeyPoint> matched_kpts1=pts2keypts(pts_img1);
    vector<KeyPoint> matched_kpts2=pts2keypts(pts_img2);
    vector<DMatch> matches_inliers=inliers2DMatches(inliers_index_e);
    drawMatches(img_1, matched_kpts1, img_2, matched_kpts2, matches_inliers, Idst);
    cv::namedWindow(window_name, WINDOW_AUTOSIZE);
    cv::imshow(window_name, Idst);
    imwrite(window_name+".png", Idst); 
    waitKey(1);


    // return
    waitKey();
    cv::destroyAllWindows();
    return 0;
}
