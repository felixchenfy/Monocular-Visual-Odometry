#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "mygeometry/epipolar_geometry.h"
#include "mygeometry/feature_match.h"
// #include "myslam/config.h"

using namespace std;
using namespace cv;
using namespace mygeometry;

int main(int argc, char **argv)
{

    // camera intrinsics
    Mat K_fr1 = (Mat_<double>(3, 3) << 517.3, 0, 325.1, 0, 516.5, 249.7, 0, 0, 1); // fr1 dataset
    Mat K_fr2 = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1); // fr2 dataset
    Mat K;

    // read in images
    string img_file1, img_file2;
    string folder="test/";
    if (0)
    {
        img_file1 = "fr2_1.png";
        img_file2 = "fr2_2.png";
        K = K_fr2;
    }
    else if (1)
    {
        img_file1 = "fr1_1.png";
        img_file2 = "fr1_2.png";
        K = K_fr1;
    }
    Mat img_1 = imread(folder+img_file1);
    Mat img_2 = imread(folder+img_file2);

    // Extract keypoints and features. Match keypoints
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    Mat descriptors_1, descriptors_2;
    bool PRINT_RES=true, SET_PARAM_BY_YAML=false;
    extractKeyPoints(img_1, keypoints_1, SET_PARAM_BY_YAML); // Choose the config file before running this 
    extractKeyPoints(img_2, keypoints_2, SET_PARAM_BY_YAML);
    cout << "Number of keypoints: " << keypoints_1.size() << ", "<<keypoints_2.size() << endl;
    computeDescriptors(img_1, keypoints_1, descriptors_1, SET_PARAM_BY_YAML);
    computeDescriptors(img_2, keypoints_2, descriptors_2, SET_PARAM_BY_YAML);
    matchFeatures(descriptors_1, descriptors_2, matches, PRINT_RES, SET_PARAM_BY_YAML);

    // get point2f
    vector<Point2f> points1, points2;
    extractPtsFromMatches(keypoints_1, keypoints_2, matches, points1, points2);

    // Estiamte motion by Essential Matrix
    Mat R_e, t_e;
    vector<int> inliers_index_e;          // index of the inliers
    vector<Point2f> inlpts1_e, inlpts2_e; // inlier points

    estiMotionByEssential(points1, points2, K,
                          R_e, t_e, inliers_index_e, inlpts1_e, inlpts2_e);

    // Estiamte motion by Homography Matrix
    vector<Mat> R_h_list, t_h_list, normal_list;
    vector<int> inliers_index_h;          // index of the inliers
    vector<Point2f> inlpts1_h, inlpts2_h; // inlier points

    estiMotionByHomography(points1, points2, K,
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

    // Triangulation for all 3 solutions
    vector<vector<Point3f>> sols_pts3d_in_cam1;
    for (int i = 0; i < num_solutions; i++)
    {
        vector<Point3f> pts3d_in_cam1;
        triangulation(points1, points2, list_R[i], list_t[i], K, pts3d_in_cam1);
        sols_pts3d_in_cam1.push_back(pts3d_in_cam1);
    }

    // Check [Epipoloar error] and [Triangulation res pos on Normalized Plane] for each feature point
    cout << "\n---------------------------------------" << endl;
    cout << "Check [Epipoloar error] and [Triangulation res pos on Normalized Plane]"
         << " for each feature point" << endl;
    const int NUMBER_OF_KEYPOINT_TO_CHECK = 10;
    for (int i = 0; i < NUMBER_OF_KEYPOINT_TO_CHECK; i++)
    {
        cout << "\n--------------" << endl;
        cout << "Printing the " << i << "th point:" << endl;
        Point2f p1 = points1[i], p2 = points2[i];
        Point2f p_cam1 = pixel2camNormPlane(p1, K); // point's pos in the camera 1 frame, with depth=1
        Point2f p_cam2 = pixel2camNormPlane(p2, K);
        cout << "cam1, pixel on norm plane by inv(K)*(u,v): " << p_cam1.x << ", " << p_cam1.y << endl;
        cout << "cam2, pixel on norm plane by inv(K)*(u,v): " << p_cam2.x << ", " << p_cam2.y << endl;

        for (int j = 0; j < num_solutions; j++)
        {
            // print epipolar error
            double epi_err = computeEpipolarConsError(p1, p2, list_R[j], list_t[j], K);
            cout << "===solu " << j << ": epipolar error is " << epi_err * 1e6 << endl;

            // print triangulation result
            double depth;
            Point3f pts3dc1 = sols_pts3d_in_cam1[j][i]; // 3d points pos in camera 1
            
            depth = pts3dc1.z;
            Point2d pts3dc1_norm(pts3dc1.x / depth, pts3dc1.y / depth);
            cout << "cam1, by triang, on norm plane, pos:" << pts3dc1_norm << ". Depth:" << depth << endl;

            Mat tmp = list_R[j] * (Mat_<double>(3, 1) << pts3dc1.x, pts3dc1.y, pts3dc1.z) + list_t[j];
            Point3f pts3dc2(tmp.at<double>(0, 0), tmp.at<double>(1, 0), tmp.at<double>(2, 0));
            depth = pts3dc2.z;
            Point2d pts3dc2_norm(pts3dc2.x / depth, pts3dc2.y / depth);
            cout << "cam2, by triang, on norm plane, pos:" << pts3dc2_norm << ". Depth:" << depth << endl;
        }

        cout << endl;
        break;
    }

    // triangulate points in the above 3 situations, plot them, and see if its correct.

    return 0;
}
