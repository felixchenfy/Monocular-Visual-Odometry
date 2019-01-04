
#include "my_geometry/motion_estimation.h"
using namespace my_basics;

#define DEBUG_PRINT_RESULT true

namespace my_geometry
{
void helperEstimatePossibleRelativePosesByEpipolarGeometry(
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    const Mat &descriptors_1,
    const Mat &descriptors_2,
    const Mat &K,
    vector<Mat> &list_R, vector<Mat> &list_t,
    vector<vector<DMatch>> &list_matches,
    vector<Mat> &list_normal,
    vector<vector<Point3f>> &sols_pts3d_in_cam1)
{
    list_R.clear();
    list_t.clear();
    list_matches.clear();
    list_normal.clear();
    sols_pts3d_in_cam1.clear();

    // Get matched points: pts_img1 & pts_img2
    // All computations after this step are operated on these matched points
    vector<Point2f> pts_img1_all = convertKeypointsToPoint2f(keypoints_1);
    vector<Point2f> pts_img2_all = convertKeypointsToPoint2f(keypoints_2);
    vector<Point2f> pts_img1, pts_img2; // matched points
    extractPtsFromMatches(pts_img1_all, pts_img2_all, matches, pts_img1, pts_img2);
    vector<Point2f> pts_on_np1, pts_on_np2; // matched points on camera normalized plane
    int num_matched = (int)matches.size();
    for (int i = 0; i < num_matched; i++)
    {
        pts_on_np1.push_back(pixel2camNormPlane(pts_img1[i], K));
        pts_on_np2.push_back(pixel2camNormPlane(pts_img2[i], K));
    }

    // Estiamte motion by Essential Matrix
    Mat R_e, t_e, essential_matrix;
    vector<int> inliers_index_e; // index of the inliers
    estiMotionByEssential(pts_img1, pts_img2, K,
                          essential_matrix,
                          R_e, t_e, inliers_index_e);

    if (DEBUG_PRINT_RESULT)
    {
        printResult_estiMotionByEssential(essential_matrix, // debug
                                          inliers_index_e, R_e, t_e);
    }

    // Estiamte motion by Homography Matrix
    vector<Mat> R_h_list, t_h_list, normal_list;
    vector<int> inliers_index_h; // index of the inliers
    Mat homography_matrix;
    estiMotionByHomography(pts_img1, pts_img2, K,
                           homography_matrix,
                           R_h_list, t_h_list, normal_list,
                           inliers_index_h);
    removeWrongRtOfHomography(pts_on_np1, pts_on_np2, R_h_list, t_h_list, normal_list);
    int num_h_solutions = R_h_list.size();
    if (DEBUG_PRINT_RESULT)
    {
        printResult_estiMotionByHomography(homography_matrix, // debug
                                           inliers_index_h, R_h_list, t_h_list, normal_list);
    }

    // Combine the motions from Essential/Homography
    // Return: vector<Mat> list_R, list_t, list_normal;
    vector<vector<Point2f>> list_inlpts1, list_inlpts2;
    vector<vector<int>> list_inliers;
    list_R.push_back(R_e);
    list_t.push_back(t_e);
    list_normal.push_back(Mat());
    list_inliers.push_back(inliers_index_e);
    for (int i = 0; i < num_h_solutions; i++)
    {
        list_R.push_back(R_h_list[i]);
        list_t.push_back(t_h_list[i]);
        list_normal.push_back(normal_list[i]);
        list_inliers.push_back(inliers_index_h);
    }
    int num_solutions = list_R.size();

    // Triangulation for all 3 solutions
    // return: vector<vector<Point3f>> sols_pts3d_in_cam1;
    for (int i = 0; i < num_solutions; i++)
    {
        vector<Point3f> pts3d_in_cam1;
        vector<Point2f> inlpts1, inlpts2;
        doTriangulation(pts_on_np1, pts_on_np2, list_R[i], list_t[i], list_inliers[i], pts3d_in_cam1);
        removeWrongTriangulations(list_inliers[i], pts3d_in_cam1);
        sols_pts3d_in_cam1.push_back(pts3d_in_cam1);
    }

    // Debug
    if (DEBUG_PRINT_RESULT)
    {
        print_EpipolarError_and_TriangulationResult( // for each feature point
            pts_img1, pts_img2, pts_on_np1, pts_on_np2,
            sols_pts3d_in_cam1,
            list_inliers, list_R, list_t,
            K);
    }

    // Convert the inliers to the DMatch of the original points
    vector<DMatch> res_matches_e, res_matches_h;
    for (auto &idx : inliers_index_e)
        res_matches_e.push_back(DMatch(matches[idx].queryIdx, matches[idx].trainIdx, 0.0));
    for (auto &idx : inliers_index_h)
        res_matches_h.push_back(DMatch(matches[idx].queryIdx, matches[idx].trainIdx, 0.0));
    list_matches.push_back(res_matches_e);
    for (int i = 0; i < num_h_solutions; i++)
    {
        list_matches.push_back(res_matches_h);
    }
}

// ---------------------------------------
// ---------------------------------------
// ----------- debug functions -----------
// ---------------------------------------
// ---------------------------------------

void printResult_estiMotionByEssential(
    const Mat &essential_matrix,
    const vector<int> &inliers_index,
    const Mat &R,
    const Mat &t)
{
    cout << endl
         << "=====" << endl;
    cout << "* Essential_matrix (by findEssentialMat) is: " << endl
         << essential_matrix << endl;
    cout << "* Number of inliers (after triangulation): " << inliers_index.size() << endl;
    cout << "* Recovering R and t from essential matrix:" << endl;
    print_R_t(R, t);
    cout << endl;
}

void printResult_estiMotionByHomography(
    const Mat &homography_matrix,
    const vector<int> &inliers_index,
    const vector<Mat> &Rs, const vector<Mat> &ts,
    vector<Mat> &normals)
{
    cout << endl
         << "=====" << endl;
    cout << "* Homography_matrix (by findHomography) is " << endl
         << homography_matrix << endl;
    cout << "* Number of inliers: " << inliers_index.size() << endl;

    // cout << "The inliers_mask is " << inliers_mask << endl;
    // !!! add it later

    int num_solutions = Rs.size();
    for (int i = 0; i < num_solutions; i++)
    {
        cout << endl;
        cout << "* Solution " << i + 1 << ":" << endl; // Start index from 1. The index 0 is for essential matrix.
        print_R_t(Rs[i], ts[i]);
        cout << "plane normal: " << normals[i].t() << endl;
    }
}

// Check [Epipoloar error] and [Triangulation result] for each feature point
void print_EpipolarError_and_TriangulationResult(
    vector<Point2f> pts_img1, vector<Point2f> pts_img2, vector<Point2f> pts_on_np1, vector<Point2f> pts_on_np2,
    vector<vector<Point3f>> sols_pts3d_in_cam1,
    vector<vector<int>> list_inliers,
    vector<Mat> list_R, vector<Mat> list_t,
    Mat K)
{
    const int MAX_TO_CHECK = 10;
    int num_solutions = list_R.size();
    vector<int> inliers_index_e=list_inliers[0];
    vector<int> inliers_index_h=list_inliers[1];

    cout << "\n---------------------------------------" << endl;
    cout << "Check [Epipoloar error] and [Triangulation result]" << endl;
    cout << "for the first " << MAX_TO_CHECK << " points:";

    // Iterate through points.
    int cnt = 0;
    int num_points = pts_img1.size();
    for (int i = 0; i < num_points && cnt < MAX_TO_CHECK; i++)
    {
        auto pe=find(inliers_index_e.begin(), inliers_index_e.end(), i);
        auto ph=find(inliers_index_h.begin(), inliers_index_h.end(), i);
        if(pe == inliers_index_e.end() || ph ==inliers_index_h.end())continue;
        int ith_in_e_inliers = pe-inliers_index_e.begin();
        int ith_in_h_inliers = ph-inliers_index_h.begin();
        cout << "\n--------------" << endl;
        printf("Printing the %dth (in common) and %dth (in matched) point's real position in image:\n",cnt++,i);

        // Print point pos in image frame.
        Point2f p1 = pts_img1[i], p2 = pts_img2[i];
        cout << "cam1, pixel pos (u,v): " << p1 << endl;
        cout << "cam2, pixel pos (u,v): " << p2 << endl;

        // Print result of each method.
        Point2f p_cam1 = pts_on_np1[i]; // point pos on the normalized plane
        Point2f p_cam2 = pts_on_np2[i];
        for (int j = 0; j < num_solutions; j++)
        {
            Mat &R = list_R[j], &t = list_t[j];

            // print epipolar error
            double err_epi = computeEpipolarConsError(p1, p2, R, t, K);
            cout << "===solu " << j << ": epipolar error is " << err_epi * 1e6 << endl;

            // print triangulation result
            int ith_in_curr_sol;
            if(j==0)ith_in_curr_sol=ith_in_e_inliers;
            else ith_in_curr_sol=ith_in_h_inliers;

            Mat pts3dc1 = Point3f_to_Mat(sols_pts3d_in_cam1[j][ith_in_curr_sol]); // 3d pos in camera 1
            Mat pts3dc2 = R * pts3dc1 + t;
            Point2f pts2dc1 = cam2pixel(pts3dc1, K);
            Point2f pts2dc2 = cam2pixel(pts3dc2, K);

            cout << "-- In img1, pos: " << pts2dc1 << endl;
            cout << "-- In img2, pos: " << pts2dc2 << endl;
            cout << "-- On cam1, pos: " << pts3dc1.t() << endl;
            cout << "-- On cam2, pos: " << pts3dc2.t() << endl;
        }

        cout << endl;
    }
}

} // namespace my_geometry