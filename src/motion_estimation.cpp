
#include "my_geometry/motion_estimation.h"
using namespace my_basics;

#define DEBUG_PRINT_RESULT false

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
    vector<vector<Point3f>> &sols_pts3d_in_cam1,
    const bool print_res)
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

    if (print_res && DEBUG_PRINT_RESULT)
    {
        printResult_estiMotionByEssential(essential_matrix, // debug
                                          inliers_index_e, R_e, t_e);
    }

    // Estiamte motion by Homography Matrix
    vector<Mat> R_h_list, t_h_list, normal_list;
    vector<int> inliers_index_h; // index of the inliers
    Mat homography_matrix;
    estiMotionByHomography(pts_img1, pts_img2, K,
                           /*output*/
                           homography_matrix,
                           R_h_list, t_h_list, normal_list,
                           inliers_index_h);
    removeWrongRtOfHomography(pts_on_np1, pts_on_np2, inliers_index_h, R_h_list, t_h_list, normal_list);
    int num_h_solutions = R_h_list.size();
    if (print_res && DEBUG_PRINT_RESULT)
    {
        printResult_estiMotionByHomography(homography_matrix, // debug
                                           inliers_index_h, R_h_list, t_h_list, normal_list);
    }

    // Combine the motions from Essential/Homography
    // Return: vector<Mat> list_R, list_t, list_normal;
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
        doTriangulation(pts_on_np1, pts_on_np2, list_R[i], list_t[i], list_inliers[i], pts3d_in_cam1);
        // removeWrongTriangulations(list_inliers[i], pts3d_in_cam1);
        sols_pts3d_in_cam1.push_back(pts3d_in_cam1);
        if (DEBUG_PRINT_RESULT && i == 1)
        {
            printf("\n\n----------------------------------------------------\n");
            printf("DEBUGING: print triangulation result of solution %d\n\n", i);
            int cnt = 0;
            for (int idx : list_inliers[i])
            {
                Point2f p1_real = pts_img1[idx];
                Point2f p1_predict = cam2pixel(pts3d_in_cam1[cnt], K);
                cout << cnt << "th inlier, " << idx << "th keypoint" << endl
                     << "-- Pos in image: real:" << p1_real << ", predict" << p1_predict << endl;
                cnt++;
            }
        }
    }

    // Debug EpipolarError and TriangulationResult
    if (print_res)
    {
        // print_EpipolarError_and_TriangulationResult_By_Solution(
        //     pts_img1, pts_img2, pts_on_np1, pts_on_np2,
        //     sols_pts3d_in_cam1,list_inliers, list_R, list_t, K);
        print_EpipolarError_and_TriangulationResult_By_Common_Inlier(
            pts_img1, pts_img2, pts_on_np1, pts_on_np2,
            sols_pts3d_in_cam1, list_inliers, list_R, list_t, K);
    }

    // Convert the inliers to the DMatch of the original points
    for (int i = 0; i < num_solutions; i++)
    {
        list_matches.push_back(vector<DMatch>());
        const vector<int> &inliers = list_inliers[i];
        for (const int &idx : inliers)
        {
            list_matches[i].push_back(
                DMatch(matches[idx].queryIdx, matches[idx].trainIdx, matches[idx].distance));
        }
    }
}

double helperEvaluateEstimationsError(
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<vector<DMatch>> &list_matches,
    const vector<vector<Point3f>> &sols_pts3d_in_cam1_by_triang,
    const vector<Mat> &list_R, const vector<Mat> &list_t, const vector<Mat> &list_normal,
    const Mat &K,
    vector<double> &list_error_epipolar,
    vector<double> &list_error_triangulation, // the error on the normalized image plane
    bool print_res)
{
    list_error_epipolar.clear();
    list_error_triangulation.clear();
    int num_solutions = list_R.size();

    for (int i = 0; i < num_solutions; i++)
    {
        const Mat &R = list_R[i], &t = list_t[i];
        const vector<DMatch> &matches = list_matches[i];
        const vector<Point3f> &pts3d = sols_pts3d_in_cam1_by_triang[i];
        vector<Point2f> inlpts1, inlpts2;
        extractPtsFromMatches(keypoints_1, keypoints_2, matches, inlpts1, inlpts2);

        // epipolar error
        double err_epipolar = computeEpipolarConsError(inlpts1, inlpts2, R, t, K);

        // In image frame,  the error between triangulation and real
        double err_triangulation = 0;
        int num_inlier_pts = inlpts1.size();
        for (int idx_inlier = 0; idx_inlier < num_inlier_pts; idx_inlier++)
        {
            const Point2f &p1 = inlpts1[idx_inlier], &p2 = inlpts2[idx_inlier];
            // print triangulation result
            Mat pts3dc1 = Point3f_to_Mat(pts3d[idx_inlier]); // 3d pos in camera 1
            Mat pts3dc2 = R * pts3dc1 + t;
            Point2f pts2dc1 = cam2pixel(pts3dc1, K);
            Point2f pts2dc2 = cam2pixel(pts3dc2, K);
            err_triangulation = err_triangulation + calcError(p1, pts2dc1) + calcError(p2, pts2dc2);
            // printf("%dth inlier, err_triangulation = %f\n", idx_inlier, err_triangulation);
        }
        if (num_inlier_pts == 0)
            err_triangulation = 9999999999;
        else
            err_triangulation = sqrt(err_triangulation / 2.0 / num_inlier_pts);

        // print
        list_error_epipolar.push_back(err_epipolar);
        list_error_triangulation.push_back(err_triangulation);
    }
    if (print_res)
    {
        printf("\n------------------------------------\n");
        printf("Print the mean error of each E/H method by using the inlier points.\n");
        for (int i = 0; i < num_solutions; i++)
        {

            printf("\n---------------\n");
            printf("Solution %d, num inliers = %d \n", i, (int)list_matches[i].size());
            print_R_t(list_R[i], list_t[i]);
            if (i >= 1)
                cout << "norm is:" << (list_normal[i]).t() << endl;
            printf("-- Epipolar cons error = %f \n", list_error_epipolar[i]);
            printf("-- Triangulation error = %f \n", list_error_triangulation[i]);
        }
    }
}
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// ----------- debug functions --------------------------------------------------
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------

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

// Check [Epipoloar error] and [Triangulation result] for each common inlier in both E and H
void print_EpipolarError_and_TriangulationResult_By_Common_Inlier(
    const vector<Point2f> &pts_img1, const vector<Point2f> &pts_img2,
    const vector<Point2f> &pts_on_np1, const vector<Point2f> &pts_on_np2,
    const vector<vector<Point3f>> &sols_pts3d_in_cam1,
    const vector<vector<int>> &list_inliers,
    const vector<Mat> &list_R, const vector<Mat> &list_t, const Mat &K)
{
    const int MAX_TO_CHECK_AND_PRINT = 1000;
    int num_solutions = list_R.size();
    const vector<int> &inliers_index_e = list_inliers[0];
    const vector<int> &inliers_index_h = list_inliers[1];

    cout << "\n---------------------------------------" << endl;
    cout << "Check [Epipoloar error] and [Triangulation result]" << endl;
    cout << "for the first " << MAX_TO_CHECK_AND_PRINT << " points:";

    // Iterate through points.
    int cnt = 0;
    int num_points = pts_img1.size();
    for (int i = 0; i < num_points && cnt < MAX_TO_CHECK_AND_PRINT; i++)
    {
        auto pe = find(inliers_index_e.begin(), inliers_index_e.end(), i);
        auto ph = find(inliers_index_h.begin(), inliers_index_h.end(), i);
        if (pe == inliers_index_e.end() || ph == inliers_index_h.end())
            continue;
        int ith_in_e_inliers = pe - inliers_index_e.begin();
        int ith_in_h_inliers = ph - inliers_index_h.begin();
        cout << "\n--------------" << endl;
        printf("Printing the %dth (in common) and %dth (in matched) point's real position in image:\n", cnt++, i);

        // Print point pos in image frame.
        Point2f p1 = pts_img1[i], p2 = pts_img2[i];
        cout << "cam1, pixel pos (u,v): " << p1 << endl;
        cout << "cam2, pixel pos (u,v): " << p2 << endl;

        // Print result of each method.
        Point2f p_cam1 = pts_on_np1[i]; // point pos on the normalized plane
        Point2f p_cam2 = pts_on_np2[i];
        for (int j = 0; j < num_solutions; j++)
        {
            const Mat &R = list_R[j], &t = list_t[j];

            // print epipolar error
            double err_epipolar = computeEpipolarConsError(p1, p2, R, t, K);
            cout << "===solu " << j << ": epipolar error is " << err_epipolar * 1e6 << endl;

            // print triangulation result
            int ith_in_curr_sol;
            if (j == 0)
                ith_in_curr_sol = ith_in_e_inliers;
            else
                ith_in_curr_sol = ith_in_h_inliers;

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

// Print each solution's result in order
void print_EpipolarError_and_TriangulationResult_By_Solution(
    const vector<Point2f> &pts_img1, const vector<Point2f> &pts_img2,
    const vector<Point2f> &pts_on_np1, const vector<Point2f> &pts_on_np2,
    const vector<vector<Point3f>> &sols_pts3d_in_cam1,
    const vector<vector<int>> &list_inliers,
    const vector<Mat> &list_R, const vector<Mat> &list_t, const Mat &K)
{
    cout << "\n---------------------------------------" << endl;
    cout << "Check [Epipoloar error] and [Triangulation result]" << endl;
    cout << "for each solution" << endl;
    int num_solutions = list_R.size();
    for (int j = 0; j < num_solutions; j++)
    {
        const Mat &R = list_R[j], &t = list_t[j];
        const vector<int> &inliers = list_inliers[j];
        int num_inliers = inliers.size();
        for (int idx_inlier = 0; idx_inlier < num_inliers; idx_inlier++)
        {
            int idx_pts = inliers[idx_inlier];
            cout << "\n--------------" << endl;
            printf("Printing the %dth inlier point in solution %d\n", idx_inlier, j);
            printf("which is %dth keypoint\n", idx_pts);

            // Print point pos in image frame.
            Point2f p1 = pts_img1[idx_pts], p2 = pts_img2[idx_pts];
            cout << "cam1, pixel pos (u,v): " << p1 << endl;
            cout << "cam2, pixel pos (u,v): " << p2 << endl;

            // print epipolar error
            double err_epipolar = computeEpipolarConsError(p1, p2, R, t, K);
            cout << "===solu " << j << ": epipolar error is " << err_epipolar * 1e6 << endl;

            // print triangulation result
            Mat pts3dc1 = Point3f_to_Mat(sols_pts3d_in_cam1[j][idx_inlier]); // 3d pos in camera 1
            Mat pts3dc2 = R * pts3dc1 + t;
            Point2f pts2dc1 = cam2pixel(pts3dc1, K);
            Point2f pts2dc2 = cam2pixel(pts3dc2, K);

            cout << "-- In img1, pos: " << pts2dc1 << endl;
            cout << "-- In img2, pos: " << pts2dc2 << endl;
            cout << "-- On cam1, pos: " << pts3dc1.t() << endl;
            cout << "-- On cam2, pos: " << pts3dc2.t() << endl;
        }
    }
}

} // namespace my_geometry