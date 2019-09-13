
#ifndef MOTION_ESTIMATION_H
#define MOTION_ESTIMATION_H

#include "my_slam/geometry/common_include.h"

#include "my_slam/geometry/camera.h"
#include "my_slam/geometry/feature_match.h"
#include "my_slam/geometry/epipolar_geometry.h"
#include "my_slam/basics/opencv_funcs.h"


namespace geometry
{
using namespace basics;

// This is a giant function, which computes: E21/H21, all their decompositions, all corresponding triangulation results,
//      Then, choose between E/H based on ORB-SLAM2 paper. 
//      If choose H, then choose the one with largest norm z in camera direction.
int helperEstimatePossibleRelativePosesByEpipolarGeometry(
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    const Mat &K, // camera intrinsics
    vector<Mat> &list_R, vector<Mat> &list_t,
    vector<vector<DMatch>> &list_matches,
    vector<Mat> &list_normal,
    vector<vector<Point3f>> &sols_pts3d_in_cam1,
    const bool print_res = false,
    const bool compute_homography = true,
    const bool is_frame_cam2_to_cam1=true);

// Compute Eppipolar_Constraint and Triangulation error
void helperEvalEppiAndTriangErrors(
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<vector<DMatch>> &list_matches,
    const vector<vector<Point3f>> &sols_pts3d_in_cam1_by_triang,
    const vector<Mat> &list_R, const vector<Mat> &list_t, const vector<Mat> &list_normal,
    const Mat &K, // camera intrinsics
    bool print_res);

// Estimate camera motion by Essential matrix.
void helperEstiMotionByEssential( 
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    const Mat &K, // camera intrinsics
    Mat &R, Mat &t,
    vector<DMatch> &inlier_matches,
    const bool print_res=false);

// After feature matching, find inlier matches by using epipolar constraint to exclude wrong matches
vector<DMatch> helperFindInlierMatchesByEpipolarCons(
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    const Mat &K);

// Get the 3d-2d corrsponding points
void helperFind3Dto2DCorrespondences( 
    const vector<DMatch> &curr_inlier_matches, const vector<KeyPoint> &curr_kpts, 
    const vector<DMatch> &prev_inlier_matches, const vector<Point3f> &prev_inliers_pts3d,
    vector<Point3f> &pts_3d, vector<Point2f> &pts_2d);

// Triangulate points
vector<Point3f> helperTriangulatePoints(
    const vector<KeyPoint> &prev_kpts, const vector<KeyPoint> &curr_kpts,
    const vector<DMatch> &curr_inlier_matches,
    const Mat &T_curr_to_prev,
    const Mat &K
);
vector<Point3f> helperTriangulatePoints(
    const vector<KeyPoint> &prev_kpts, const vector<KeyPoint> &curr_kpts,
    const vector<DMatch> &curr_inlier_matches,
    const Mat &R_curr_to_prev, const Mat &t_curr_to_prev,
    const Mat &K
);

// Compute the score of estiamted E/H matrix by the method in ORB-SLAM
double checkEssentialScore(const Mat &E21, const Mat &K, const vector<Point2f> &pts_img1, const vector<Point2f> &pts_img2, 
    vector<int> &inliers_index, double sigma=1.0);
double checkHomographyScore(const Mat &H21,const vector<Point2f> &pts_img1, const vector<Point2f> &pts_img2, 
    vector<int> &inliers_index, double sigma=1.0);

// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// ----------- debug functions --------------------------------------------------
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------

void printResult_estiMotionByEssential(
    const Mat &essential_matrix,
    const vector<int> &inliers_index,
    const Mat &R,
    const Mat &t);

void printResult_estiMotionByHomography(
    const Mat &homography_matrix,
    const vector<int> &inliers_index,
    const vector<Mat> &Rs, const vector<Mat> &ts,
    vector<Mat> &normals);

void print_EpipolarError_and_TriangulationResult_By_Common_Inlier(
    const vector<Point2f> &pts_img1, const vector<Point2f> &pts_img2,
    const vector<Point2f> &pts_on_np1, const vector<Point2f> &pts_on_np2,
    const vector<vector<Point3f>> &sols_pts3d_in_cam1,
    const vector<vector<int>> &list_inliers,
    const vector<Mat> &list_R, const vector<Mat> &list_t, const Mat &K);

void print_EpipolarError_and_TriangulationResult_By_Solution(
    const vector<Point2f> &pts_img1, const vector<Point2f> &pts_img2,
    const vector<Point2f> &pts_on_np1, const vector<Point2f> &pts_on_np2,
    const vector<vector<Point3f>> &sols_pts3d_in_cam1,
    const vector<vector<int>> &list_inliers,
    const vector<Mat> &list_R, const vector<Mat> &list_t, const Mat &K);

} // namespace geometry
#endif