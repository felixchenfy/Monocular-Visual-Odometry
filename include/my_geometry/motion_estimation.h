
#ifndef MOTION_ESTIMATION_H
#define MOTION_ESTIMATION_H

#include "my_geometry/common_include.h"

#include "my_geometry/camera.h"
#include "my_geometry/feature_match.h"
#include "my_geometry/epipolar_geometry.h"
#include "my_basics/opencv_funcs.h"

using namespace my_basics;

namespace my_geometry
{
void helperEstimatePossibleRelativePosesByEpipolarGeometry(
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    const Mat &K,
    vector<Mat> &list_R, vector<Mat> &list_t,
    vector<vector<DMatch>> &list_matches,
    vector<Mat> &list_normal,
    vector<vector<Point3f>> &sols_pts3d_in_cam1,
    const bool print_res = false);

double helperEvaluateEstimationsError(
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<vector<DMatch>> &list_matches,
    const vector<vector<Point3f>> &sols_pts3d_in_cam1_by_triang,
    const vector<Mat> &list_R, const vector<Mat> &list_t, const vector<Mat> &list_normal,
    const Mat &K,
    vector<double> &list_error_epipolar,
    vector<double> &list_error_triangulation,// the error on the normalized image plane
    bool print_res);

void helperEstiMotionByEssential(
    const vector<KeyPoint> &keypoints_1,
    const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    const Mat &K,
    Mat &R, Mat &t, vector<int> &inliers_of_matches,
    const bool print_res = false);

// void helperDoTriangulation(
//     const vector<KeyPoint> &keypoints_1,
//     const vector<KeyPoint> &keypoints_2,
//     const vector<DMatch> &matches,
//     const Mat &K,
//     Mat &R, Mat &t, vector<int> &inliers_in_kpts2,
//     const bool print_res = false);

// Get the 3d-2d corrsponding points
void helperFind3Dto2DCorrespondences( 
    const vector<DMatch> &curr_dmatch, const vector<KeyPoint> &curr_kpts,
    const vector<int> &prev_inliers_of_all_pts, const vector<Point3f> &prev_inliers_pts3d,
    vector<Point3f> &pts_3d, vector<Point2f> &pts_2d);

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

} // namespace my_geometry
#endif