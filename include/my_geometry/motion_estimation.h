
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
    const Mat &descriptors_1,
    const Mat &descriptors_2,
    const Mat &K,
    vector<Mat> &list_R, vector<Mat> &list_t,
    vector<vector<DMatch>> &list_matches,
    vector<Mat> &list_normal,
    vector<vector<Point3f>> &sols_pts3d_in_cam1);

// ---------------------------------------
// ---------------------------------------
// ----------- debug functions -----------
// ---------------------------------------
// ---------------------------------------

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

// Check [Epipoloar error] and [Triangulation result] for each feature point
void print_EpipolarError_and_TriangulationResult(
    vector<Point2f> pts_img1, vector<Point2f> pts_img2, vector<Point2f> pts_on_np1, vector<Point2f> pts_on_np2,
    vector<vector<Point3f>> sols_pts3d_in_cam1,
    vector<vector<int>> list_inliers,
    vector<Mat> list_R, vector<Mat> list_t,
    Mat K);
} // namespace my_geometry
#endif