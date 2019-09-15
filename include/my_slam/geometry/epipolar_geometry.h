
#ifndef MY_SLAM_EPIPOLAR_GEOMETRY_H
#define MY_SLAM_EPIPOLAR_GEOMETRY_H

#include "my_slam/common_include.h"

#include "my_slam/basics/opencv_funcs.h"
#include "my_slam/geometry/camera.h" // transformations related to camera

namespace my_slam
{
namespace geometry
{

// -------------------------------------------------------------------------------------------
// ---------------- Main: Motion from Essential and Homography; Triangulation ----------------
// -------------------------------------------------------------------------------------------

/* @brief Estimate camera motion by using Essential matrix.
 */
void estiMotionByEssential(
    const vector<cv::Point2f> &pts_in_img1, const vector<cv::Point2f> &pts_in_img2,
    const cv::Mat &camera_intrinsics,
    cv::Mat &essential_matrix,
    cv::Mat &R, cv::Mat &t,    // R_curr_to_prev, t_curr_to_prev
    vector<int> &inliers_index // the inliers used in estimating Essential
);

/* @brief Estimate camera motion by using Homography matrix.
 *      There might be 1 or 2 possible Homography matrices. Return them all. 
 */
void estiMotionByHomography(
    const vector<cv::Point2f> &pts_in_img1, const vector<cv::Point2f> &pts_in_img2,
    const cv::Mat &camera_intrinsics,
    cv::Mat &homography_matrix,
    vector<cv::Mat> &Rs, vector<cv::Mat> &ts, // R_curr_to_prev, t_curr_to_prev
    vector<cv::Mat> &normals,                 // Homography plane's normal
    vector<int> &inliers_index                // The inliers used for estimating Homography
);

/* @brief Remove wrong solutions (R&t pairs) of homography.
 *      Input 4 solutions; output 1 or 2 solutions.
 * @param: Rs, length 4
 * @param: ts, length 4
 * @param: normals, length 4
 * @return: Rs, length 1 or 2
 * @return: ts, length 1 or 2
 * @return: normals, length 1 or 2
 */
void removeWrongRtOfHomography(
    const vector<cv::Point2f> &pts_on_np1, const vector<cv::Point2f> &pts_on_np2,
    const vector<int> &inliers,
    vector<cv::Mat> &Rs, vector<cv::Mat> &ts, vector<cv::Mat> &normals);

/* @brief Triangulate points.
 * @return: pts3d_in_cam1: points 3d position in current frame.
 */
void doTriangulation(
    const vector<cv::Point2f> &pts_on_np1,
    const vector<cv::Point2f> &pts_on_np2,
    const cv::Mat &R_cam2_to_cam1, const cv::Mat &t_cam2_to_cam1,
    const vector<int> &inliers,
    vector<cv::Point3f> &pts3d_in_cam1);

// ----------------------------------------------------------
// ---------------- Validation (Print error) ----------------
// ----------------------------------------------------------

double computeEpipolarConsError(
    const cv::Point2f &p1, const cv::Point2f &p2,
    const cv::Mat &R, const cv::Mat &t, const cv::Mat &K);

double computeEpipolarConsError( // mean square error of a vector of points
    const vector<cv::Point2f> &pts1, const vector<cv::Point2f> &pts2,
    const cv::Mat &R, const cv::Mat &t, const cv::Mat &K);

inline double calcErrorSquare(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

// -----------------------------------------------------
// ---------------- Datatype conversion ----------------
// -----------------------------------------------------

vector<cv::Point2f> convertkeypointsToPoint2f(const vector<cv::KeyPoint> kpts);

vector<cv::Point2f> getInlierPts(
    const vector<cv::Point2f> &pts,
    const vector<int> &inliers_idx);

vector<cv::KeyPoint> getInlierKpts(
    const vector<cv::KeyPoint> &kpts,
    const vector<int> &inliers_idx);

void extractPtsFromMatches(
    const vector<cv::Point2f> &points_1, const vector<cv::Point2f> &points_2,
    const vector<cv::DMatch> &matches,
    vector<cv::Point2f> &pts1, vector<cv::Point2f> &pts2);

void extractPtsFromMatches(
    const vector<cv::KeyPoint> &keypoints_1, const vector<cv::KeyPoint> &keypoints_2,
    const vector<cv::DMatch> &matches,
    vector<cv::Point2f> &pts1, vector<cv::Point2f> &pts2);

// ----------------------------------------------------------------------------
// ---------------- Other assistant functions ----------------
// ----------------------------------------------------------------------------

/* Compute a point's pos on both cam1 and cam2's normalized plane.
 */
double ptPosInNormPlane(const cv::Point3f &pt_3d_pos_in_cam1,
                        const cv::Mat &R_cam2_to_cam1, const cv::Mat &t_cam2_to_cam1,
                        cv::Point2f &pt_pos_on_cam1_nplane, double &depth1,
                        cv::Point2f &pt_pos_on_cam2_nplane, double &depth2);

} // namespace geometry
} // namespace my_slam

#endif