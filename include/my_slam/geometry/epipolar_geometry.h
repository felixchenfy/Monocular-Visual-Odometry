
#ifndef EPIPOLAR_GEOMETRY_H
#define EPIPOLAR_GEOMETRY_H

#include "my_slam/geometry/common_include.h"

#include "my_slam/basics/opencv_funcs.h"
#include "my_slam/geometry/camera.h" // transformations related to camera


namespace geometry
{
using namespace std;
using namespace cv;
using namespace basics;

// ---------------- main: essential, homography, triangulation ----------------
void estiMotionByEssential(
    const vector<Point2f> &pts_in_img1, const vector<Point2f> &pts_in_img2,
    const Mat &camera_intrinsics,
    Mat &essential_matrix,
    Mat &R, Mat &t,
    vector<int> &inliers_index); // the inliers used in estimating Essential

void estiMotionByHomography(
    const vector<Point2f> &pts_in_img1,  const vector<Point2f> &pts_in_img2,
    const Mat &camera_intrinsics,
    Mat &homography_matrix,
    vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> &normals, // Might have 1 or 2 solutions. Return by vector.
    vector<int> &inliers_index); // the inliers used for estimating Homography

// Remove wrong solutions of R,t.
// 4 solutions --> 1 or 2 solutions
void removeWrongRtOfHomography(
    const vector<Point2f> &pts_on_np1, const vector<Point2f> &pts_on_np2,
    const vector<int> &inliers,
    vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> &normals);
    
void doTriangulation(
    const vector<Point2f> &pts_on_np1,
    const vector<Point2f> &pts_on_np2,
    const Mat &R_cam2_to_cam1, const Mat &t_cam2_to_cam1,
    const vector<int> &inliers,
    vector<Point3f> &pts3d_in_cam1); // points real pos in camera1's frame


// ---------------- validation ----------------
double computeEpipolarConsError(
    const Point2f &p1, const Point2f &p2, const Mat &R, const Mat &t, const Mat &K);
double computeEpipolarConsError( // mean square error of a vector of points
    const vector<Point2f> &pts1, const vector<Point2f> &pts2,
    const Mat &R, const Mat &t, const Mat &K);
double calcErrorSquare(const Point2f &p1, const Point2f &p2);

// ---------------- datatype conversion ----------------

vector<Point2f> convertKeypointsToPoint2f(const vector<KeyPoint> kpts);

vector<Point2f> getInlierPts(
    const vector<Point2f> &pts,
    const vector<int> &inliers_idx);

vector<KeyPoint> getInlierKpts(
    const vector<KeyPoint> &kpts,
    const vector<int> &inliers_idx);

void extractPtsFromMatches(
    const vector<Point2f> &points_1, const vector<Point2f> &points_2,
    const vector<DMatch> &matches,
    vector<Point2f> &pts1, vector<Point2f> &pts2);

void extractPtsFromMatches(
    const vector<KeyPoint> &keypoints_1, const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    vector<Point2f> &pts1, vector<Point2f> &pts2);
// ---------------- assist ----------------
// compute a point's pos on both cam1 and cam2's normalized plane
double ptPosInNormPlane(const Point3f &pt_3d_pos_in_cam1,
    const Mat &R_cam2_to_cam1, const Mat &t_cam2_to_cam1,
    Point2f &pt_pos_on_cam1_nplane, double &depth1,
    Point2f &pt_pos_on_cam2_nplane, double &depth2);

} // namespace geometry
#endif