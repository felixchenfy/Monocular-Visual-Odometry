
#ifndef EPIPOLAR_GEOMETRY_H
#define EPIPOLAR_GEOMETRY_H

#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "my_common/opencv_funcs.h"

using namespace std;
using namespace cv;
using namespace my;

namespace my_geometry
{

// ---------------- main: essential, homography, triangulation ----------------
void estiMotionByEssential(
    const vector<Point2f> &pts_in_img1, const vector<Point2f> &pts_in_img2,
    const Mat &camera_intrinsics,
    Mat &R, Mat &t,
    /*optional*/ vector<int> &inliers_index, // the inliers used for estimating Essential
    vector<Point2f> &inlier_pts_in_img1, vector<Point2f> &inlier_pts_in_img2);

void estiMotionByEssential(
    const vector<Point2f> &pts_in_img1, const vector<Point2f> &pts_in_img2,
    const Mat &camera_intrinsics,
    Mat &R, Mat &t);

void estiMotionByHomography(
    const vector<Point2f> &pts_in_img1,  const vector<Point2f> &pts_in_img2,
    const Mat &camera_intrinsics,
    vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> &normals, // Might have 1 or 2 solutions. Return by vector.
    /*optional*/ vector<int> &inliers_index, // the inliers used for estimating Homography
    vector<Point2f> &inlier_pts_in_img1, vector<Point2f> &inlier_pts_in_img2);

void estiMotionByHomography(
    const vector<Point2f> &pts_in_img1, const vector<Point2f> &pts_in_img2,
    const Mat &camera_intrinsics,
    vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> &normals);

// Remove wrong solutions of R,t.
// 4 solutions --> 1 or 2 solutions
void removeWrongRtOfHomography(
    const vector<Point2f> &pts_in_img1, const vector<Point2f> &pts_in_img2,
    vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> &normals); 

void triangulation(
    const vector<Point2f> &pts_in_img1, const vector<Point2f> &pts_in_img2,
    const Mat &R_cam2_to_cam1, const Mat &t_cam2_to_cam1,
    const Mat &K,
    vector<Point3f> &pts3d_in_cam1); // points real pos in camera1's frame

// ---------------- validation ----------------
double computeEpipolarConsError(
    const Point2f &p1, const Point2f &p2, const Mat &R, const Mat &t, const Mat &K);
double computeEpipolarConsError( // mean square error of a vector of points
    const vector<Point2f> &pts1, const vector<Point2f> &pts2,
    const Mat &R, const Mat &t, const Mat &K);


// ---------------- transformation ----------------
Point2f pixel2camNormPlane(const Point2f &p, const Mat &K);
Point3f pixel2cam(const Point2f &p, const Mat &K, double depth=1);
Point2f cam2pixel(const Point3f &p, const Mat &K);
Point2f cam2pixel(const Mat &p, const Mat &K);


// ---------------- assist ----------------
Mat Point3f_to_Mat(const Point3f &p);
Mat Point2f_to_Mat(const Point2f &p);
double calcDist(const Point2f &p1, const Point2f &p2);
double calcError(const Point2f &p1, const Point2f &p2);
void extractPtsFromMatches(
    const vector<KeyPoint> &keypoints_1, const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    vector<Point2f> &pts1, vector<Point2f> &pts2);

vector<int> getIntersection(vector<int> v1, vector<int> v2); // please sort v1 and v2 first

// compute a point's pos on both cam1 and cam2's normalized plane
double ptPosInNormPlane(const Point3f &pt_3d_pos_in_cam1,
    const Mat &R_cam2_to_cam1, const Mat &t_cam2_to_cam1,
    Point2f &pt_pos_on_cam1_nplane, double &depth1,
    Point2f &pt_pos_on_cam2_nplane, double &depth2);

} // namespace my_geometry
#endif