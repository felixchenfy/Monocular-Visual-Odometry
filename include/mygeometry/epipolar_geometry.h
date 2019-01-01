
#ifndef EPIPOLAR_GEOMETRY_H
#define EPIPOLAR_GEOMETRY_H

#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

namespace mygeometry
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
    vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> normals, // Might have 1 or 2 solutions. Return by vector.
    /*optional*/ vector<int> &inliers_index, // the inliers used for estimating Homography
    vector<Point2f> &inlier_pts_in_img1, vector<Point2f> &inlier_pts_in_img2);

void estiMotionByHomography(
    const vector<Point2f> &pts_in_img1, const vector<Point2f> &pts_in_img2,
    const Mat &camera_intrinsics,
    vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> normals);

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

// ---------------- assist ----------------
void extractPtsFromMatches(
    const vector<KeyPoint> &keypoints_1, const vector<KeyPoint> &keypoints_2,
    const vector<DMatch> &matches,
    vector<Point2f> &pts1, vector<Point2f> &pts2);

// ---------------- transformation ----------------
Point2f pixel2camNormPlane(const Point2f &p, const Mat &K);
Point3f pixel2cam(const Point2f &p, const Mat &K, double depth=1);


// ---------------- Math ----------------
Mat skew(const Mat &t); // 3x1 vec to 3x3 skew symmetric matrix
Mat transRt2T(const Mat &R, const Mat &t);

// ---------------- print ----------------
string cvMatType2str(int cvMatType);
void print_MatProperty(cv::Mat &M); // print data type and size
void print_R_t(const Mat &R, const Mat &t);


} // namespace mygeometry
#endif