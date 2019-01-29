// some functions for
//  * Geometrical transformations.
//  * Printing out result.

#ifndef OPENCV_FUNCS_H
#define OPENCV_FUNCS_H

#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace my_basics
{
using namespace std;
using namespace cv;

// ---------------- image operation ----------------
vector<unsigned char> getPixelAt(const Mat &image, int x, int y);
unsigned char getPixelAt(const Mat &image, int row, int col, int idx_rgb);

// ---------------- datatype conversion ----------------
Mat Point3f_to_Mat(const Point3f &p);
Mat Point3f_to_Mat4x1(const Point3f &p);
Point3f Mat_to_Point3f(const Mat &p);
Mat Point2f_to_Mat(const Point2f &p);

// ---------------- Transformations ----------------
Point3f transCoord(const Point3f &p, const Mat &R, const Mat &t);
void invRt(Mat &R, Mat &t);
Mat transRt2T(const Mat &R, const Mat &t);
Mat transRt2T_3x4(const Mat &R, const Mat &t);
void getRtFromT(const Mat &T, Mat &R, Mat &t);
Mat getPosFromT(const Mat &T);
Point3f preTranslatePoint3f(const Point3f &p3x1, const Mat &T4x4);

// ---------------- Math ----------------
Mat skew(const Mat &t); // 3x1 vec to 3x3 skew symmetric matrix
double calcDist(const Point2f &p1, const Point2f &p2);
double calcMeanDepth(const vector<Point3f> &pts_3d);
double scalePointPos(Point3f &p, double scale);
double calcMatNorm(const Mat &mat);
void normalize(Mat &mat);

// ---------------- Print ----------------
void print_MatProperty(const cv::Mat &M); // print data type and size
void print_R_t(const Mat &R, const Mat &t);
string cvMatType2str(int cvMatType);

} // namespace my_basics
#endif