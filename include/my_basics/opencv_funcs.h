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

using namespace std;
using namespace cv;

namespace my_basics
{

// ---------------- Math ----------------
Mat skew(const Mat &t); // 3x1 vec to 3x3 skew symmetric matrix
Mat transRt2T(const Mat &R, const Mat &t);

// ---------------- Print ----------------
void print_MatProperty(cv::Mat &M); // print data type and size
void print_R_t(const Mat &R, const Mat &t);
string cvMatType2str(int cvMatType);

}
#endif