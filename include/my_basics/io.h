
#ifndef IO_H
#define IO_H

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>


namespace my_basics
{
using namespace std;

// -- Read in from config file: image paths, camera intrinsics
vector<string> readImagePaths(const string &path_of_config_file, bool print_res=false);
cv::Mat readCameraIntrinsics(const string &path_of_config_file, bool print_res=false);

// -- Read/Write camera pose to file
void writePoseToFile(const string filename, vector<cv::Mat> list_T); // Format: x, y, z, 1st row of R, 2nd row of R, 3rd row of R
vector<cv::Mat> readPoseFromFile(const string filename); // Read pose from file

} // namespace my_basics

#endif