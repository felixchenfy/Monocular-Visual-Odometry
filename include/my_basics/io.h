
#ifndef IO_H
#define IO_H

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>


namespace my_basics
{
using namespace std;

// Read in all image paths
vector<string> readImagePaths(const string &path_of_config_file, bool print_res=false);
cv::Mat readCameraIntrinsics(const string &path_of_config_file,bool print_res=false);

} // namespace my_basics

#endif