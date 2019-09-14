/* @brief Read data from file, or write data to file.
 */

#ifndef MY_SLAM_IO_H
#define MY_SLAM_IO_H

#include "my_slam/common_include.h"

namespace my_slam
{
namespace basics
{

// Read image paths from config_file by the key_dataset_dir and key_num_images.
// The image should be named as
vector<string> readImagePaths(
    const string &config_file, bool print_res,
    const string &key_dataset_dir = "dataset_dir",
    const string &key_num_images = "num_images",
    const string &image_formatting = "/rgb_%05d.png");

// Read image paths from config_file by the keys:
//      camera_info.fx, camera_info.fy, camera_info.cx, camera_info.cy
cv::Mat readCameraIntrinsics(const string &config_file, bool print_res = false);

// Write camera pose to file.
//      Numbers of each row: x, y, z, 1st row of R, 2nd row of R, 3rd row of R
void writePoseToFile(const string filename, vector<cv::Mat> list_T);

// Read pose from file
//      Numbers of each row: x, y, z, 1st row of R, 2nd row of R, 3rd row of R
vector<cv::Mat> readPoseFromFile(const string filename);

} // namespace basics
} // namespace my_slam

#endif