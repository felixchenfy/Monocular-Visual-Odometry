/* @brief Read data from file, or write data to file.
 */

#ifndef MY_SLAM_IO_H
#define MY_SLAM_IO_H

#include "my_slam/common_include.h"
#include "my_slam/basics/yaml.h"

namespace my_slam
{
namespace vo
{

// Read image paths from config_file by the key_dataset_dir and key_num_images.
// The image should be named as
vector<string> readImagePaths(
    const string &dataset_dir,
    int num_images,
    const string &image_formatting,
    bool is_print_res);

// Read image paths from config_file by the keys:
//      camera_info.fx, camera_info.fy, camera_info.cx, camera_info.cy
cv::Mat readCameraIntrinsics(const basics::Yaml &config, bool is_print_res = true);

// Write camera pose to file.
//      Numbers of each row: x, y, z, 1st row of R, 2nd row of R, 3rd row of R
void writePoseToFile(const string filename, vector<cv::Mat> list_T);

// Read pose from file
//      Numbers of each row: x, y, z, 1st row of R, 2nd row of R, 3rd row of R
vector<cv::Mat> readPoseFromFile(const string filename);

} // namespace basics
} // namespace my_slam

#endif