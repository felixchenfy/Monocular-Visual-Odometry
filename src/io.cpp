

#include <iostream>
#include <boost/format.hpp> // for setting image filename

#include "my_basics/io.h"
#include "my_basics/config.h"

using namespace std;

namespace my_basics
{

// Read in all image paths
vector<string> readImagePaths(const string &path_of_config_file, int NUMBER_OF_IMAGES, bool print_res)
{
    my_basics::Config::setParameterFile(path_of_config_file);

    // Set up image_paths
    vector<string> image_paths;
    string dataset_dir = my_basics::Config::get<string>("dataset_dir"); // get dataset_dir from config
    boost::format filename_fmt(dataset_dir + "/rgb_%05d.png");
    for (int i = 0; i < NUMBER_OF_IMAGES; i++)
    {
        image_paths.push_back((filename_fmt % i).str());
    }

    // Print result
    if (print_res)
    {
        cout << endl;
        cout << "Reading from dataset_dir: " << dataset_dir << endl;
        cout << "Number of images: " << image_paths.size() << endl;
        cout << "Print the first 5 image filenames:" << endl;
        for (auto s : image_paths)
            cout << s << endl;
        cout << endl;
    }
    // Return
    return image_paths;
}

cv::Mat readCameraIntrinsics(const string &path_of_config_file, bool print_res)
{
    my_basics::Config::setParameterFile(path_of_config_file);
    double fx = my_basics::Config::get<double>("camera_info.fx");
    double fy = my_basics::Config::get<double>("camera_info.fy");
    double cx = my_basics::Config::get<double>("camera_info.cx");
    double cy = my_basics::Config::get<double>("camera_info.cy");
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    return K;
}

} // namespace my_basics