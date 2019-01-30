

#include <iostream>
#include <boost/format.hpp> // for setting image filename
#include <fstream>

#include "my_basics/io.h"
#include "my_basics/config.h"

using namespace std;

namespace my_basics
{

// -- Read in from config file: image paths, camera intrinsics

// Read image paths
vector<string> readImagePaths(const string &path_of_config_file, bool print_res)
{
    my_basics::Config::setParameterFile(path_of_config_file);

    // Set up image_paths
    vector<string> image_paths;
    string dataset_dir = my_basics::Config::get<string>("dataset_dir"); // get dataset_dir from config
    int num_images = my_basics::Config::get<int>("num_images"); 
    boost::format filename_fmt(dataset_dir + "/rgb_%05d.png");
    for (int i = 0; i < num_images; i++)
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

// Read camera intrinsics
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

// -- Read/Write camera pose to file


// Write pose to file: x, y, z, 1st row of R, 2nd row of R, 3rd row of R
void writePoseToFile(const string filename, vector<cv::Mat> list_T)
{
    ofstream fout;
    fout.open(filename);
    for (auto T : list_T)
    {
        double x = T.at<double>(0, 3);
        double y = T.at<double>(1, 3);
        double z = T.at<double>(2, 3);
        fout << x << " " << y << " " << z << " ";
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                fout << T.at<double>(i, j) << " ";
            }
        }
        fout << '\n';
    }
    fout.close();
}

// Read pose from file
vector<cv::Mat> readPoseToFile(const string filename)
{
    // Output
    vector<cv::Mat> list_T;
    vector<vector<double>> list_pose; // store, but not output

    // Open file
    ifstream fin;
    fin.open(filename);
    assert(fin.is_open()); // Fail to find the config file

    // Read data
    const int NUM_IN_ROW = 12; // x,y,z, 1st row of R, 2nd row of R, 3rd row of R
    vector<double> pose(NUM_IN_ROW, 0.0);
    double val;
    int cnt = 0;
    while (fin >> val)
    {
        pose[cnt++] = val;
        if (cnt == NUM_IN_ROW)
        {
            cnt = 0;
            list_pose.push_back(pose);
            cv::Mat T = (cv::Mat_<double>(4, 4) << pose[3], pose[4], pose[5], pose[0],
                         pose[6], pose[7], pose[8], pose[1],
                         pose[9], pose[10], pose[11], pose[2],
                         0, 0, 0, 1);
            list_T.push_back(T);
        }
    }

    // Return
    fin.close();
    return list_T;
}

} // namespace my_basics