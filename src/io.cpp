

#include <iostream>
#include <boost/format.hpp> // for setting image filename
#include <fstream>

#include "my_slam/basics/io.h"
#include "my_slam/basics/config.h"

using namespace std;

namespace my_slam
{
namespace basics
{

vector<string> readImagePaths(
    const string &config_file, bool print_res, 
    const string &key_dataset_dir, 
    const string &key_num_images,
    const string &image_formatting)
{
    basics::Config::setParameterFile(config_file);

    // Set up image_paths
    vector<string> image_paths;
    string dataset_dir = basics::Config::get<string>(key_dataset_dir);
    int num_images = basics::Config::get<int>(key_num_images);
    boost::format filename_fmt(dataset_dir + image_formatting);
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
    return image_paths;
}

cv::Mat readCameraIntrinsics(const string &config_file, bool print_res)
{
    basics::Config::setParameterFile(config_file);
    double fx = basics::Config::get<double>("camera_info.fx");
    double fy = basics::Config::get<double>("camera_info.fy");
    double cx = basics::Config::get<double>("camera_info.cx");
    double cy = basics::Config::get<double>("camera_info.cy");
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    return K;
}


void writePoseToFile(const string filename, vector<cv::Mat> list_T)
{
    ofstream fout;
    fout.open(filename);
    if (!fout.is_open())
    {
        cout << "my WARNING: failed to store camera trajectory to the wrong file name of:" << endl;
        cout << "    " << filename << endl;
        return;
    }
    for (auto T : list_T)
    {
        double x = T.at<double>(0, 3);
        double y = T.at<double>(1, 3);
        double z = T.at<double>(2, 3);
        fout << x << " " << y << " " << z << " ";
        for (int i = 0; i < 3; i++) // order: 1st column, 2nd column, 3rd column
        {
            for (int j = 0; j < 3; j++)
            {
                fout << T.at<double>(j, i) << " ";
            }
        }
        fout << '\n';
    }
    fout.close();
}

vector<cv::Mat> readPoseFromFile(const string filename)
{
    // Output
    vector<cv::Mat> list_T;
    vector<vector<double>> list_pose; // store, but not output

    // Open file
    ifstream fin;
    fin.open(filename);
    assert(fin.is_open()); // Fail to find the config file

    // Read data
    const int NUM_IN_ROW = 12; // x,y,z, 1st column of R, 2nd column of R, 3rd column of R
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
            // cv::Mat T = (cv::Mat_<double>(4, 4) << pose[3], pose[4], pose[5], pose[0],
            //              pose[6], pose[7], pose[8], pose[1],
            //              pose[9], pose[10], pose[11], pose[2],
            //              0, 0, 0, 1);
            cv::Mat T = (cv::Mat_<double>(4, 4) << pose[3], pose[6], pose[9], pose[0],
                         pose[4], pose[7], pose[10], pose[1],
                         pose[5], pose[8], pose[11], pose[2],
                         0, 0, 0, 1);
            list_T.push_back(T);
        }
    }

    // Return
    fin.close();
    return list_T;
}

} // namespace basics
} // namespace my_slam