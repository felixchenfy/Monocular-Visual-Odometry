

// std
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// my
#include "my_basics/io.h"
#include "my_basics/config.h"
#include "my_geometry/feature_match.h"
#include "my_geometry/epipolar_geometry.h"

using namespace std;
using namespace cv;
using namespace my_geometry;

bool checkInputArguments(int argc, char** argv);
int main(int argc, char **argv)
{
    // Read in image filenames and camera prameters.
    assert( checkInputArguments(argc, argv) );
    const string config_file = argv[1];
    bool print_res=false;
    vector<string> image_paths=my_basics::readImagePaths(config_file, print_res);
    cv::Mat K = my_basics::readCameraIntrinsics(config_file);
    my_basics::Config::setParameterFile( // just to remind to set up this config file.
        config_file);  // Following algorithms will read from it for setting params.

    // read in image
    for (int img_id = 0; img_id < (int)image_paths.size(); img_id++){
        
        // Read image.
        cv::Mat img = cv::imread ( image_paths[img_id] );
        if ( img.data==nullptr )
            break;
        printf("Start processing the %dth image.\n", img_id);

        // Extract keypoints and descriptors      
        vector<KeyPoint> keypoints;
        vector<DMatch> matches;
        Mat descriptors;  
        extractKeyPoints(img, keypoints);
        computeDescriptors(img, keypoints, descriptors);
        if (img_id==0){
            continue;
        }
        
        // Display
        // cv::Mat img_show=img.clone();
        // cv::Scalar color(0,255,0);
        // cv::Scalar color2= cv::Scalar::all(-1);
        // cv::drawKeypoints(img_show, vo->getCurrentKeypoints(), img_show, color);
        // cv::imshow ( "img", img_show );
        // cv::waitKey (1000);
        if(img_id==2)break;
    }
}


bool checkInputArguments(int argc, char** argv){
    // The only argument is Path to the configuration file, which stores the dataset_dir and camera_info
    const int NUM_ARGUMENTS = 1;
    if (argc - 1 != NUM_ARGUMENTS)
    {
        cout << "Lack arguments: Please input the path to the .yaml config file" << endl;
        return false;
    }
    return true;
}