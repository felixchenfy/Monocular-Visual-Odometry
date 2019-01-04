

// std
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <deque>

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// my
#include "my_basics/io.h"
#include "my_basics/config.h"
#include "my_geometry/feature_match.h"
#include "my_geometry/epipolar_geometry.h"
#include "my_slam/frame.h"

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
    cv::Mat K = my_basics::readCameraIntrinsics(config_file); // camera intrinsics
    my_geometry::Camera::Ptr camera( // init a camera class with common transformations
        new my_geometry::Camera(K)); 
    my_basics::Config::setParameterFile( // just to remind to set this config file.
        config_file);  // Following algorithms will read from it for setting params.

    // read in image
    deque<my_slam::Frame::Ptr> frames;
    for (int img_id = 0; img_id < (int)image_paths.size(); img_id++){
        
        // Read image.
        cv::Mat rgb_img = cv::imread ( image_paths[img_id] );
        if ( rgb_img.data==nullptr )
            break;
        printf("\n\n----------------------------------------------------\n");
        printf("Start processing the %dth image.\n", img_id);

        // Init a frame. Extract keypoints and descriptors.
        my_slam::Frame::Ptr frame=my_slam::Frame::createFrame(rgb_img, camera); 
        frame->extractKeyPoints();
        frame->computeDescriptors();

        // Start vo.
        vector<KeyPoint> keypoints;
        vector<DMatch> matches;
        Mat descriptors; 

        if (img_id==0){ // initiliazation stage
        }else{
            my_slam::Frame::Ptr prev_frame=frames.back();
            frame->matchFeatures(prev_frame);
        }
        // Display
        // cv::Mat img_show=rgb_img.clone();
        // cv::Scalar color(0,255,0);
        // cv::Scalar color2= cv::Scalar::all(-1);
        // cv::drawKeypoints(img_show, vo->getCurrentKeypoints(), img_show, color);
        // cv::imshow ( "rgb_img", img_show );
        // cv::waitKey (1000);


        // Save to buff.
        frames.push_back(frame); 
        if (frames.size()>10)frames.pop_front();
        
        // Return
        if(img_id==5)break;
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