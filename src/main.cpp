
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <boost/format.hpp> // for setting image filename

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/camera.h"
#include "myslam/frame.h"

// const int FRAME_RATE=30;

// Read in all image paths
vector<string> getImagePaths()
{
    //  Input: path of the configuration file, which stores the dataset(images) folder
    //  Output: image_paths
    vector<string> image_paths;

    // Set up image_paths
    string dataset_dir = myslam::Config::get<string>("dataset_dir"); // get dataset_dir from config
    boost::format filename_fmt(dataset_dir + "/rgb_%05d.png");
    for (int i = 0; i < 5; i++)
    {
        image_paths.push_back((filename_fmt % i).str());
    }

    // Print result
    cout << endl;
    cout << "Reading from dataset_dir: " << dataset_dir << endl;
    cout << "Number of images: " << image_paths.size() << endl;
    cout << "Print the first 5 image filenames:" << endl;
    for (auto s : image_paths)
        cout << s << endl;
    cout << endl;

    // Return
    return image_paths;
}

myslam::Camera::Ptr setupCamera(){
    myslam::Camera::Ptr camera(
        new myslam::Camera(
            myslam::Config::get<double>("camera_info.fx"),
            myslam::Config::get<double>("camera_info.fy"),
            myslam::Config::get<double>("camera_info.cx"),
            myslam::Config::get<double>("camera_info.cy")
        )
    );
    return camera;
}

int main(int argc, char **argv)
{
    
    // -----------------------------------------------------------
    // Check input arguments:
    //      Path to the configuration file, which stores the dataset_dir and camera_info
    const int NUM_ARGUMENTS = 1;
    if (argc - 1 != NUM_ARGUMENTS)
    {
        cout << "Lack arguments: Please input the path to the .yaml config file" << endl;
        return 1;
    }
    const string path_of_config_file = argv[1];
    myslam::Config::setParameterFile(path_of_config_file);

    // -----------------------------------------------------------
    
    vector<string> image_paths=getImagePaths();

    myslam::Camera::Ptr camera=setupCamera();
        
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    for (int i = 0; i< image_paths.size(); i++){
        
        // Read image.
        cv::Mat rgb_img = cv::imread ( image_paths[i] );
        if ( rgb_img.data==nullptr )
            break;

        // Add to map        
        myslam::Frame::Ptr frame( new myslam::Frame(rgb_img, camera) );
        // vo.addFrame(pFrame)

        // Display
        cv::Mat img_show=rgb_img.clone();
        cv::imshow ( "image", img_show );
        cv::waitKey (1000);
    }
}