// Test PnP using a pair of depth images.
// (This script is basically imitating https://github.com/gaoxiang12/slambook/blob/master/ch7/pose_estimation_3d2d.cpp)

#include <iostream>
#include <algorithm> // std::min
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "my_geometry/feature_match.h"
#include "my_geometry/epipolar_geometry.h"
#include "my_geometry/motion_estimation.h"
#include "my_basics/config.h"

using namespace std;
using namespace cv;
using namespace my_geometry;

int main ( int argc, char** argv )
{

    cout << "program start" << endl;
    string folder="data/test_data/";
    string img_file1="fr1_1_1.png";
    string img_file2="fr1_1_2.png";
    string img_file3="fr1_1_1_depth.png";
    string img_file4="fr1_1_2_depth.png";  
    Mat K = (Mat_<double>(3, 3) << 517.3, 0, 325.1, 0, 516.5, 249.7, 0, 0, 1); // fr1 dataset
    Mat img_1 = imread (folder+ img_file1,  IMREAD_COLOR );
    Mat img_2 = imread (folder+ img_file2, IMREAD_COLOR );
    Mat d1 = imread ( folder+img_file3, IMREAD_UNCHANGED ); // 16 bit unsigned char, single channel

    // Extract keypoints and features. Match keypoints.
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    Mat descriptors_1, descriptors_2;
    string filename = "config/default.yaml";
    my_basics::Config::setParameterFile(filename);
    extractKeyPoints(img_1, keypoints_1); // Choose the config file before running this
    extractKeyPoints(img_2, keypoints_2);
    cout << "Number of keypoints: " << keypoints_1.size() << ", " << keypoints_2.size() << endl;
    computeDescriptors(img_1, keypoints_1, descriptors_1);
    computeDescriptors(img_2, keypoints_2, descriptors_2);
    matchFeatures(descriptors_1, descriptors_2, matches,
        true); // print result


    // Get points 3d and 2d correspondance
    vector<Point3f> pts_3d; // a point's 3d pos in cam1 frame
    vector<Point2f> pts_2d; // a point's 2d pos in image2 pixel frame
    for ( DMatch m:matches )
    {
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        if ( d == 0 )   // bad depth
            continue;
        float dd = d/1000.0; // mm -> m
        Point2d p1_norm = pixel2camNormPlane ( keypoints_1[m.queryIdx].pt, K );// point's pos on cam1's normalized plane
        pts_3d.push_back ( Point3f ( p1_norm.x*dd, p1_norm.y*dd, dd ) );
        pts_2d.push_back ( keypoints_2[m.trainIdx].pt );
    }
    cout<<"Number of 3d-2d pairs: "<<pts_3d.size() <<endl;

    // Solve PnP
    Mat R_vec, t, R;
    // solvePnP( pts_3d, pts_2d, K, Mat(), R_vec, t, false );
    solvePnPRansac ( pts_3d, pts_2d, K, Mat(), R_vec, t, false );
    Rodrigues ( R_vec, R );

    // Get cam1 to cam2
    Mat T_cam1_to_cam2 = transRt2T(R,t).inv();

    // Print result
    printf("Print result of PnP:\n");
    cout <<"\nFrom cam2 to cam1:"<<endl;
    cout<<"-- R    ="<<endl<<R<<endl;
    cout<<"-- R_vec="<<endl<<R_vec.t()<<endl;
    cout<<"-- t    ="<<endl<<t.t()<<endl;
    cout <<"\nFrom cam1 to cam2:"<<endl;
    cout<<"-- T="<<endl<<T_cam1_to_cam2<<endl;

}
