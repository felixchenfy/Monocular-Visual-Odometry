// Test my wrapped up pcl functions for displaying:
//  * A moving camera.
//  * Trajecotry of the camera.
//  * Keep on adding a keypoint into the cloud.
// If this works, the lib funcs can be used for my SLAM project.

#include <iostream>

// #include "my_slam/basics/eigen_funcs.h"

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "my_slam/display/pcl_display.h"

using namespace std;
using namespace cv;
// using namespace Eigen;

using namespace display;

int main()
{
    // Init my PclViewer class
    double dis_scale=3;
    double  x = 0.5*dis_scale,
            y = -1.0*dis_scale,
            z = -1.0*dis_scale;
    double rot_axis_x = -0.5, rot_axis_y = 0, rot_axis_z = 0;
    string viewer_name = "my pcl viewer";
    display::PclViewer::Ptr pcl_displayer(
        new display::PclViewer(
            viewer_name, x, y, z, rot_axis_x, rot_axis_y, rot_axis_z));

    // Set up camera pos.
    // Eigen::Affine3d T_affine;
    cv::Mat R_vec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat t = (cv::Mat_<double>(3, 1) << 0, 0, 0);

    // Loop
    int cnt = 0;
    while (true)
    {
        cnt++;

        // Simulate camera motion.
        //      Return: R_vec, t
        const double period = 10.0;
        t.at<double>(0, 0) = sin(cnt / period);
        t.at<double>(1, 0) = cos(cnt / period);
        t.at<double>(2, 0) -= 0.01;
        enum
        {
            X_AXIS,
            Y_AXIS,
            Z_AXIS
        };
        R_vec.at<double>(X_AXIS, 0) += 0.01;

        // Simulate feature point captured by the camera.
        //      Return: kpt_3d_pos_in_world, r, g, b
        double x = 0, y = 0, z = 2;
        unsigned char r = 255, g = 0, b = 0;
        cv::Mat kpt_3d_pos_in_cam = (Mat_<double>(3, 1) << x, y, z);
        Mat R;
        cv::Rodrigues(R_vec, R);
        cv::Mat kpt_3d_pos_in_world = R * kpt_3d_pos_in_cam + t;

        // Display
        pcl_displayer->updateCameraPose(R_vec, t);
        // pcl_displayer->addPoint(kpt_3d_pos_in_world, r, g, b); // This function is deprecated
        pcl_displayer->update();
        pcl_displayer->spinOnce(150);
        if (pcl_displayer->wasStopped())
            break;
    }
    return (1);
}