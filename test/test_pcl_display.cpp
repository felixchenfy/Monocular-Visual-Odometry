// Test my wrapped up pcl functions for displaying:
//  * A moving camera.
//  * Trajecotry of the camera.
//  * Keep on adding a keypoint into the cloud.
// If this works, the lib funcs can be used for my SLAM project.

#include <iostream>

#include "my_display/pcl_display.h"
#include "my_basics/eigen_funcs.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace my_display;

string cloud_name = "sample cloud";
string str_viewer = "3D Viewer";
string str_frame_reference = "reference";

boost::shared_ptr<pcl::visualization::PCLVisualizer>
initPointCloudViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(str_viewer));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}


int main()
{

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>); // This uses constructor. Cannot separate to two sentences.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = initPointCloudViewer(point_cloud_ptr);

    // set up affine matrix
    Eigen::Affine3d T_affine;
    cv::Mat R_vec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat t = (cv::Mat_<double>(3, 1) << 0, 0, 0);

    // the XYZRGB cloud will gradually go from red to green to blue.
    std::cout << "Generating example point clouds.\n\n";
    uint8_t r(255), g(15), b(15);
    float z = -1.0;
    int cnt = 0;
    while (true)
    {
        cnt++;

        // Simulate camera motion.
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
        Eigen::Affine3f T_affine = my_basics::transCVMatRt2Affine3d(R_vec, t).cast<float>();
        cout << T_affine.matrix() << endl;

        // Simulate feature point captured by the camera.
        double x = 0, y = 0, z = 2;
        cv::Mat kpt_3d_pos_in_cam = (Mat_<double>(3,1) << x, y, z);
        Mat R;
        cv::Rodrigues(R_vec, R);
        cv::Mat kpt_3d_pos_in_world = R * kpt_3d_pos_in_cam + t;

        // Update camera in pcl
        viewer->removeCoordinateSystem(str_frame_reference);
        viewer->addCoordinateSystem(1.0, T_affine, str_frame_reference, 0);

        // Update point cloud in pcl
        pcl::PointXYZRGB point;
        setPointPos(point, kpt_3d_pos_in_world);
        unsigned char r=255, g=0, b=0;
        setPointColor(point, r, g, b);
        point_cloud_ptr->points.push_back(point);

        // Add to viewer
        // viewer->removePointCloud(cloud_name);
        // viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, cloud_name);
        viewer->updatePointCloud(point_cloud_ptr, cloud_name);

        // spin
        viewer->spinOnce(100);
        cout << cnt << "th loop"  << endl;
        if (viewer->wasStopped())
            break;
    }
    return (1);
}