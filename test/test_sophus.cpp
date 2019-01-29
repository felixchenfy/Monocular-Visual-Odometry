#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>

#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/core/eigen.hpp>

using namespace std;

int main(int argc, char **argv)
{
    // -------------------- Init value from OpenCV --------------------
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.3, 0.1, -0.5);
    cv::Rodrigues(rvec, R);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 1, 2, 3);

    cout << "----- Initial value in OpenCV form -----" << endl;
    cout << "R=\n"
         << R << endl;
    cout << "t=" << tvec.t() << endl;

    // -------------------- Convert to Sophus --------------------
    Sophus::SE3 T = Sophus::SE3(
        Sophus::SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
        Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
    cout << "Change form to Sophus:"<<endl;
    cout << "Sophus::SE3 T = \n"
         << T << endl;

    // -------------------- Then convert to Eigen --------------------
    cout << "\n\nChange back to Eigen:"<<endl;
    Eigen::Vector3d eigen_t(T.translation());
    Eigen::Matrix3d eigen_R(T.rotation_matrix());
    cout << eigen_t << endl;
    cout << eigen_R << endl;

    // -------------------- Then convert to OpenCV --------------------
    cout << "\n\nChange back to OpenCV:"<<endl;
    cv::Mat cv_t, cv_R;
    eigen2cv(eigen_t, cv_t);
    eigen2cv(eigen_R, cv_R);
    cout << cv_t << endl;
    cout << cv_R << endl;

    return 0;
}
