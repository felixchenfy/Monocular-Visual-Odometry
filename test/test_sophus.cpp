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

cv::Mat transRt2T(const cv::Mat &R, const cv::Mat &t)
{
    cv::Mat T = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                 R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                 R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
                 0, 0, 0, 1);
    return T;
}

Sophus::SE3 transT_cv2sophus(const cv::Mat &T_cv)
{
    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(T_cv(cv::Rect2d(0, 0, 3, 3)), R_eigen);
    Eigen::Vector3d t_eigen(T_cv.at<double>(0, 3), T_cv.at<double>(1, 3), T_cv.at<double>(2, 3));
    Sophus::SE3 SE3_Rt(R_eigen, t_eigen);
    return SE3_Rt;
}

cv::Mat transT_sophus2cv(const Sophus::SE3 &T_sophus)
{
    Eigen::Vector3d eigen_t(T_sophus.translation());
    Eigen::Matrix3d eigen_R(T_sophus.rotation_matrix());

    cv::Mat cv_t, cv_R;
    eigen2cv(eigen_t, cv_t);
    eigen2cv(eigen_R, cv_R);

    return transRt2T(cv_R, cv_t);
}

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
    cout << "R_vec = " << rvec.t() << endl;
    cout << "t=" << tvec.t() << endl;

    // -------------------- Convert to Sophus --------------------
    Sophus::SE3 T = Sophus::SE3(
        Sophus::SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
        Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
    cout << "Change form to Sophus:" << endl;
    cout << "Sophus::SE3 T = \n"
         << T << endl;

    // -------------------- Then convert to Eigen --------------------
    cout << "\n\nChange back to Eigen:" << endl;
    Eigen::Vector3d eigen_t(T.translation());
    Eigen::Matrix3d eigen_R(T.rotation_matrix());
    cout << eigen_t << endl;
    cout << eigen_R << endl;

    // -------------------- Then convert to OpenCV --------------------
    cout << "\n\nChange back to OpenCV:" << endl;
    cv::Mat cv_t, cv_R;
    eigen2cv(eigen_t, cv_t);
    eigen2cv(eigen_R, cv_R);
    cout << cv_t << endl;
    cout << cv_R << endl;

    // -------------------- Direct trans from T_cv to T_Sophus --------------------
    cv::Mat T_cv = transRt2T(R, tvec);
    Sophus::SE3 T_SE3 = transT_cv2sophus(T_cv);
    cout << "\nDirect trans from T_cv to T_Sophus: \n"
         << T_SE3 << endl;
    // -------------------- Direct trans from T_sophus to T_cv --------------------
    T_cv = transT_sophus2cv(T_SE3);
    cout << "\nDirect trans from T_sophus to T_cv:\n"
         << T_cv << endl;
    return 0;
}
