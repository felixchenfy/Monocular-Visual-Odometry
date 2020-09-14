#include "my_slam/basics/eigen_funcs.h"

#include "my_slam/basics/opencv_funcs.h"

namespace my_slam
{
namespace basics
{

// --------------  Eigen --------------
Eigen::Affine3d getAffine3d(double x, double y, double z, double rot_axis_x, double rot_axis_y, double rot_axis_z)
{
    cv::Mat t = (cv::Mat_<double>(3, 1) << x, y, z);
    cv::Mat R_vec = (cv::Mat_<double>(3, 1) << rot_axis_x, rot_axis_y, rot_axis_z);
    return transT_CVRt_to_EigenAffine3d(R_vec, t);
}

// -------------- CV <--> Eigen --------------
Eigen::Affine3d transT_CVRt_to_EigenAffine3d(const cv::Mat &R0, const cv::Mat &t)
{
    // check input: whether R0 is a SO3 or xyz-Euler-Angles
    cv::Mat R = R0.clone();
    if (R.rows == 3 && R.cols == 1)
        cv::Rodrigues(R, R);
    assert(R.rows == 3 && R.cols == 3);

    // compute
    Eigen::Affine3d T;
    T.linear() = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Map(reinterpret_cast<const double *>(R.data));
    T.translation() = Eigen::Vector3d::Map(reinterpret_cast<const double *>(t.data));
    return T;
}

// -------------- CV <--> Sophus --------------

Sophus::SE3d transT_cv2sophus(const cv::Mat &T_cv)
{
    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(T_cv(cv::Rect2d(0, 0, 3, 3)), R_eigen);
    Eigen::Vector3d t_eigen(T_cv.at<double>(0, 3), T_cv.at<double>(1, 3), T_cv.at<double>(2, 3));
    Sophus::SE3d SE3_Rt(R_eigen, t_eigen);
    return SE3_Rt;
}

cv::Mat transT_sophus2cv(const Sophus::SE3d &T_sophus)
{
    Eigen::Vector3d eigen_t(T_sophus.translation());
    Eigen::Matrix3d eigen_R(T_sophus.rotationMatrix());

    cv::Mat cv_t, cv_R;
    eigen2cv(eigen_t, cv_t);
    eigen2cv(eigen_R, cv_R);

    return basics::convertRt2T(cv_R, cv_t);
}

} // namespace basics
} // namespace my_slam
