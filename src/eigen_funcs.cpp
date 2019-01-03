
#include "my_basics/eigen_funcs.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace my_basics
{
    
Eigen::Affine3d transCVMatRt2Affine3d(const Mat &R0, const Mat &t)
{
    // check input: whether R0 is a SO3 or xyz-Euler-Angles
    Mat R = R0.clone();
    if (R.rows == 3 && R.cols == 1)
        cv::Rodrigues(R, R);
    assert(R.rows == 3 && R.cols == 3);

    // compute
    Eigen::Affine3d T;
    T.linear() = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Map(reinterpret_cast<const double *>(R.data));
    T.translation() = Eigen::Vector3d::Map(reinterpret_cast<const double *>(t.data));
    return T;
}

Eigen::Affine3d getAffine3d(double x, double y, double z, double ea_x, double ea_y, double ea_z){
    cv::Mat t=(cv::Mat_<double>(3,1)<<x,y,z);
    cv::Mat R_vec=(cv::Mat_<double>(3,1)<<ea_x,ea_y,ea_z);
    return transCVMatRt2Affine3d(R_vec, t);
}

}