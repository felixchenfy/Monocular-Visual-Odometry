

// #include <iostream>
// int main(void){
//     return (1);
// }

// This is for testing basic functions/datatypes in Eigen.
// 1. Datatype conversions between:
//      OpenCV and Matrix3d/Vector3d/Isometry3d/Affine3d.

#include "my_slam/basics/eigen_funcs.h" // namespce my

using namespace std;
using namespace cv;
using namespace Eigen;

// tutorials:
// https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html

void printRtT(Eigen::Matrix3d &Re, Eigen::Vector3d &te,
              Eigen::Isometry3d &Te, Eigen::Affine3d &Te_affine);

void transRtFromCV2Eigen_good_method(const cv::Mat &R, const cv::Mat &t,
                                     Eigen::Matrix3d &Re, Eigen::Vector3d &te,
                                     Eigen::Isometry3d &Te, Eigen::Affine3d &Te_affine);
void transRtFromCV2Eigen_manually(const cv::Mat &R_vec, const cv::Mat &t,
                                  Eigen::Matrix3d &Re, Eigen::Vector3d &te,
                                  Eigen::Isometry3d &Te, Eigen::Affine3d &Te_affine);
void transRtFromCV2Eigen_copy_elements(const cv::Mat &R, const cv::Mat &t,
                                       Eigen::Isometry3d &Te);
void notes_about_AngelAxis();

int main(void)
{

    // Suppose I get camera pose from OpenCV.
    cv::Mat t = (cv::Mat_<double>(3, 1) << 1, 2, 3);
    cv::Mat R_vec = (cv::Mat_<double>(3, 1) << 0, 0.1, 0);
    cv::Mat R;
    cv::Rodrigues(R_vec, R);

    // Print
    cout << "Printing R and t in OpenCV mat format:" << endl;
    cout << "t: " << t.t() << endl;
    cout << "R: \n"
         << R << endl;

    Eigen::Matrix3d Re;
    Eigen::Vector3d te;
    Eigen::Isometry3d Te;
    Eigen::Affine3d Te_affine;
    transRtFromCV2Eigen_good_method(R, t, Re, te, Te, Te_affine);
    transRtFromCV2Eigen_manually(R_vec, t, Re, te, Te, Te_affine);
    transRtFromCV2Eigen_copy_elements(R, t, Te);
    return 1;
}

void printRtT(Eigen::Matrix3d &Re, Eigen::Vector3d &te,
              Eigen::Isometry3d &Te, Eigen::Affine3d &Te_affine)
{
    cout << endl;
    cout << "--------------------------------------" << endl;
    cout << "Print result after transforming from cv to Eigen:" << endl;
    cout << "t:" << te.transpose() << endl;
    cout << "R:\n"
         << Re << endl;
    cout << "T:\n"
         << Te.matrix() << endl;
    cout << "T_affine:\n"
         << Te_affine.matrix() << endl;
    cout << "--------------------------------------" << endl;
}

void transRtFromCV2Eigen_good_method(const cv::Mat &R, const cv::Mat &t,
                                     Eigen::Matrix3d &Re, Eigen::Vector3d &te,
                                     Eigen::Isometry3d &Te, Eigen::Affine3d &Te_affine)
{
    assert(R.rows == 3 && R.cols == 3);
    typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> RMatrix3d;

    Re = RMatrix3d::Map(reinterpret_cast<const double *>(R.data));
    te = Eigen::Vector3d::Map(reinterpret_cast<const double *>(t.data));
    if (1)
    {
        Te.linear() = Re;
        Te.translation() = te;
    }
    else
    { // or directly assign values
        Te.linear() = RMatrix3d::Map(reinterpret_cast<const double *>(R.data));
        Te.translation() = Eigen::Vector3d::Map(reinterpret_cast<const double *>(t.data));
    }
    Te_affine = Te.matrix();

    // print result
    printRtT(Re, te, Te, Te_affine);
}

void transRtFromCV2Eigen_manually(const cv::Mat &R_vec, const cv::Mat &t,
                                  Eigen::Matrix3d &Re, Eigen::Vector3d &te,
                                  Eigen::Isometry3d &Te, Eigen::Affine3d &Te_affine)
{
    assert(R_vec.rows == 3 && R_vec.cols == 1);

    // 1. convert te
    te = Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));

    // 2. convert Re
    double xyz_euler_angles[3] = {
        R_vec.at<double>(0, 0), R_vec.at<double>(1, 0), R_vec.at<double>(2, 0)};
    Re = AngleAxisd(xyz_euler_angles[0], Vector3d::UnitX()) *
         AngleAxisd(xyz_euler_angles[1], Vector3d::UnitY()) *
         AngleAxisd(xyz_euler_angles[2], Vector3d::UnitZ()).toRotationMatrix();

    // 3. convert Te
    Te = Eigen::Isometry3d::Identity();
    Te.rotate(Re).pretranslate(te);

    // 4. convert Te_affine
    Te_affine = Te.matrix();

    // print result
    printRtT(Re, te, Te, Te_affine);
}
cv::Mat convertRt2T(const cv::Mat &R, const cv::Mat &t)
{
    cv::Mat T = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
             0, 0, 0, 1);
    return T;
}
void transRtFromCV2Eigen_copy_elements(const cv::Mat &R, const cv::Mat &t,
                                       Eigen::Isometry3d &Te)
{
    cv::Mat T_cv2 = convertRt2T(R, t);
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            Te.matrix()(row, col) = T_cv2.at<double>(row, col);
    cout << "\nT in eigen:\n"
         << Te.matrix() << endl;
}
void notes_about_AngelAxis()
{
    /*
    Correct:
        Matrix3f m;
        m = AngleAxisf(1.0, Vector3f::UnitY());
    Correct:
        Matrix3f m=AngleAxisf(1.0, Vector3f::UnitY()).toRotationMatrix();
    Wrong:
        Matrix3f m=AngleAxisf(1.0, Vector3f::UnitY());
    */
}