

// #include <iostream>
// int main(void){
//     return (1);
// }


// This is for testing basic functions/datatypes in Eigen.
// 1. Datatype conversions between:
//      OpenCV and Matrix3d/Vector3d/Isometry3d/Affine3d.

#include "my_common/eigen_funcs.h" // namespce my

using namespace std;
using namespace cv;
using namespace Eigen;

// tutorials:
// https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html

void printRtT(Eigen::Matrix3d &Re, Eigen::Vector3d &te,
              Eigen::Isometry3d &Te, Eigen::Affine3d &Te_affine);

void transRtFromCV2Eigen_good_method(const Mat &R, const Mat t,
                                     Eigen::Matrix3d &Re, Eigen::Vector3d &te,
                                     Eigen::Isometry3d &Te, Eigen::Affine3d &Te_affine);
void transRtFromCV2Eigen_manually(const Mat &R_vec, const Mat t,
                                  Eigen::Matrix3d &Re, Eigen::Vector3d &te,
                                  Eigen::Isometry3d &Te, Eigen::Affine3d &Te_affine);
void notes_about_AngelAxis();

int main(void)
{

    // Suppose I get camera pose from OpenCV.
    Mat t = (Mat_<double>(3, 1) << 1, 1, 1);
    Mat R_vec = (Mat_<double>(3, 1) << 0, 0.1, 0);
    Mat R;
    cv::Rodrigues(R_vec, R);

    // Print
    cout << "Printing R and t in OpenCV mat format:" << endl;
    cout << "t: " << t.t() << endl;
    cout << "R: \n"
         << R << endl;

    { // test 1: test test functions defined in this script

        Eigen::Matrix3d Re;
        Eigen::Vector3d te;
        Eigen::Isometry3d Te;
        Eigen::Affine3d Te_affine;
        transRtFromCV2Eigen_good_method(R, t, Re, te, Te, Te_affine);
        transRtFromCV2Eigen_manually(R_vec, t, Re, te, Te, Te_affine);
    }

    { // test 2: functions in the library

        Eigen::Affine3d T_affine = my::transCVMatRt2Affine3d(R_vec, t);
        Eigen::Affine3f tmpT = T_affine.cast<float>();
        cout << "\n==============================================\n";
        cout << "Print result of T_affine:\n"
             << tmpT.matrix() << endl;
    }
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

void transRtFromCV2Eigen_good_method(const Mat &R, const Mat t,
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

void transRtFromCV2Eigen_manually(const Mat &R_vec, const Mat t,
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