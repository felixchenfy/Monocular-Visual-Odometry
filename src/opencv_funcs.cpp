
#include "my_basics/opencv_funcs.h"

namespace my_basics
{

// ---------------- Math ----------------

Mat skew(const Mat &t)
{
    Mat t_x = (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
               t.at<double>(2, 0), 0, -t.at<double>(0, 0),
               -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    return t_x;
}

Mat transRt2T(const Mat &R, const Mat &t)
{
    Mat T = (Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    return T;
}
// ---------------- Print ----------------

void print_MatProperty(Mat &M)
{
    string ty = cvMatType2str(M.type());
    printf("Mat: type = %s, size = %dx%d \n", ty.c_str(), M.rows, M.cols);
}

void print_R_t(const Mat &R, const Mat &t)
{
    Mat rvec;
    Rodrigues(R, rvec);
    // cout << "R is:\n"
    //      << R << endl;
    cout << "R_vec is: " << rvec.t() << endl;
    cout << "t is: " << t.t() << endl;
}

string cvMatType2str(int cvMatType)
{
    /*
    https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
    example:
    string ty =  cvMatType2str( M.type() );
    printf("Matrix: %s %dx%d \n", ty.c_str(), M.cols, M.rows );
    */
    string r;

    uchar depth = cvMatType & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (cvMatType >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C";
    r += (chans + '0');
    return r;
}

}
