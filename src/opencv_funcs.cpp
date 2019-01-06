
#include "my_basics/opencv_funcs.h"

namespace my_basics
{

Mat Point3f_to_Mat(const Point3f &p)
{
    return (Mat_<double>(3, 1) << p.x, p.y, p.z);
}
Mat Point2f_to_Mat(const Point2f &p)
{
    return (Mat_<double>(2, 1) << p.x, p.y);
}


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
    Mat T = (Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
             0, 0, 0, 1);
    return T;
}
Mat transRt2T_3x4(const Mat &R, const Mat &t)
{
    Mat T = (Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    return T;
}
void getRtFromT(const Mat &T, Mat &R, Mat &t)
{
    R = (Mat_<double>(3, 3) << T.at<double>(0, 0), T.at<double>(0, 1), T.at<double>(0, 2),
         T.at<double>(1, 0), T.at<double>(1, 1), T.at<double>(1, 2),
         T.at<double>(2, 0), T.at<double>(2, 1), T.at<double>(2, 2));
    t = (Mat_<double>(3, 1) << t.at<double>(0, 0),
         t.at<double>(1, 0),
         t.at<double>(2, 0));
}
Point3f transCoord(const Point3f &p, const Mat &R, const Mat &t){
   Mat p2 = R * Point3f_to_Mat(p) + t; // 3d pos in camera 2
   return Point3f(p2.at<double>(0,0),p2.at<double>(1,0), p2.at<double>(2,0));
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

} // namespace my_basics
