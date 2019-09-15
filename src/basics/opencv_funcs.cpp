
#include "my_slam/basics/opencv_funcs.h"

namespace my_slam
{
namespace basics
{

// ---------------- image operation ----------------
vector<unsigned char> getPixelAt(const cv::Mat &image, int x, int y)
{
    unsigned char bgr[3] = {0, 0, 0};

    if (0)
    {
        const unsigned char *row_ptr = image.ptr<unsigned char>(y);
        const unsigned char *data_ptr = &row_ptr[x * image.channels()];
        for (int c = 0; c != image.channels(); c++)
        {
            bgr[c] = data_ptr[c];
        }
    }
    else
    {
        for (int c = 0; c < 3; c++)
        {
            bgr[c] = image.at<cv::Vec3b>(y, x)[c];
        }
    }
    vector<unsigned char> rgb{bgr[2], bgr[1], bgr[0]};
    return rgb;
}

unsigned char getPixelAt(const cv::Mat &image, int row, int col, int idx_rgb)
{
    return image.at<cv::Vec3b>(row, col)[idx_rgb];
}

// ---------------- datatype conversion ----------------

cv::Mat point3f_to_mat3x1(const cv::Point3f &p)
{
    return (cv::Mat_<double>(3, 1) << p.x, p.y, p.z);
}
cv::Mat point3f_to_mat4x1(const cv::Point3f &p)
{
    return (cv::Mat_<double>(4, 1) << p.x, p.y, p.z, 1);
}

cv::Point3f Mat3x1_to_Point3f(const cv::Mat &p)
{
    return cv::Point3f(p.at<double>(0, 0), p.at<double>(1, 0), p.at<double>(2, 0));
}

cv::Mat point2f_to_mat2x1(const cv::Point2f &p)
{
    return (cv::Mat_<double>(2, 1) << p.x, p.y);
}

cv::Mat getZerosMat(int rows, int cols, int type)
{
    return cv::Mat::zeros(cv::Size(rows, cols), type);
}

// ---------------- Transformations ----------------

cv::Point3f preTranslatePoint3f(const cv::Point3f &p3x1, const cv::Mat &T4x4)
{
    const cv::Mat &T = T4x4;
    double p[4] = {p3x1.x, p3x1.y, p3x1.z, 1};
    double res[3] = {0, 0, 0};
    for (int row = 0; row < 3; row++)
    {
        for (int j = 0; j < 4; j++)
            res[row] += T.at<double>(row, j) * p[j];
    }
    return cv::Point3f(res[0], res[1], res[2]);
}

// ---------------- Math ----------------

cv::Mat skew(const cv::Mat &t)
{
    cv::Mat t_x = (cv::Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
                   t.at<double>(2, 0), 0, -t.at<double>(0, 0),
                   -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    return t_x;
}

cv::Mat convertRt2T(const cv::Mat &R, const cv::Mat &t)
{
    cv::Mat T = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                 R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                 R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
                 0, 0, 0, 1);
    return T;
}
cv::Mat convertRt2T_3x4(const cv::Mat &R, const cv::Mat &t)
{
    cv::Mat T = (cv::Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                 R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                 R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    return T;
}
void getRtFromT(const cv::Mat &T, cv::Mat &R, cv::Mat &t)
{
    R = (cv::Mat_<double>(3, 3) << T.at<double>(0, 0), T.at<double>(0, 1), T.at<double>(0, 2),
         T.at<double>(1, 0), T.at<double>(1, 1), T.at<double>(1, 2),
         T.at<double>(2, 0), T.at<double>(2, 1), T.at<double>(2, 2));
    t = (cv::Mat_<double>(3, 1) << T.at<double>(0, 3),
         T.at<double>(1, 3),
         T.at<double>(2, 3));
    // R = T(cv::Rect(0,0,3,3)).clone();
    // t = T(cv::Rect(3,0,1,3)).clone(); // x, y, width, height
}

cv::Mat getPosFromT(const cv::Mat &T)
{
    return T(cv::Rect(3, 0, 1, 3)).clone();
}
cv::Point3f transCoord(const cv::Point3f &p, const cv::Mat &R, const cv::Mat &t)
{
    cv::Mat p2 = R * point3f_to_mat3x1(p) + t; // 3d pos in camera 2
    return cv::Point3f(p2.at<double>(0, 0), p2.at<double>(1, 0), p2.at<double>(2, 0));
}
void invRt(cv::Mat &R, cv::Mat &t)
{
    cv::Mat T = convertRt2T(R, t);
    getRtFromT(T.inv(), R, t);
}

double calcDist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

double calcMeanDepth(const vector<cv::Point3f> &pts_3d)
{
    double mean_depth = 0;
    for (const cv::Point3f &p : pts_3d)
        mean_depth += p.z;
    mean_depth /= pts_3d.size();
    return mean_depth;
}

double scalePointPos(cv::Point3f &p, double scale)
{
    p.x *= scale;
    p.y *= scale;
    p.z *= scale;
}
double calcMatNorm(const cv::Mat &mat)
{
    double sum = 0;
    for (int i = 0; i < mat.rows; i++)
    {
        for (int j = 0; j < mat.cols; j++)
        {
            sum = sum + mat.at<double>(i, j) * mat.at<double>(i, j);
        }
    }
    double norm = sqrt(sum);
    return norm;
}
cv::Mat getNormalizedMat(const cv::Mat mat)
{
    double len = calcMatNorm(mat);
    cv::Mat res = mat / len;
    return res;
}

double calcAngleBetweenTwoVectors(const cv::Mat &vec1, const cv::Mat &vec2)
{
    // cos(angle) = vec1.dot(vec2) / (||vec1|| * ||vec2||)
    assert(vec1.rows == vec2.rows && vec1.rows > vec1.cols);
    int N = vec1.rows;
    double res = 0;
    for (int i = 0; i < N; i++)
    {
        res += vec1.at<double>(i, 0) * vec2.at<double>(i, 0);
    }
    double len = calcMatNorm(vec1) * calcMatNorm(vec2);
    assert(len != 0);
    res = acos(res / len);
    return res;
}

// ---------------- Print ----------------
string getCvMatType(int cvMatType);
void print_MatProperty(const cv::Mat &M)
{
    string ty = getCvMatType(M.type());
    printf("cv::Mat: type = %s, size = %dx%d \n", ty.c_str(), M.rows, M.cols);
}

void print_R_t(const cv::Mat &R, const cv::Mat &t)
{
    cv::Mat rvec;
    Rodrigues(R, rvec);
    // cout << "R is:\n"
    //      << R << endl;
    cout << "R_vec is: " << rvec.t() << endl;
    cout << "t is: " << t.t() << endl;
}

string getCvMatType(int cvMatType)
{
    /*
    https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
    example:
    string ty =  getCvMatType( M.type() );
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

} // namespace basics
} // namespace my_slam
