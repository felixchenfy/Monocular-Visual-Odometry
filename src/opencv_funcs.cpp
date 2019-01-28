
#include "my_basics/opencv_funcs.h"

namespace my_basics
{


// ---------------- image operation ----------------
vector<unsigned char> getPixelAt(const Mat &image, int x, int y)
{
    unsigned char bgr[3]={0,0,0};
    
    if(0){
        const unsigned char *row_ptr = image.ptr<unsigned char>(y);
        const unsigned char *data_ptr = &row_ptr[x * image.channels()];
        for (int c = 0; c != image.channels(); c++)
        {
            bgr[c]=data_ptr[c];
        }
    }else{
        for (int c=0;c<=3;c++){
            bgr[c]=image.at<Vec3b>(y, x)[c];        
        }
    }
    vector<unsigned char> rgb{bgr[2],bgr[1],bgr[0]};
    return rgb;
}

unsigned char getPixelAt(const Mat &image, int row, int col, int idx_rgb){
    return image.at<Vec3b>(row, col)[idx_rgb];
}

// ---------------- datatype conversion ----------------

Mat Point3f_to_Mat(const Point3f &p)
{
    return (Mat_<double>(3, 1) << p.x, p.y, p.z);
}
Mat Point3f_to_Mat4x1(const Point3f &p){
    return (Mat_<double>(4, 1) << p.x, p.y, p.z, 1);
}

Point3f Mat_to_Point3f(const Mat &p){
    return Point3f(p.at<double>(0,0), p.at<double>(1,0), p.at<double>(2,0));
}


Mat Point2f_to_Mat(const Point2f &p)
{
    return (Mat_<double>(2, 1) << p.x, p.y);
}

// ---------------- Transformations ----------------

Point3f preTranslatePoint3f(const Point3f &p3x1, const Mat &T4x4){
    const Mat &T=T4x4;
    double p[4]={p3x1.x,p3x1.y,p3x1.z,1};
    double res[3]={0,0,0};
    for(int row=0;row<3;row++){
        for(int j=0; j<4; j++)
            res[row]+=T.at<double>(row,j)*p[j];
    }
    return Point3f(res[0],res[1],res[2]);
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
    t = (Mat_<double>(3, 1) << T.at<double>(0, 3),
         T.at<double>(1, 3),
         T.at<double>(2, 3));
    // R = T(cv::Rect(0,0,3,3)).clone(); 
    // t = T(cv::Rect(3,0,1,3)).clone(); // x, y, width, height
}
Mat getPosFromT(const Mat &T){
    return T(cv::Rect(3,0,1,3)).clone();
}
Point3f transCoord(const Point3f &p, const Mat &R, const Mat &t)
{
    Mat p2 = R * Point3f_to_Mat(p) + t; // 3d pos in camera 2
    return Point3f(p2.at<double>(0, 0), p2.at<double>(1, 0), p2.at<double>(2, 0));
}
void invRt(Mat &R, Mat &t)
{
    Mat T = transRt2T(R, t);
    getRtFromT(T.inv(), R, t);
}

double calcDist(const Point2f &p1, const Point2f &p2)
{
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}
double calcMeanDepth(const vector<Point3f> &pts_3d)
{
    double mean_depth = 0;
    for (const Point3f &p : pts_3d)
        mean_depth += p.z;
    mean_depth /= pts_3d.size();
    return mean_depth;
}
double scalePointPos(Point3f &p, double scale){
    p.x *= scale;
    p.y *= scale;
    p.z *= scale;
}
double calcMatNorm(const Mat &mat){
    double sum=0;
    for(int i=0;i<mat.rows;i++){
        for(int j=0;j<mat.cols;j++){
            sum=sum+mat.at<double>(i,j)*mat.at<double>(i,j);
        }
    }
    double norm=sqrt(sum);
    return norm;
}
void normalize(Mat &mat){
    double len = calcMatNorm(mat);
    mat /= len;
}


// ---------------- Print ----------------

void print_MatProperty(const Mat &M)
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
