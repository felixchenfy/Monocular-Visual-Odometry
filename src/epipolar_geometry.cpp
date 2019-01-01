
#include "mygeometry/epipolar_geometry.h"

using namespace std;
using namespace cv;

#define PRINT_RESULT true
namespace mygeometry
{

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

void print_MatProperty(Mat &M)
{
    string ty = cvMatType2str(M.type());
    printf("Mat: type = %s, size = %dx%d \n", ty.c_str(), M.rows, M.cols);
}

Point2f pixel2camNormPlane(const Point2f &p, const Mat &K)
{
    return Point2f(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}
Point3f pixel2cam(const Point2f &p, const Mat &K, double depth)
{
    return Point3f(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1),
        depth);
}

void print_R_t(const Mat &R, const Mat &t)
{
    Mat rvec;
    Rodrigues(R, rvec);
    cout << "R is:\n"
         << R << endl;
    cout << "R_vec is: " << rvec.t() << endl;
    cout << "t is: " << t.t() << endl;
}

Mat skew(const Mat &t)
{
    Mat t_x = (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
               t.at<double>(2, 0), 0, -t.at<double>(0, 0),
               -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    return t_x;
}

void extractPtsFromMatches(const vector<KeyPoint> &keypoints_1,
                           const vector<KeyPoint> &keypoints_2,
                           const vector<DMatch> &matches,
                           vector<Point2f> &pts1,
                           vector<Point2f> &pts2)
{
    for (auto &m : matches)
    {
        pts1.push_back(keypoints_1[m.queryIdx].pt);
        pts2.push_back(keypoints_2[m.trainIdx].pt);
    }
}

void estiMotionByEssential(
    const vector<Point2f> &pts_in_img1,
    const vector<Point2f> &pts_in_img2,
    const Mat &camera_intrinsics,
    Mat &R, Mat &t,
    vector<int> &inliers_index,
    vector<Point2f> &inlier_pts_in_img1,
    vector<Point2f> &inlier_pts_in_img2)
{
    Mat K = camera_intrinsics;                                           // rename
    Point2f principal_point(K.at<double>(0, 2), K.at<double>(1, 2));     // optical center
    double focal_length = (K.at<double>(0, 0) + K.at<double>(1, 1)) / 2; // focal length

    // -- Essential matrix
    Mat essential_matrix;
    int method = RANSAC;
    double prob = 0.999;
    double threshold = 1.0;
    Mat inliers_mask; //Use print_MatProperty to know its type: 8UC1
    essential_matrix = findEssentialMat(
        pts_in_img1, pts_in_img2, focal_length, principal_point,
        method, prob, threshold,
        inliers_mask);
    essential_matrix /= essential_matrix.at<double>(2, 2);
    // print_MatProperty(inliers_mask);

    // Get inliers
    inliers_index.clear();
    inlier_pts_in_img1.clear();
    inlier_pts_in_img2.clear();
    for (int i = 0; i < inliers_mask.rows; i++)
    {
        if ((int)inliers_mask.at<unsigned char>(i, 0) == 1)
        {
            inliers_index.push_back(i);
            inlier_pts_in_img1.push_back(pts_in_img1[i]);
            inlier_pts_in_img2.push_back(pts_in_img2[i]);
        }
    }

    // Recover R,t from Essential matrix
    recoverPose(essential_matrix, pts_in_img1, pts_in_img2, R, t, focal_length, principal_point);

    // Print
    if (PRINT_RESULT)
    {
        cout << endl
             << "=====" << endl;
        cout << "* Essential_matrix (by findEssentialMat) is: " << endl
             << essential_matrix << endl;
        cout << "* Number of inliers: " << inliers_index.size() << endl;
        cout << "* Recovering R and t from essential matrix:" << endl;
        print_R_t(R, t);
        cout << endl;
    }
}

void estiMotionByEssential(
    const vector<Point2f> &pts_in_img1,
    const vector<Point2f> &pts_in_img2,
    const Mat &camera_intrinsics,
    Mat &R, Mat &t)
{

    vector<int> inliers_index;
    vector<Point2f> inlier_pts_in_img1, inlier_pts_in_img2;
    estiMotionByEssential(pts_in_img1, pts_in_img2, camera_intrinsics,
                          R, t, inliers_index, inlier_pts_in_img1, inlier_pts_in_img2);
}

void removeWrongRtOfHomography(
    const vector<Point2f> &pts_in_img1, const vector<Point2f> &pts_in_img2,
    vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> &normals)
{
    Mat possibleSolutions; //Use print_MatProperty to know its type: 32SC1
    filterHomographyDecompByVisibleRefpoints(Rs, normals, pts_in_img1, pts_in_img2, possibleSolutions);
    vector<Mat> res_Rs, res_ts, res_normals;
    int num_solutions = possibleSolutions.rows;

    for (int i = 0; i < num_solutions; i++)
    {
        int idx = possibleSolutions.at<int>(i, 0);
        res_Rs.push_back(Rs[idx]);
        res_ts.push_back(ts[idx]);
        res_normals.push_back(normals[idx]);
    }
    Rs = res_Rs;
    ts = res_ts;
    normals = res_normals;
}

void estiMotionByHomography(const vector<Point2f> &pts_in_img1,
                            const vector<Point2f> &pts_in_img2,
                            const Mat &camera_intrinsics,
                            vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> normals,
                            vector<int> &inliers_index,
                            vector<Point2f> &inlier_pts_in_img1, vector<Point2f> &inlier_pts_in_img2)
{
    // https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#findhomography

    // -- Homography matrix
    int method = RANSAC;
    double ransacReprojThreshold = 3;
    Mat inliers_mask; //Use print_MatProperty to know its type: 8UC1
    Mat homography_matrix = findHomography(pts_in_img1, pts_in_img2, method, ransacReprojThreshold, inliers_mask);
    homography_matrix /= homography_matrix.at<double>(2, 2);

    // Get inliers
    inliers_index.clear();
    inlier_pts_in_img1.clear();
    inlier_pts_in_img2.clear();
    for (int i = 0; i < inliers_mask.rows; i++)
    {
        if ((int)inliers_mask.at<unsigned char>(i, 0) == 1)
        {
            inliers_index.push_back(i);
            inlier_pts_in_img1.push_back(pts_in_img1[i]);
            inlier_pts_in_img2.push_back(pts_in_img2[i]);
        }
    }

    // -- Recover R,t from Homograph matrix
    decomposeHomographyMat(
        homography_matrix, camera_intrinsics, Rs, ts, normals);

    // Try to remove wrong solutions
    removeWrongRtOfHomography(pts_in_img1, pts_in_img2, Rs, ts, normals);

    // Print result
    if (PRINT_RESULT)
    {
        cout << endl
             << "=====" << endl;
        cout << "* Homography_matrix (by findHomography) is " << endl
             << homography_matrix << endl;
        cout << "* Number of inliers: " << inliers_index.size() << endl;

        // cout << "The inliers_mask is " << inliers_mask << endl;
        // !!! add it later

        int num_solutions = Rs.size();
        for (int i = 0; i < num_solutions; i++)
        {
            cout << endl;
            cout << "* Solution " << i + 1 << ":" << endl; // Start index from 1. The index 0 is for essential matrix.
            print_R_t(Rs[i], ts[i]);
            cout << "plane normal: " << normals[i].t() << endl;
            cout << endl;
        }
    }
}

void estiMotionByHomography(const vector<Point2f> &pts_in_img1,
                            const vector<Point2f> &pts_in_img2,
                            const Mat &camera_intrinsics,
                            vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> normals)
{
    vector<int> inliers_index;
    vector<Point2f> inlier_pts_in_img1, inlier_pts_in_img2;
    estiMotionByHomography(pts_in_img1, pts_in_img2, camera_intrinsics, Rs, ts, normals,
                           inliers_index, inlier_pts_in_img1, inlier_pts_in_img2);
}

double computeEpipolarConsError(
    const Point2f &p1, const Point2f &p2, const Mat &R, const Mat &t, const Mat &K)
{
    Point2f p_cam1 = pixel2camNormPlane(p1, K); // point's pos in the camera frame
    Mat y1 = (Mat_<double>(3, 1) << p_cam1.x, p_cam1.y, 1);
    Point2f p_cam2 = pixel2camNormPlane(p2, K);
    Mat y2 = (Mat_<double>(3, 1) << p_cam2.x, p_cam2.y, 1);
    Mat d = y2.t() * skew(t) * R * y1;
    return d.at<double>(0, 0);
}

Mat transRt2T(const Mat &R, const Mat &t)
{
    Mat T = (Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    return T;
}

void triangulation(
    const vector<Point2f> &pts_in_img1,
    const vector<Point2f> &pts_in_img2,
    const Mat &R_cam2_to_cam1, const Mat &t_cam2_to_cam1,
    const Mat &K,
    vector<Point3f> &pts3d_in_cam1)
{
    Mat T_cam1_to_world =
        (Mat_<float>(3, 4) << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0);
    Mat T_cam2_to_world = transRt2T(R_cam2_to_cam1, t_cam2_to_cam1);

    // points pos from [pixel frame] to [normalized camera frame]
    vector<Point2f> pts_in_cam1, pts_in_cam2;
    for (int i = 0; i < pts_in_img1.size(); i++)
    {
        pts_in_cam1.push_back(pixel2camNormPlane(pts_in_img1[i], K));
        pts_in_cam2.push_back(pixel2camNormPlane(pts_in_img2[i], K));
    }

    // triangulartion
    Mat pts4d_in_world;
    cv::triangulatePoints(T_cam1_to_world, T_cam2_to_world,
                          pts_in_cam1, pts_in_cam2, pts4d_in_world);

    // change to homogeneous coords
    vector<Point3f> pts3d_in_world;
    for (int i = 0; i < pts4d_in_world.cols; i++)
    {
        Mat x = pts4d_in_world.col(i);
        x /= x.at<float>(3, 0); // 归一化
        Point3f p(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0));
        pts3d_in_world.push_back(p);
        // if(i<3)cout<<"in tri, i="<<i<<","<<"p="<<p<<endl;
    }

    // return
    pts3d_in_cam1 = pts3d_in_world;
}

} // namespace mygeometry