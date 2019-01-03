
#include "my_geometry/epipolar_geometry.h"

using namespace std;
using namespace cv;

#define PRINT_DEBUG_RESULT true
namespace my_geometry
{

Point2f pixel2camNormPlane(const Point2f &p, const Mat &K)
{
    return Point2f(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}
Point3f pixel2cam(const Point2f &p, const Mat &K, double depth)
{
    return Point3f(
        depth * (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        depth * (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1),
        depth);
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
    // Normalize t
    t = t / sqrt(t.at<double>(1, 0) * t.at<double>(1, 0) + t.at<double>(2, 0) * t.at<double>(2, 0) +
                 t.at<double>(0, 0) * t.at<double>(0, 0));

    // Print
    if (PRINT_DEBUG_RESULT)
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
    const vector<Point2f> &pts_in_img1, const vector<Point2f> &pts_in_img2, const Mat &K,
    vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> &normals)
{
    // Convert pts_in_image to pts_on_norm_plane
    vector<Point2f> pts_on_nplane1, pts_on_nplane2;
    for (int i = 0; i < pts_in_img1.size(); i++)
    {
        pts_on_nplane1.push_back(pixel2camNormPlane(pts_in_img1[i], K));
        pts_on_nplane2.push_back(pixel2camNormPlane(pts_in_img2[i], K));
    }

    // Remove wrong R,t.
    // If for a (R,t), a point's pos is behind the camera, then this is wrong.
    // see: https://github.com/opencv/opencv/blob/master/modules/calib3d/src/homography_decomp.cpp
    vector<Mat> res_Rs, res_ts, res_normals;
    Mat possibleSolutions; //Use print_MatProperty to know its type: 32SC1
    filterHomographyDecompByVisibleRefpoints(Rs, normals, pts_on_nplane1, pts_on_nplane2, possibleSolutions);
    for (int i = 0; i < possibleSolutions.rows; i++)
    {
        int idx = possibleSolutions.at<int>(i, 0);
        res_Rs.push_back(Rs[idx]);
        res_ts.push_back(ts[idx]);
        res_normals.push_back(normals[idx]);
    }

    // return
    Rs = res_Rs;
    ts = res_ts;
    normals = res_normals;
}

void estiMotionByHomography(const vector<Point2f> &pts_in_img1,
                            const vector<Point2f> &pts_in_img2,
                            const Mat &camera_intrinsics,
                            vector<Mat> &Rs, vector<Mat> &ts, vector<Mat> &normals,
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
    // Normalize t
    for (auto &t : ts)
    {
        t = t / sqrt(t.at<double>(1, 0) * t.at<double>(1, 0) + t.at<double>(2, 0) * t.at<double>(2, 0) +
                     t.at<double>(0, 0) * t.at<double>(0, 0));
    }
    // Try to remove wrong solutions
    removeWrongRtOfHomography(pts_in_img1, pts_in_img2, camera_intrinsics, Rs, ts, normals);

    // Print result
    if (PRINT_DEBUG_RESULT)
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

double computeEpipolarConsError(
    const vector<Point2f> &pts1, const vector<Point2f> &pts2,
    const Mat &R, const Mat &t, const Mat &K)
{
    double err = 0;
    int N = pts1.size();
    if (N == 0)
        return 99999999999999;
    for (int i = 0; i < N; i++)
    {
        double e = computeEpipolarConsError(pts1[i], pts2[i], R, t, K);
        err += e * e;
    }
    return sqrt(err / N);
}
double ptPosInNormPlane(const Point3f &pt_3d_pos_in_cam1,
                        const Mat &R_cam2_to_cam1, const Mat &t_cam2_to_cam1,
                        Point2f &pt_pos_on_cam1_nplane, double &depth1,
                        Point2f &pt_pos_on_cam2_nplane, double &depth2)
{
    Point3f pts3dc1 = pt_3d_pos_in_cam1;

    // cam1
    depth1 = pts3dc1.z;
    Point2f pts3dc1_norm(pts3dc1.x / depth1, pts3dc1.y / depth1);

    // cam2
    Mat tmp = t_cam2_to_cam1 +
              R_cam2_to_cam1 * (Mat_<double>(3, 1) << pts3dc1.x, pts3dc1.y, pts3dc1.z);
    Point3f pts3dc2(tmp.at<double>(0, 0), tmp.at<double>(1, 0), tmp.at<double>(2, 0));
    depth2 = pts3dc2.z;
    Point2f pts3dc2_norm(pts3dc2.x / depth2, pts3dc2.y / depth2);

    // return
    pt_pos_on_cam1_nplane = pts3dc1_norm;
    pt_pos_on_cam2_nplane = pts3dc2_norm;
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

vector<int> getIntersection(vector<int> v1, vector<int> v2)
{
    int comb_len = v1.size() + v2.size();
    std::vector<int> v(comb_len);
    std::vector<int>::iterator it;
    it = std::set_intersection(
        v1.begin(), v1.end(),
        v2.begin(), v2.end(),
        v.begin());
    v.resize(it - v.begin());
    return v;
}

Mat Point3f_to_Mat(const Point3f &p)
{
    return (Mat_<double>(3, 1) << p.x, p.y, p.z);
}
Mat Point2f_to_Mat(const Point2f &p)
{
    return (Mat_<double>(2, 1) << p.x, p.y);
}

Point2f cam2pixel(const Point3f &p, const Mat &K)
{
    return Point2f(
        K.at<double>(0, 0) * p.x / p.z + K.at<double>(0, 2),
        K.at<double>(1, 1) * p.y / p.z + K.at<double>(1, 2));
}

Point2f cam2pixel(const Mat &p, const Mat &K)
{
    Mat p0 = K * p;                    // project onto image
    Mat pp = p0 / p0.at<double>(2, 0); // normalize
    return Point2f(pp.at<double>(0, 0), pp.at<double>(1, 0));
}

double calcError(const Point2f &p1, const Point2f &p2)
{
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}
double calcDist(const Point2f &p1, const Point2f &p2)
{
    double dx = p1.x - p2.x, dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

} // namespace my_geometry