
#include "my_slam/geometry/epipolar_geometry.h"
#include "my_slam/basics/config.h"


#define PRINT_DEBUG_RESULT true

namespace my_slam
{
namespace geometry
{

// -------------------------------------------------------------------------------------------
// ---------------- Main: Motion from Essential and Homography; Triangulation ----------------
// -------------------------------------------------------------------------------------------

void estiMotionByEssential(
    const vector<cv::Point2f> &pts_in_img1,
    const vector<cv::Point2f> &pts_in_img2,
    const cv::Mat &camera_intrinsics,
    cv::Mat &essential_matrix,
    cv::Mat &R, cv::Mat &t, vector<int> &inliers_index)
{
    inliers_index.clear();
    cv::Mat K = camera_intrinsics;                                       // rename
    cv::Point2f principal_point(K.at<double>(0, 2), K.at<double>(1, 2)); // optical center
    double focal_length = (K.at<double>(0, 0) + K.at<double>(1, 1)) / 2; // focal length

    // -- Essential matrix
    int method = cv::RANSAC;
    static double findEssentialMat_prob = basics::Config::get<double>("findEssentialMat_prob");
    static double findEssentialMat_threshold = basics::Config::get<double>("findEssentialMat_threshold");
    // double prob = 0.99; //This param settings give big error. Tested by image0001 and image0015.
    // double threshold = 3.0;
    cv::Mat inliers_mask; //Use print_MatProperty to know its type: 8UC1
    essential_matrix = findEssentialMat(
        pts_in_img1, pts_in_img2, focal_length, principal_point,
        method, findEssentialMat_prob, findEssentialMat_threshold,
        inliers_mask);
    essential_matrix /= essential_matrix.at<double>(2, 2);
    // print_MatProperty(inliers_mask);

    // Get inliers
    for (int i = 0; i < inliers_mask.rows; i++)
    {
        if ((int)inliers_mask.at<unsigned char>(i, 0) == 1)
        {
            inliers_index.push_back(i);
        }
    }

    // Recover R,t from Essential matrix
    recoverPose(essential_matrix, pts_in_img1, pts_in_img2, R, t, focal_length, principal_point, inliers_mask);
    // Normalize t
    t = t / sqrt(t.at<double>(1, 0) * t.at<double>(1, 0) + t.at<double>(2, 0) * t.at<double>(2, 0) +
                 t.at<double>(0, 0) * t.at<double>(0, 0));
}

void removeWrongRtOfHomography(
    const vector<cv::Point2f> &pts_on_np1, const vector<cv::Point2f> &pts_on_np2,
    const vector<int> &inliers,
    vector<cv::Mat> &Rs, vector<cv::Mat> &ts, vector<cv::Mat> &normals)
{
    // Remove wrong R,t.
    // If for a (R,t), a point's pos is behind the camera, then this is wrong.
    // see: https://github.com/opencv/opencv/blob/master/modules/calib3d/src/homography_decomp.cpp
    vector<cv::Mat> res_Rs, res_ts, res_normals;
    cv::Mat possibleSolutions; //Use print_MatProperty to know its type: 32SC1
    vector<cv::Point2f> inl_pts_on_np1, inl_pts_on_np2;
    for (int idx : inliers)
    {
        inl_pts_on_np1.push_back(pts_on_np1[idx]);
        inl_pts_on_np2.push_back(pts_on_np2[idx]);
    }
    filterHomographyDecompByVisibleRefpoints(Rs, normals, inl_pts_on_np1, inl_pts_on_np2, possibleSolutions);
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

void estiMotionByHomography(const vector<cv::Point2f> &pts_in_img1,
                            const vector<cv::Point2f> &pts_in_img2,
                            const cv::Mat &camera_intrinsics,
                            cv::Mat &homography_matrix,
                            vector<cv::Mat> &Rs, vector<cv::Mat> &ts, vector<cv::Mat> &normals,
                            vector<int> &inliers_index)
{
    // https://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#findhomography
    Rs.clear();
    ts.clear();
    normals.clear();
    inliers_index.clear();
    // -- Homography matrix
    int method = cv::RANSAC;
    double ransacReprojThreshold = 3;
    cv::Mat inliers_mask; //Use print_MatProperty to know its type: 8UC1
    homography_matrix = findHomography(pts_in_img1, pts_in_img2, method, ransacReprojThreshold, inliers_mask);
    homography_matrix /= homography_matrix.at<double>(2, 2);

    // Get inliers
    inliers_index.clear();
    for (int i = 0; i < inliers_mask.rows; i++)
    {
        if ((int)inliers_mask.at<unsigned char>(i, 0) == 1)
        {
            inliers_index.push_back(i);
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
}

void doTriangulation(
    const vector<cv::Point2f> &pts_on_np1,
    const vector<cv::Point2f> &pts_on_np2,
    const cv::Mat &R_cam2_to_cam1, const cv::Mat &t_cam2_to_cam1,
    const vector<int> &inliers,
    vector<cv::Point3f> &pts3d_in_cam1)
{

    // extract inliers points
    vector<cv::Point2f> inlier_pts_on_np1, inlier_pts_on_np2;
    for (auto idx : inliers)
    {
        inlier_pts_on_np1.push_back(pts_on_np1[idx]);
        inlier_pts_on_np2.push_back(pts_on_np2[idx]);
    }
    // set up
    cv::Mat T_cam1_to_world =
        (cv::Mat_<float>(3, 4) << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0);
    cv::Mat T_cam2_to_world = basics::convertRt2T_3x4(R_cam2_to_cam1, t_cam2_to_cam1);

    // triangulartion
    cv::Mat pts4d_in_world;
    cv::triangulatePoints(
        T_cam1_to_world, T_cam2_to_world,
        inlier_pts_on_np1, inlier_pts_on_np2, pts4d_in_world);

    // change to homogeneous coords
    vector<cv::Point3f> pts3d_in_world;
    for (int i = 0; i < pts4d_in_world.cols; i++)
    {
        cv::Mat x = pts4d_in_world.col(i);
        x /= x.at<float>(3, 0); // 归一化
        cv::Point3f pt3d_in_world(
            x.at<float>(0, 0),
            x.at<float>(1, 0),
            x.at<float>(2, 0));
        pts3d_in_world.push_back(pt3d_in_world);
        // pts3d_in_world.push_back( preTranslatePoint3f(pt3d_in_world, convertRt2T(R_cam2_to_cam1,t_cam2_to_cam1).inv())); // This is wrong
        // if(i<3)cout<<"in tri, i="<<i<<","<<"p="<<p<<endl;
    }

    // return
    pts3d_in_cam1 = pts3d_in_world;
}



// ----------------------------------------------------------
// ---------------- Validation (Print error) ----------------
// ----------------------------------------------------------


double computeEpipolarConsError(
    const cv::Point2f &p1, const cv::Point2f &p2, const cv::Mat &R, const cv::Mat &t, const cv::Mat &K)
{
    cv::Point2f p_cam1 = pixel2CamNormPlane(p1, K); // point's pos in the camera frame
    cv::Mat y1 = (cv::Mat_<double>(3, 1) << p_cam1.x, p_cam1.y, 1);
    cv::Point2f p_cam2 = pixel2CamNormPlane(p2, K);
    cv::Mat y2 = (cv::Mat_<double>(3, 1) << p_cam2.x, p_cam2.y, 1);
    cv::Mat d = y2.t() * basics::skew(t) * R * y1;
    return d.at<double>(0, 0);
}

double computeEpipolarConsError(
    const vector<cv::Point2f> &pts1, const vector<cv::Point2f> &pts2,
    const cv::Mat &R, const cv::Mat &t, const cv::Mat &K)
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


// -----------------------------------------------------
// ---------------- Datatype conversion ----------------
// -----------------------------------------------------

vector<cv::Point2f> convertkeypointsToPoint2f(const vector<cv::KeyPoint> kpts)
{
    vector<cv::Point2f> pts;
    for (auto &kpt : kpts)
        pts.push_back(kpt.pt);
    return pts;
}

vector<cv::Point2f> getInlierPts(
    const vector<cv::Point2f> &pts,
    const vector<int> &inliers_idx)
{
    vector<cv::Point2f> res;
    for (auto idx : inliers_idx)
        res.push_back(pts[idx]);
    return res;
}

vector<cv::KeyPoint> getInlierKpts(
    const vector<cv::KeyPoint> &kpts,
    const vector<int> &inliers_idx)
{
    vector<cv::KeyPoint> res;
    for (auto idx : inliers_idx)
        res.push_back(kpts[idx]);
    return res;
}
void extractPtsFromMatches(
    const vector<cv::Point2f> &points_1, const vector<cv::Point2f> &points_2,
    const vector<cv::DMatch> &matches,
    vector<cv::Point2f> &pts1, vector<cv::Point2f> &pts2)
{
    pts1.clear();
    pts2.clear();
    for (auto &m : matches)
    {
        pts1.push_back(points_1[m.queryIdx]);
        pts2.push_back(points_2[m.trainIdx]);
    }
}

void extractPtsFromMatches(
    const vector<cv::KeyPoint> &keypoints_1,
    const vector<cv::KeyPoint> &keypoints_2,
    const vector<cv::DMatch> &matches,
    vector<cv::Point2f> &pts1,
    vector<cv::Point2f> &pts2)
{
    pts1.clear();
    pts2.clear();
    for (auto &m : matches)
    {
        pts1.push_back(keypoints_1[m.queryIdx].pt);
        pts2.push_back(keypoints_2[m.trainIdx].pt);
    }
}


// ----------------------------------------------------------------------------
// ---------------- Other assistant functions ----------------
// ----------------------------------------------------------------------------

double ptPosInNormPlane(const cv::Point3f &pt_3d_pos_in_cam1,
                        const cv::Mat &R_cam2_to_cam1, const cv::Mat &t_cam2_to_cam1,
                        cv::Point2f &pt_pos_on_cam1_nplane, double &depth1,
                        cv::Point2f &pt_pos_on_cam2_nplane, double &depth2)
{
    cv::Point3f pts3dc1 = pt_3d_pos_in_cam1;

    // cam1
    depth1 = pts3dc1.z;
    cv::Point2f pts3dc1_norm(pts3dc1.x / depth1, pts3dc1.y / depth1);

    // cam2
    cv::Mat tmp = t_cam2_to_cam1 +
                  R_cam2_to_cam1 * (cv::Mat_<double>(3, 1) << pts3dc1.x, pts3dc1.y, pts3dc1.z);
    cv::Point3f pts3dc2(tmp.at<double>(0, 0), tmp.at<double>(1, 0), tmp.at<double>(2, 0));
    depth2 = pts3dc2.z;
    cv::Point2f pts3dc2_norm(pts3dc2.x / depth2, pts3dc2.y / depth2);

    // return
    pt_pos_on_cam1_nplane = pts3dc1_norm;
    pt_pos_on_cam2_nplane = pts3dc2_norm;
}


} // namespace geometry
} // namespace my_slam