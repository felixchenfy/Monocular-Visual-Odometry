
#include "my_slam/geometry/camera.h"

namespace my_slam
{
namespace geometry
{
// ---------------- transformation ----------------

cv::Point2f pixel2camNormPlane(const cv::Point2f &p, const cv::Mat &K)
{
    return cv::Point2f(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}
cv::Point3f pixel2cam(const cv::Point2f &p, const cv::Mat &K, double depth)
{
    return cv::Point3f(
        depth * (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        depth * (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1),
        depth);
}
cv::Point2f cam2pixel(const cv::Point3f &p, const cv::Mat &K)
{
    return cv::Point2f(
        K.at<double>(0, 0) * p.x / p.z + K.at<double>(0, 2),
        K.at<double>(1, 1) * p.y / p.z + K.at<double>(1, 2));
}

cv::Point2f cam2pixel(const cv::Mat &p, const cv::Mat &K)
{
    cv::Mat p0 = K * p;                    // project onto image
    cv::Mat pp = p0 / p0.at<double>(2, 0); // normalize
    return cv::Point2f(pp.at<double>(0, 0), pp.at<double>(1, 0));
}

} // namespace geometry
} // namespace my_slam
