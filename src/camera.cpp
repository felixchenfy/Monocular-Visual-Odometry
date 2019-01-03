
#include "my_geometry/camera.h"

namespace my_geometry
{
// ---------------- transformation ----------------

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


// ---------------- Class ----------------


} // namespace my_slam
