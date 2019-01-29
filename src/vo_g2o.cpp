
#include <iostream>
#include <cmath>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/core/eigen.hpp>


#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

using namespace std;
using namespace cv;

// ------------------------- Assistant Functions -------------------------

cv::Mat transRt2T(const cv::Mat &R, const cv::Mat &t)
{
    cv::Mat T = (cv::Mat_<double>(4, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                 R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                 R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
                 0, 0, 0, 1);
    return T;
}

Sophus::SE3 transT_cv2sophus(const cv::Mat &T_cv)
{
    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(T_cv(cv::Rect2d(0, 0, 3, 3)), R_eigen);
    Eigen::Vector3d t_eigen(T_cv.at<double>(0, 3), T_cv.at<double>(1, 3), T_cv.at<double>(2, 3));
    Sophus::SE3 SE3_Rt(R_eigen, t_eigen);
    return SE3_Rt;
}

cv::Mat transT_sophus2cv(const Sophus::SE3 &T_sophus)
{
    Eigen::Vector3d eigen_t(T_sophus.translation());
    Eigen::Matrix3d eigen_R(T_sophus.rotation_matrix());

    cv::Mat cv_t, cv_R;
    eigen2cv(eigen_t, cv_t);
    eigen2cv(eigen_R, cv_R);

    return transRt2T(cv_R, cv_t);
}



namespace my_slam
{

// ------------------------- g2o optimization -------------------------

// void VisualOdometry::bundleAdjustment(
//     const vector<cv::Point3f> points_3d,
//     const vector<cv::Point2f> points_2d,
//     const Mat &K, // camera intrinsics
//     Mat &T_w2c_cv)
// {
//     // TODO: bool flag_camera_only = false;

//     // -- Change data format
//     Sophus::SE3 T = transT_cv2sophus(T_w2c_cv);

//     // Step 1: init g2o
//     typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block; // pose 维度为 6, landmark 维度为 3
//     Block::LinearSolverType *linearSolver;
//     if (~flag_camera_only)
//     {
//         linearSolver =
//             new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
//     }
//     else
//     {
//         linearSolver =
//             new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
//     }
//     Block *solver_ptr = new Block(linearSolver); // 矩阵块求解器
//     g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//     g2o::SparseOptimizer optimizer;
//     optimizer.setAlgorithm(solver);

//     // Step 2: add vertex
//     // Step 2.1: add camera pose "pose"

//     g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // camera pose
//     // Eigen::Matrix3d R_mat;
//     // R_mat <<
//     //       R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
//     //            R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
//     //            R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
//     pose->setId(0);
//     // pose->setEstimate ( g2o::SE3Quat ( // R_mat and t are the paras to optimize
//     //     R_mat,
//     //     Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
//     // ) );
//     pose->setEstimate(g2o::SE3Quat(
//         T_cam_to_world.rotation_matrix(),
//         T_cam_to_world.translation()));
//     optimizer.addVertex(pose);

//     // Step 2.2: add landmarks, i.e. keypoins' pose
//     // int index;
//     // if (~flag_camera_only)
//     // {
//     //     index = 1;
//     //     for (const cv::Point3f p : points_3d) // landmarks
//     //     {
//     //         g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
//     //         point->setId(index++);
//     //         point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
//     //         point->setMarginalized(true); // g2o 中必须设置 marg 参见第十讲内容
//     //         optimizer.addVertex(point);   // TO DO : put point into a vector, and point->estimate to get its value
//     //     }
//     // }

//     // Step 3.0 : add camera intrinsics "K" of the curr image,
//     //            so p_world could be mapped to p_image
//     g2o::CameraParameters *camera = new g2o::CameraParameters(
//         K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
//     camera->setId(0);
//     optimizer.addParameter(camera);

//     // Step 3.1: add edges

//     Version 1
//     index = 1;
//     for ( const cv::Point2f p:points_2d )
//     {
//         g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
//         edge->setId ( index );
//         edge->setVertex ( 0, dynamic_cast<g2o::VertexSBAPointXYZ*> (
//              optimizer.vertex ( index ) ) );
//         edge->setVertex ( 1, pose );
//         edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
//         edge->setParameterId ( 0,0 );
//         edge->setInformation ( Eigen::Matrix2d::Identity() );
//         optimizer.addEdge ( edge );
//         index++;
//     }

//     // Version 2: Same as above, but using customed class
//     // for (int i = 0; i < points_2d.size(); i++)
//     // {
//     //     // 3D -> 2D projection
//     //     EdgeProjectXYZ2UVPoseOnly *edge = new EdgeProjectXYZ2UVPoseOnly();
//     //     edge->setId(i);
//     //     edge->setVertex(0, pose);
//     //     edge->camera_ = curr_img_->camera_.get(); // .get() returns the ptr of the object
//     //     edge->point_ = Vector3d(points_3d[i].x, points_3d[i].y, points_3d[i].z);
//     //     edge->setMeasurement(Vector2d(points_2d[i].x, points_2d[i].y));
//     //     edge->setInformation(Eigen::Matrix2d::Identity());
//     //     optimizer.addEdge(edge);
//     // }

//     // Step 4: optimize
//     bool IF_PRINT_TIME = false;
//     int optimize_iters = 100;
//     if (IF_PRINT_TIME)
//     {
//         chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//         optimizer.setVerbose(true);
//         optimizer.initializeOptimization();
//         optimizer.optimize(optimize_iters);
//         chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//         chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//         cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
//     }
//     else
//     {
//         optimizer.initializeOptimization();
//         optimizer.optimize(optimize_iters);
//     }
//     // cout<<endl<<"after optimization:"<<endl;
//     // cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;

//     // Final: update T_cam_to_world
//     T_cam_to_world = SE3(
//         pose->estimate().rotation(),
//         pose->estimate().translation());

//     // -- Change data format back
//     T_w2c_cv = transT_sophus2cv(T_cam_to_world);
// }

void bundleAdjustment(
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat &K,
    Mat &T_world_to_cam_cv)
// Mat& R, Mat& t )
{

    Mat T_cam_to_world_cv = T_world_to_cam_cv.inv();
    Sophus::SE3 T_cam_to_world = transT_cv2sophus(T_cam_to_world_cv);

    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block; // pose 维度为 6, landmark 维度为 3

    // ---- old version ----
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block *solver_ptr = new Block(linearSolver);                                                   // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    // ---- new version ----
    // std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());
    // std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));     // 矩阵块求解器
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
        T_cam_to_world.rotation_matrix(),
        T_cam_to_world.translation()));

    optimizer.addVertex(pose);

    int index = 1;
    for (const Point3f p : points_3d) // landmarks
    {
        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex(point);
    }

    // parameter: camera intrinsics
    g2o::CameraParameters *camera = new g2o::CameraParameters(
        K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // edges
    index = 1;
    for (const Point2f p : points_2d)
    {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(index)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    // Step 4: optimize
    bool IF_PRINT_TIME = false;
    int optimize_iters = 100;
    if (IF_PRINT_TIME)
    {
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(optimize_iters);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    }
    else
    {
        optimizer.initializeOptimization();
        optimizer.optimize(optimize_iters);
    }
    cout << endl
         << "after optimization:" << endl;
    cout << "T=" << endl
         << Eigen::Isometry3d(pose->estimate()).matrix() << endl;

    // Final: update T_cam_to_world
    T_cam_to_world = Sophus::SE3(
        pose->estimate().rotation(),
        pose->estimate().translation());

    // -- Change data format back
    T_cam_to_world_cv = transT_sophus2cv(T_cam_to_world);
    T_world_to_cam_cv = T_cam_to_world_cv.inv();
}

} // namespace my_slam
