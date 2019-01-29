
#include "my_optimization/g2o_ba.h"

#include <cmath>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>

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
#include <chrono> // timer

namespace my_optimization
{
// Declaration of some functions used only in this script
Sophus::SE3 transT_cv2sophus(const cv::Mat &T_cv);
cv::Mat transT_sophus2cv(const Sophus::SE3 &T_sophus);
}

namespace my_optimization
{
// ------------------------- Assistant Functions -------------------------

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

    return my_basics::transRt2T(cv_R, cv_t);
}

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
    vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat &K,
    Mat &T_world_to_cam_cv)
{
    const Mat pose_before_optimization = T_world_to_cam_cv.clone();
    
    // Change pose format from OpenCV to Sophus::SE3
    Mat T_cam_to_world_cv = T_world_to_cam_cv.inv();
    Sophus::SE3 T_cam_to_world = transT_cv2sophus(T_cam_to_world_cv);

    // Init g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block; // dim(pose) = 6, dim(landmark) = 3

    // ---- old version ----
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // solver for linear equation
    Block *solver_ptr = new Block(linearSolver);                                                   // solver for matrix block
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    // ---- new version ---- 
    // (Once the old version couldn't work, so I copied from somewhere this new version. However, this somewhat couldn't work again.)
    // std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverCSparse<Block::PoseMatrixType>());
    // std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver))); 
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // -- Add vertex: parameters to optimize

    // Camera pose
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
        T_cam_to_world.rotation_matrix(),
        T_cam_to_world.translation()));

    optimizer.addVertex(pose);

    // Points pos in world frame
    int index = 1;
    vector<g2o::VertexSBAPointXYZ*> points_3d_g2o;
    for (const Point3f &p : points_3d) // landmarks
    {
        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
        points_3d_g2o.push_back(point);
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex(point);
    }

    // Parameter: camera intrinsics
    g2o::CameraParameters *camera = new g2o::CameraParameters(
        K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // -- Add edges, which define the error/cost function.
    index = 1;
    for (const Point2f &p : points_2d)
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

    // -- Optimize
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

    // -- Final: get the result from solver 

    // 1. Camera pose
    T_cam_to_world = Sophus::SE3(
        pose->estimate().rotation(),
        pose->estimate().translation());
    // Eigen::Matrix4d T_cam_to_world = Eigen::Isometry3d(pose->estimate()).matrix();
    T_world_to_cam_cv = transT_sophus2cv(T_cam_to_world).inv(); // Change data format back to OpenCV

    cout << "\nBefore Bundle Adjustment:\n"
         << pose_before_optimization << endl;
    cout << "After Bundle Adjustment:\n"
         << T_world_to_cam_cv << endl
         << endl;
         
    // 2. Points 3d world pos
    int N=points_3d.size();
    for (int i=0;i<N;i++){
        Eigen::Vector3d p = points_3d_g2o[i]->estimate();
        
        if (0){ // Print
            // cout << typeid(p).name() <<endl; // print this: N5Eigen6MatrixIdLi3ELi1ELi0ELi3ELi1EEE
            cout << "point "<<i<<" pose:"<<points_3d[i]<<endl;
            cout << "point "<<i<<" pose:"<<p(0,0)<<","<<p(1,0)<<","<<p(2,0)<<endl;
            // A result is at below. Change of digit happens at 0.0001 level.
            // [-0.188413, -0.212443, 0.940536] -> [-0.188231,-0.211423,0.940817]
        }
        points_3d[i].x=p(0,0);
        points_3d[i].y=p(1,0);
        points_3d[i].z=p(2,0);
    }

}

} // namespace my_optimization