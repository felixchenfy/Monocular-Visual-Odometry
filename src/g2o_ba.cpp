
#include "my_optimization/g2o_ba.h"
#include "my_basics/eigen_funcs.h"

#include <cmath>
#include <stdio.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono> // timer

namespace my_optimization
{

void bundleAdjustment(
    vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat &K,
    Mat &T_world_to_cam_cv)
{
    const Mat pose_before_optimization = T_world_to_cam_cv.clone();

    // Change pose format from OpenCV to Sophus::SE3
    Mat T_cam_to_world_cv = T_world_to_cam_cv.inv();
    Sophus::SE3 T_cam_to_world = my_basics::transT_cv2sophus(T_cam_to_world_cv);

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
    vector<g2o::VertexSBAPointXYZ *> points_3d_g2o;
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
    T_world_to_cam_cv = my_basics::transT_sophus2cv(T_cam_to_world).inv(); // Change data format back to OpenCV

    cout << "\nBefore Bundle Adjustment:\n"
         << pose_before_optimization << endl;
    cout << "After Bundle Adjustment:\n"
         << T_world_to_cam_cv << endl
         << endl;

    // 2. Points 3d world pos
    int N = points_3d.size();
    for (int i = 0; i < N; i++)
    {
        Eigen::Vector3d p = points_3d_g2o[i]->estimate();

        if (0)
        { // Print
            // cout << typeid(p).name() <<endl; // print this: N5Eigen6MatrixIdLi3ELi1ELi0ELi3ELi1EEE
            cout << "point " << i << " pose:" << points_3d[i] << endl;
            cout << "point " << i << " pose:" << p(0, 0) << "," << p(1, 0) << "," << p(2, 0) << endl;
            // A result is at below. Change of digit happens at 0.0001 level.
            // [-0.188413, -0.212443, 0.940536] -> [-0.188231,-0.211423,0.940817]
        }
        points_3d[i].x = p(0, 0);
        points_3d[i].y = p(1, 0);
        points_3d[i].z = p(2, 0);
    }
}

} // namespace my_optimization