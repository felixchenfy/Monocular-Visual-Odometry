/* This script is mainly copied and then modified from Chapter 7 of Dr. Xiang Gao's book. Link is here:
https://github.com/gaoxiang12/slambook/blob/master/ch7/pose_estimation_3d2d.cpp
*/

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

#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <chrono> // timer

namespace my_optimization
{

void optimizeSingleFrame(
    const vector<Point2f> &points_2d,
    const Mat &K,
    vector<Point3f*> &points_3d,
    Mat &pose_src,
    bool fix_map_pts, bool update_map_pts)
{
    const Mat pose_src0 = pose_src.clone();

    // Change pose format from OpenCV to Sophus::SE3
    Mat T_cam_to_world_cv = pose_src.inv();
    Sophus::SE3 T_cam_to_world = my_basics::transT_cv2sophus(T_cam_to_world_cv);

    // Init g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block; // dim(pose) = 6, dim(landmark) = 3
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // solver for linear equation
    Block *solver_ptr = new Block(linearSolver);                                                   // solver for matrix block
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
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
    vector<g2o::VertexSBAPointXYZ *> g2o_points_3d;
    for (const Point3f* p : points_3d) // landmarks
    {
        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
        if(fix_map_pts)
            point->setFixed(true);
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p->x, p->y, p->z));
        point->setMarginalized(true);
        optimizer.addVertex(point);
        g2o_points_3d.push_back(point);
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
    int optimize_iters = 50;
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
    pose_src = my_basics::transT_sophus2cv(T_cam_to_world).inv(); // Change data format back to OpenCV
    
    printf("Cam t: Before:{%.5f,%.5f,%.5f}, After:{%.5f,%.5f,%.5f}\n",
        pose_src0.at<double>(0,0), pose_src0.at<double>(1,0), pose_src0.at<double>(2,0),
        pose_src.at<double>(0,0), pose_src.at<double>(1,0), pose_src.at<double>(2,0));


    // 2. Points 3d world pos
    int N = points_3d.size();
    Point3f point_src0(*points_3d[0]);
    Eigen::Vector3d point_dst0 = g2o_points_3d[0]->estimate();
    for (int i = 0; update_map_pts && i < N; i++)
    {
        Eigen::Vector3d p = g2o_points_3d[i]->estimate();
        points_3d[i]->x = p(0, 0);
        points_3d[i]->y = p(1, 0);
        points_3d[i]->z = p(2, 0);
    }

    printf("Point 0: Before:{%.5f,%.5f,%.5f}, After:{%.5f,%.5f,%.5f}\n",
                point_src0.x, point_src0.y, point_src0.z,
                point_dst0(0, 0), point_dst0(1, 0), point_dst0(2, 0));

}


//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

void bundleAdjustment(
    const vector<vector<Point2f *>> &v_pts_2d,
    const vector<vector<int>> &v_pts_2d_to_3d_idx,
    const Mat &K,
    unordered_map<int, Point3f *> &pts_3d,
    vector<Mat *> &v_camera_g2o_poses,
    bool fix_map_pts, bool update_map_pts)
{

    // Change pose format from OpenCV to Sophus::SE3
    int num_frames = v_camera_g2o_poses.size();
    // vector<Sophus::SE3, aligned_allocator<Sophus::SE3>> v_T_cam_to_world;
    vector<Sophus::SE3> v_T_cam_to_world;
    for (int i = 0; i < num_frames; i++)
    {
        v_T_cam_to_world.push_back(
            my_basics::transT_cv2sophus((*v_camera_g2o_poses[i]).inv()));
    }

    // Init g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block; // dim(pose) = 6, dim(landmark) = 3
    Block::LinearSolverType *linearSolver;
    // linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // solver for linear equation
    linearSolver =  new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(linearSolver); // solver for matrix block
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // -- Add vertex: parameters to optimize
    int vertex_id = 0;
    // Camera pose
    vector<g2o::VertexSE3Expmap *> g2o_poses;
    for (int ith_frame = 0; ith_frame < num_frames; ith_frame++)
    {
        g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // camera pose
        pose->setId(vertex_id++);
        // if (num_frames > 1 && ith_frame == num_frames - 1)
            // pose->setFixed(true); // Fix the last one -- which is the earliest frame
        pose->setEstimate(g2o::SE3Quat(
            v_T_cam_to_world[ith_frame].rotation_matrix(),
            v_T_cam_to_world[ith_frame].translation()));
        optimizer.addVertex(pose);
        g2o_poses.push_back(pose);
    }
    // Parameter: camera intrinsics
    g2o::CameraParameters *camera = new g2o::CameraParameters(
        K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // Points pos in world frame
    unordered_map<int, g2o::VertexSBAPointXYZ *> g2o_points_3d;
    unordered_map<int, int> pts3dID_to_vertexID;
    for (auto it = pts_3d.begin(); it != pts_3d.end(); it++) // landmarks
    {
        int pt3d_id = it->first;
        Point3f *p = it->second;
        
        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
        point->setId(vertex_id);
        if(fix_map_pts)
            point->setFixed(true);

        pts3dID_to_vertexID[pt3d_id] = vertex_id;
        vertex_id++;
        point->setEstimate(Eigen::Vector3d(p->x, p->y, p->z));
        point->setMarginalized(true); // g2o must set marg
        optimizer.addVertex(point);
        g2o_points_3d[pt3d_id] = point;
    }
    printf("Number of frames = %d, 3d points = %d\n", num_frames, vertex_id - num_frames);

    // -- Add edges, which define the error/cost function.
    int edge_id = 0;
    for (int ith_frame = 0; ith_frame < num_frames; ith_frame++)
    {
        int num_pts_2d = v_pts_2d[ith_frame].size();
        for (int j = 0; j < num_pts_2d; j++)
        {
            const Point2f *p = v_pts_2d[ith_frame][j];
            int pt3d_id = v_pts_2d_to_3d_idx[ith_frame][j];

            g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId(edge_id++);
            edge->setVertex(0, // XYZ point
                            dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pts3dID_to_vertexID[pt3d_id])));
            edge->setVertex(1, // camera pose
                            dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(ith_frame)));

            edge->setMeasurement(Eigen::Vector2d(p->x, p->y));
            edge->setParameterId(0, 0);
            edge->setInformation(Eigen::Matrix2d::Identity());
            // edge->setRobustKernel( new g2o::RobustKernelHuber() );
            optimizer.addEdge(edge);
        }
    }

    // -- Optimize
    bool IF_PRINT_TIME_AND_RES = false;
    int optimize_iters = 50;
    if (IF_PRINT_TIME_AND_RES)
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

    // --------------------------------------------------
    // -- Final: get the result from solver

    // 1. Camera pose
    for (int i = 0; i < num_frames; i++)
    {
        Sophus::SE3 T_cam_to_world = Sophus::SE3(
            g2o_poses[i]->estimate().rotation(),
            g2o_poses[i]->estimate().translation());
        Mat pose_src = my_basics::transT_sophus2cv(T_cam_to_world).inv(); // Change data format back to OpenCV
        pose_src.copyTo(*v_camera_g2o_poses[i]);
    }

    // 2. Points 3d world pos // This makes the performance bad
    for (auto it = pts_3d.begin(); update_map_pts && it != pts_3d.end(); it++)
    {
        int pt3d_id = it->first;
        Point3f *p = it->second;
        Eigen::Vector3d p_res = g2o_points_3d[pt3d_id]->estimate();
        p->x = p_res(0, 0);
        p->y = p_res(1, 0);
        p->z = p_res(2, 0);
    }
}

} // namespace my_optimization