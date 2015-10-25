/*
 * main.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: jonathan
 */

#define DEBUG 1

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <map>
#include <cmath>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "glog/logging.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/local_parameterization.h"

#include <opencv2/opencv.hpp>

#include "se3quat.h"
#include "CameraIntrinsics.h"
#include "MockData.h"
#include "MarkerAlignmentProblem.h"
#include "math_constants.h"

// Visualization
#include "Viewer3D.h"

using namespace G2D;
using namespace std;
using namespace Eigen;

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
struct CostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

void CeresSolver()
{
	// The variable to solve for with its initial value.
    double initial_x = 5.0;
    double x = initial_x;

    // Build the problem.
    ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
            << " -> " << x << "\n";
}

template <typename ProjModel>
void GetMockData(
        vector<LidarObservation,
               Eigen::aligned_allocator<LidarObservation>> &observations,
        vector<LidarMarker> &lidarMarkers,
        vector<FramePose> &poses,
        SE3Quat &solution,
        const ProjModel* intrinsics)
{
    MockData dataProvider;

    // Generate the LIDAR marker positions
    dataProvider.LidarMarkers(lidarMarkers);

    // Generate the trajectory the rig took. 10 poses, 2 meter radius
    dataProvider.Trajectory_Circle(poses, 10, 8);

    // Generate the observations the rig would have seen with this trajectory
    dataProvider.Observations<ProjModel>(observations, lidarMarkers, poses, intrinsics, nullptr);

    // Generate the solution we should arrive at
    dataProvider.Solution(solution);

    // Transform all the poses by the solution
    SE3Quat rig2lidar = solution.inverse();
    for (int i = 0; i < poses.size(); i++)
    {
        SE3Quat &pose = *poses[i].pose;
        *poses[i].pose = rig2lidar * pose;
    }
}

template <typename ProjModel>
void CreateMockCameraIntrinsics(ProjModel** ppIntrinsics)
{
    MockData dataProvider;
    dataProvider.CameraModel<ProjModel>(ppIntrinsics);
}

int main(int /*argc*/, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    G2D::Viewer3D* pViewer = new G2D::Viewer3D();
    pViewer->InitializeAndRunAsync();
    pViewer->MaybeYieldToViewer();

    vector<LidarObservation,
           Eigen::aligned_allocator<LidarObservation>> observations;
    vector<LidarMarker> lidarMarkers;
    vector<FramePose> poses;
    shared_ptr<LinearCameraIntrinsics> spIntrinsics;
    SE3Quat gt_solution;

    // Get the sensor data
    LinearCameraIntrinsics* pIntrinsics = nullptr;
    CreateMockCameraIntrinsics<LinearCameraIntrinsics>(&pIntrinsics);
    spIntrinsics.reset(pIntrinsics);
    GetMockData<LinearCameraIntrinsics>(
            observations,
            lidarMarkers,
            poses,
            gt_solution,
            spIntrinsics.get());

    // Render the LIDAR point cloud
    {
        size_t i = 0;
        pViewer->AddPoints(
                 [&](Eigen::Vector3d & pt, G2D::ViewerColor & color) -> bool
                 {
                     if (i >= lidarMarkers.size())
                         return false;

                     pt = lidarMarkers[i].pos;
                     color = G2D::ViewerColors::Banana;
                     ++i;

                     return true;
                 });
    }

    // Setup dictionaries to look up poses/lidar markers by ids
    std::map<int, LidarMarker> lidarMarkers_dict;
    std::map<int, FramePose> poses_dict;
    for (size_t i = 0; i < lidarMarkers.size(); i++)
    {
        LidarMarker &m = lidarMarkers[i];
        lidarMarkers_dict[m.id] = m;
    }
    for (size_t i = 0; i < poses.size(); i++)
    {
        FramePose &p = poses[i];
        poses_dict[p.frame_id] = p;
    }

    // Setup the initial guess
    Eigen::Vector3d solution_t;
    Eigen::Vector4d solution_r;
    solution_r[0] = 0.0; // x
    solution_r[1] = 0.0; // y
    solution_r[2] = 0.0; // z
    solution_r[3] = 1.0; // w

    // Render the initial guess
    std::cerr << "Rendering initial guess" << std::endl;
    for (size_t i = 0; i < poses.size(); i++)
    {
        SE3Quat currentSolution(Eigen::Quaterniond(solution_r), solution_t);
        SE3Quat xformedPose = currentSolution * (*poses[i].pose);
        pViewer->AddFrustum(xformedPose.rotation(), xformedPose.translation());
    }
    pViewer->MaybeYieldToViewer();

    // Setup the solver
    // Add a residual block for every pose that has an observation
    ceres::Problem problem;

//    ceres::LocalParameterization* quat_plus = new ceres::AutoDiffLocalParameterization<QuaternionPlus, 4, 3>;
    ceres::LocalParameterization* quat_plus = new ceres::QuaternionParameterization;

    for (size_t o = 0; o < observations.size(); o++)
    {
        LidarObservation &obs = observations[o];

        LidarReprojectionError<LinearCameraIntrinsics>* pProblem =
                new LidarReprojectionError<LinearCameraIntrinsics>(
                    obs.uv,
                    lidarMarkers_dict[obs.marker_id].pos,
                    poses_dict[obs.frame_id].pose,
                    spIntrinsics.get());

        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<LidarReprojectionError<LinearCameraIntrinsics>, 2, 4, 3>(
                        pProblem);
        problem.AddResidualBlock(cost_function, NULL, solution_r.data(), solution_t.data());
        problem.SetParameterization(solution_r.data(), quat_plus);
    }

    // Create a callback for visualization
    G2D::ItrCallback callback(
            pViewer,
            solution_r,
            solution_t,
            poses.data(),
            static_cast<unsigned int>(poses.size()));

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.min_linear_solver_iterations = 1;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = true;
    options.update_state_every_iteration = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;

    options.callbacks.push_back(&callback);

    // Initialized residuals/jacobians
    ceres::Problem::EvaluateOptions evaluateOptions;
    double cost[2];
    vector<double> residuals;
    bool evaluate_result = problem.Evaluate(evaluateOptions, cost, &residuals, NULL, NULL);

    if(!evaluate_result)
    {
        cerr << "Failed to evaluate cost function initially" << endl;
    }

    // Print out evaluate results
    cout << "Residuals:" << endl;
    for (size_t i = 0; i < residuals.size(); i+=2)
    {
        cout << (i/2 + 1) << ": " << residuals[i] << " " << residuals[i+1] << endl;
    }

    // Optimize away!
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.message << "\n";
    std::cout << summary.FullReport() << "\n";
//    std::cout << "x : " << initial_x
//            << " -> " << x << "\n";

#ifdef DEBUG
    double solution_r_d[4];
    double solution_t_d[3];

    ExtractVec(solution_r_d, solution_r);
    ExtractVec(solution_t_d, solution_t);
#endif

    Eigen::Quaterniond solution_final_q(solution_r);
    SE3Quat solution_final(solution_final_q, solution_t);
    SE3Quat test_solution = solution_final * gt_solution;

    // Check if the test_solution is identity
    const double epsilon_r = 1e-4;
    const double epsilon_t = 1e-4;

    Eigen::AngleAxisd test_a(test_solution.rotation());
    double test_r = test_a.angle();
    if (epsilon_r < std::abs(test_r - PI)) test_r = std::abs(test_r - PI);
    cout << "Rotation test epsilon = " << epsilon_r << endl;
    cout << "Test solution rotation = " << test_r << endl;
    if (test_r > epsilon_r)
    {
        cout << "Rotation test Failed" << endl;
    }
    else
    {
        cout << "Rotation test passed" << endl;
    }

    Eigen::Vector3d test_t(test_solution.translation());
    cout << "Translation test epsilon = " << epsilon_t << endl;
    printf("Test solution translation = (%f, %f, %f)\n", test_t[0], test_t[2], test_t[2]);
    if (epsilon_t < test_t[0] || epsilon_t < test_t[1] || epsilon_t < test_t[2])
    {
        cout << "Translation test Failed" << endl;
    }
    else
    {
        cout << "Translation test passed" << endl;
    }

    // TODO: Visualize the output
//    Viewer3D* pViewer = nullptr;
//    bool success = Viewer3D::CreateAndInit(&pViewer);
//    if(!success)
//    {
//        cout << "Failed to create viewer" << endl;
//    }
//    else
//    {
//
//    }

    std::cerr << "Press space to finish" << std::endl;
    pViewer->MaybeYieldToViewer();

	return 0;
}

