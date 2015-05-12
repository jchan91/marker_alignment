/*
 * main.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: jonathan
 */


#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <map>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glog/logging.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "se3quat.h"
#include "CameraIntrinsics.h"
#include "MockData.h"
#include "MarkerAlignmentProblem.h"

using namespace G2D;
using namespace std;
using namespace Eigen;

#define DEBUG 1

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
        vector<LidarObservation> &observations,
        vector<LidarMarker> &lidarMarkers,
        vector<FramePose> &poses,
        SE3Quat &solution,
        const ProjModel* intrinsics)
{
    MockData dataProvider;

    // Generate the LIDAR marker positions
    dataProvider.LidarMarkers(lidarMarkers);

    // Generate the trajectory the rig took. 10 poses, 2 meter radius
    dataProvider.Trajectory_Circle(poses, 4, 2);

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

    vector<LidarObservation> observations;
    vector<LidarMarker> lidarMarkers;
    vector<FramePose> poses;
    shared_ptr<LinearCameraIntrinsics> spIntrinsics;
    SE3Quat gt_solution;

    // Get the sensor data
    LinearCameraIntrinsics* pIntrinsics = nullptr;
    CreateMockCameraIntrinsics<LinearCameraIntrinsics>(&pIntrinsics);
    spIntrinsics.reset(pIntrinsics);
    GetMockData<LinearCameraIntrinsics>(observations, lidarMarkers, poses, gt_solution, spIntrinsics.get());

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
    SE3Quat solution_init_guess;
    Eigen::Quaterniond solution_init_guess_q = solution_init_guess.rotation();
    Eigen::Vector3d solution_init_guess_t = solution_init_guess.translation();
    Eigen::Vector3d solution_t = solution_init_guess_t;
    Eigen::Vector4d solution_r;
    solution_r[0] = solution_init_guess_q.w();
    solution_r[1] = solution_init_guess_q.x();
    solution_r[2] = solution_init_guess_q.y();
    solution_r[3] = solution_init_guess_q.z();

    // Setup the solver
    // Add a residual block for every pose that has an observation
    ceres::Problem problem;

    ceres::LocalParameterization* quat_plus = new ceres::AutoDiffLocalParameterization<QuaternionPlus, 4, 3>;

    for (size_t o = 0; o < observations.size(); o++)
    {
        LidarObservation &obs = observations[o];

        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<LidarReprojectionError<LinearCameraIntrinsics>, 2, 4, 3>(
                        new LidarReprojectionError<LinearCameraIntrinsics>(
                        obs.uv,
                        lidarMarkers_dict[obs.marker_id].pos,
                        poses_dict[obs.frame_id].pose,
                        spIntrinsics.get()));
        problem.AddResidualBlock(cost_function, NULL, solution_r.data(), solution_t.data());
        problem.SetParameterization(solution_r.data(), quat_plus);
    }

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // Initialized residuals/jacobians
    ceres::Problem::EvaluateOptions evaluateOptions;
    double cost;
    vector<double> residuals;
    problem.Evaluate(evaluateOptions, &cost, &residuals, NULL, NULL);

    // Optimize away!
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
//    std::cout << "x : " << initial_x
//            << " -> " << x << "\n";


	return 0;
}

