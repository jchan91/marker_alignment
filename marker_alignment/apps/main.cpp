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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glog/logging.h"
#include "ceres/ceres.h"

#include "se3quat.h"
#include "CameraState.h"
#include "CameraIntrinsics.h"
#include "MockData.h"

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

struct LidarReprojectionError
{
    LidarReprojectionError(
        Eigen::Vector2d observedMarker,
        Eigen::Vector3d lidarMarker,
        std::shared_ptr<G2D::SE3Quat> pPose,
        CameraIntrinsics* pIntrinsics) :
            m_observedMarker(observedMarker),
            m_lidarMarker(lidarMarker),
            m_pPose(pPose),
            m_pIntrinsics(pIntrinsics)
    {
    }

    template <typename T>
    bool operator() (const T* lidar2world_r, const T* lidar2world_t, T* residuals) const
    {
        Vector6d l2w_vec;
        l2w_vec[0] = lidar2world_t[0];
        l2w_vec[1] = lidar2world_t[1];
        l2w_vec[2] = lidar2world_t[2];
        l2w_vec[0] = lidar2world_r[0];
        l2w_vec[1] = lidar2world_r[1];
        l2w_vec[2] = lidar2world_r[2];


        G2D::SE3Quat l2w_se3;
        l2w_se3.fromMinimalVector(l2w_vec);

        Eigen::Vector3d lidarMarker_world = l2w_se3 * m_lidarMarker;
        Eigen::Vector3d lidarMarker_camera = *m_pPose * lidarMarker_world;

        Eigen::Vector2d uv;
        Eigen::Vector2d xy;

        // Convert lidarMarker xyz into homogeneous coordinates
        xy(0) = lidarMarker_camera(0) / lidarMarker_camera(2);
        xy(1) = lidarMarker_camera(1) / lidarMarker_camera(2);

        bool success = m_pIntrinsics->Project(static_cast<double*>(uv.data()), static_cast<const double*>(xy.data()));

        if(!success)
        {
            return false;
        }

        size_t width = m_pIntrinsics->Width();
        size_t height = m_pIntrinsics->Height();

        Eigen::Vector2d pixel;
        pixel(0) = uv(0) * width;
        pixel(1) = uv(1) * height;

        Eigen::Vector2d error = m_observedMarker - pixel;

        residuals[0] = error[0];
        residuals[1] = error[1];

        return true;
    }

private:
    Eigen::Vector2d m_observedMarker;
    Eigen::Vector3d m_lidarMarker;
    std::shared_ptr<G2D::SE3Quat> m_pPose;
    G2D::CameraIntrinsics* m_pIntrinsics;
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

void GetMockData(
        vector<LidarObservation> &observations,
        vector<LidarMarker> &lidarMarkers,
        vector<FramePose> &poses,
        std::shared_ptr<CameraIntrinsics> pIntrinsics,
        SE3Quat &solution)
{
    MockData dataProvider;

    // Create a fake camera model
    CameraIntrinsics* intrinsics = nullptr;
    dataProvider.CameraModel(&intrinsics);
    pIntrinsics.reset(intrinsics);

    // Generate the LIDAR marker positions
    dataProvider.LidarMarkers(lidarMarkers);

    // Generate the trajectory the rig took. 10 poses, 2 meter radius
    dataProvider.Trajectory_Circle(poses, 4, 2);

    // Generate the observations the rig would have seen with this trajectory
    dataProvider.Observations(observations, lidarMarkers, poses, intrinsics, nullptr);

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

int main(int /*argc*/, char** /*argv[]*/)
{
    vector<LidarObservation> observations;
    vector<LidarMarker> lidarMarkers;
    vector<FramePose> poses;
    shared_ptr<CameraIntrinsics> pIntrinsics;
    SE3Quat gt_solution;

    // Get the sensor data
    GetMockData(observations, lidarMarkers, poses, pIntrinsics, gt_solution);

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

    // Setup the solver
    // Add a residual block for every pose that has an observation
    Eigen::Vector3d solution_t;
    Eigen::Vector3d solution_r;
    ceres::Problem problem;
    for (size_t o = 0; o < observations.size(); o++)
    {
        LidarObservation &obs = observations[o];

        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<LidarReprojectionError, 2, 3, 3>(new LidarReprojectionError(
                        obs.uv,
                        lidarMarkers_dict[obs.marker_id].pos,
                        poses_dict[obs.frame_id].pose,
                        pIntrinsics.get()));
        problem.AddResidualBlock(cost_function, NULL, solution_r.data(), solution_t.data());
    }

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
//    std::cout << "x : " << initial_x
//            << " -> " << x << "\n";


	return 0;
}

