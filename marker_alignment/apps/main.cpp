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

struct QuaternionPlus
{
    template <typename T>
    bool operator() (const T* x, const T* delta, T* x_plus_delta) const
    {
        const T squared_norm_delta = delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2];

        T q_delta[4];
        if(squared_norm_delta > T(0.0))
        {
            T norm_delta = sqrt(squared_norm_delta);
            const T sin_delta_by_delta = sin(norm_delta) / norm_delta;
            q_delta[0] = cos(norm_delta);
            q_delta[1] = sin_delta_by_delta * delta[0];
            q_delta[2] = sin_delta_by_delta * delta[1];
            q_delta[3] = sin_delta_by_delta * delta[2];
        }
        else
        {
            q_delta[0] = T(1.0);
            q_delta[1] = delta[0];
            q_delta[2] = delta[1];
            q_delta[3] = delta[2];
        }

        ceres::QuaternionProduct(q_delta, x, x_plus_delta);
        return true;
    }
};

template <typename ProjModel>
struct LidarReprojectionError
{
    LidarReprojectionError(
        Eigen::Vector2d observedMarker,
        Eigen::Vector3d lidarMarker,
        std::shared_ptr<G2D::SE3Quat> pPose,
        const ProjModel* pIntrinsics) :
            m_observedMarker(observedMarker),
            m_lidarMarker(lidarMarker),
            m_pPose(pPose),
            m_pIntrinsics(pIntrinsics)
    {
        Eigen::Quaterniond pose_q = m_pPose->rotation();
        m_pose_q[0] = pose_q.w();
        m_pose_q[1] = pose_q.x();
        m_pose_q[2] = pose_q.y();
        m_pose_q[3] = pose_q.z();
        m_pose_t = m_pPose->translation();
    }

    template <typename T>
    bool operator() (const T* lidar2world_q, const T* lidar2world_t, T* residuals) const
    {
        // Transform into the world space
        T lidarMarker_world[3];
        Eigen::Matrix<T,3,1> lidarMarker = m_lidarMarker.cast<T>();
        ceres::QuaternionRotatePoint(lidar2world_q, lidarMarker.data(), lidarMarker_world);
        for (int i = 0; i < 3; i++) lidarMarker_world[i] += lidar2world_t[i];

        // Transform into the camera
        T lidarMarker_camera[3];
        Eigen::Matrix<T,4,1> pose_q = m_pose_q.cast<T>();
        Eigen::Matrix<T,3,1> pose_t = m_pose_t.cast<T>();
        ceres::QuaternionRotatePoint(pose_q.data(), lidarMarker_world, lidarMarker_camera);
        for (int i = 0; i < 3; i++) lidarMarker_camera[i] += pose_t[i];

        T uv[2];
        T xy[2];

        // Convert lidarMarker xyz into homogeneous coordinates
        xy[0] = lidarMarker_camera[0] / lidarMarker_camera[2];
        xy[1] = lidarMarker_camera[1] / lidarMarker_camera[2];

        bool success = m_pIntrinsics->Project(uv, xy);

        if(!success)
        {
            return false;
        }

        size_t width = m_pIntrinsics->Width();
        size_t height = m_pIntrinsics->Height();

        T pixel[2];
        pixel[0] = uv[0] * (T)width;
        pixel[1] = uv[1] * (T)height;

        Eigen::Matrix<T, 2, 1> m_observedMarker_t = m_observedMarker.cast<T>();

        residuals[0] = m_observedMarker_t.data()[0] - pixel[0];
        residuals[1] = m_observedMarker_t.data()[1] - pixel[1];

        return true;
    }

private:
    Eigen::Vector2d m_observedMarker;
    Eigen::Vector3d m_lidarMarker;
    std::shared_ptr<G2D::SE3Quat> m_pPose;
    Eigen::Vector4d m_pose_q;
    Eigen::Vector3d m_pose_t;
    const ProjModel* m_pIntrinsics;
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
void GetMockCameraIntrinsics(std::shared_ptr<ProjModel> pIntrinsics)
{
    MockData dataProvider;
    ProjModel* pModel = nullptr;
    dataProvider.CameraModel<ProjModel>(&pModel);
    pIntrinsics.reset(pModel);
}

int main(int /*argc*/, char** /*argv[]*/)
{
    vector<LidarObservation> observations;
    vector<LidarMarker> lidarMarkers;
    vector<FramePose> poses;
    shared_ptr<LinearCameraIntrinsics> pIntrinsics;
    SE3Quat gt_solution;

    // Get the sensor data
    GetMockData<LinearCameraIntrinsics>(observations, lidarMarkers, poses, gt_solution, pIntrinsics.get());

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
    Eigen::Vector4d solution_r;
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
                        pIntrinsics.get()));
        problem.AddResidualBlock(cost_function, NULL, solution_r.data(), solution_t.data());
        problem.SetParameterization(solution_r.data(), quat_plus);
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

