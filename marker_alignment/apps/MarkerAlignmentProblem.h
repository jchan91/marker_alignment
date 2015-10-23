/*
 * main.cpp
 *
 *  Created on: May 12, 2015
 *      Author: jonathan
 */

#ifndef MARKERALIGNMENTPROBLEM_H_
#define MARKERALIGNMENTPROBLEM_H_

#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glog/logging.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "se3quat.h"
#include "CameraIntrinsics.h"
#include "MockData.h"

#include "Viewer3D.h"

#ifndef DEBUG
#define DEBUG 1
#endif

namespace G2D
{

    // Class for Ceres to call after each iteration. Useful for things like viz
    class ItrCallback : public ceres::IterationCallback
    {
    public:
        ItrCallback(
                G2D::Viewer3D* pViewer,
                const Eigen::Vector4d & solution_r,
                const Eigen::Vector3d & solution_t) :
            m_pViewer(pViewer),
            m_solution_r(solution_r),
            m_solution_t(solution_t)
        {
            if (nullptr == pViewer)
            {
                // Warn
                std::cerr << "[WARNING] ItrCallback: Viewer shouldn't be null!" << std::endl;
            }
        }

        virtual ceres::CallbackReturnType operator()(
                const ceres::IterationSummary& summary)
        {
            std::cerr << "Iteration callback" << std::endl;

            Eigen::Quaterniond q(m_solution_r);

            printf("Q: x=%f y=%f z=%f w=%f\n", q.x(), q.y(), q.z(), q.w());
            printf("t: %f %f %f\n", m_solution_t[0], m_solution_t[1], m_solution_t[2]);

            m_pViewer->AddFrustum(q, m_solution_t);
            m_pViewer->MaybeYieldToViewer();

            return ceres::SOLVER_CONTINUE;
        }

    private:
        G2D::Viewer3D* m_pViewer;
        const Eigen::Vector4d & m_solution_r;
        const Eigen::Vector3d & m_solution_t;
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
        // Use Eigen constructor to generate 16-byte aligned ptrs
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LidarReprojectionError(
            Eigen::Vector2d observedMarker,
            Eigen::Vector3d lidarMarker,
            std::shared_ptr<G2D::SE3Quat> pRig2World,
            const ProjModel* pIntrinsics,
            G2D::Viewer3D* pViewer = nullptr) :
                m_observedMarker(observedMarker),
                m_lidarMarker(lidarMarker),
                m_pRig2World(pRig2World),
                m_pIntrinsics(pIntrinsics),
                m_pViewer(pViewer)
        {
            G2D::SE3Quat w2r = m_pRig2World->inverse();
            Eigen::Quaterniond pose_q = w2r.rotation();
            m_world2rig_q[0] = pose_q.w();
            m_world2rig_q[1] = pose_q.x();
            m_world2rig_q[2] = pose_q.y();
            m_world2rig_q[3] = pose_q.z();
            m_world2rig_t = w2r.translation();
        }

        template <typename T>
        bool operator() (const T* lidar2world_q, const T* lidar2world_t, T* residuals) const
        {
    #ifdef DEBUG
            T l2w_q[4];
            l2w_q[0] = lidar2world_q[0];
            l2w_q[1] = lidar2world_q[1];
            l2w_q[2] = lidar2world_q[2];
            l2w_q[3] = lidar2world_q[3];

            T l2w_t[3];
            l2w_t[0] = lidar2world_t[0];
            l2w_t[1] = lidar2world_t[1];
            l2w_t[2] = lidar2world_t[2];
    #endif
            // Transform into the world space
            T lidarMarker_world[3];
            Eigen::Matrix<T,3,1> lidarMarker = m_lidarMarker.cast<T>();
            ceres::QuaternionRotatePoint(lidar2world_q, lidarMarker.data(), lidarMarker_world);
            for (int i = 0; i < 3; i++) lidarMarker_world[i] += lidar2world_t[i];

            // Transform into the camera
            T lidarMarker_camera[3];
            Eigen::Matrix<T,4,1> w2r_q = m_world2rig_q.cast<T>();
            Eigen::Matrix<T,3,1> w2r_t = m_world2rig_t.cast<T>();
            ceres::QuaternionRotatePoint(w2r_q.data(), lidarMarker_world, lidarMarker_camera);
            for (int i = 0; i < 3; i++) lidarMarker_camera[i] += w2r_t[i];

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

            Eigen::Matrix<T, 2, 1> observedMarker_t = m_observedMarker.cast<T>();

            residuals[0] = observedMarker_t.data()[0] - pixel[0];
            residuals[1] = observedMarker_t.data()[1] - pixel[1];

    #ifdef DEBUG
            T l_pos_w[3];
            l_pos_w[0] = lidarMarker[0];
            l_pos_w[1] = lidarMarker[1];
            l_pos_w[2] = lidarMarker[2];

            T residuals_debug[2];
            residuals_debug[0] = residuals[0];
            residuals_debug[1] = residuals[1];
    #endif

            return true;
        }

    private:
        Eigen::Vector2d m_observedMarker;
        Eigen::Vector3d m_lidarMarker;
        std::shared_ptr<G2D::SE3Quat> m_pRig2World;
        Eigen::Vector4d m_world2rig_q;
        Eigen::Vector3d m_world2rig_t;
        const ProjModel* m_pIntrinsics;

        G2D::Viewer3D* m_pViewer;
    };
}


#endif
