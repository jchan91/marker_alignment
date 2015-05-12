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

#ifndef DEBUG
#define DEBUG 1
#endif

namespace G2D
{
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
            std::shared_ptr<G2D::SE3Quat> pRig2World,
            const ProjModel* pIntrinsics) :
                m_observedMarker(observedMarker),
                m_lidarMarker(lidarMarker),
                m_pRig2World(pRig2World),
                m_pIntrinsics(pIntrinsics)
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
            Eigen::Matrix<T,4,1> l2w_q;
            l2w_q[0] = lidar2world_q[0];
            l2w_q[1] = lidar2world_q[1];
            l2w_q[2] = lidar2world_q[2];
            l2w_q[3] = lidar2world_q[3];

            Eigen::Matrix<T,3,1> l2w_t;
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

            Eigen::Matrix<T, 2, 1> m_observedMarker_t = m_observedMarker.cast<T>();

            residuals[0] = m_observedMarker_t.data()[0] - pixel[0];
            residuals[1] = m_observedMarker_t.data()[1] - pixel[1];

    #ifdef DEBUG
            Eigen::Matrix<T, 2, 1> residuals_debug(residuals);
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
    };
}


#endif
