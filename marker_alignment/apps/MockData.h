/*
 * MockData.h
 *
 *  Created on: Mar 27, 2015
 *      Author: jonathan
 */

#ifndef MOCKDATA_H_
#define MOCKDATA_H_

#include "se3quat.h"
#include "math_constants.h"
#include "CameraIntrinsics.h"

#include <Eigen/Core>

#include <vector>
#include <memory>

namespace G2D
{
    struct LidarMarker
    {
        int id;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d pos;
    };

    struct LidarObservation
    {
        int marker_id;
        int frame_id;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector2d uv;
    };

    struct FramePose
    {
        int frame_id;
        std::shared_ptr<G2D::SE3Quat> pose;
    };

    class MockData
    {
    public:
        void LidarMarkers(std::vector<LidarMarker> &points)
        {
            points.push_back({ 0, Eigen::Vector3d(0.0, 0.0, 1.0) });
            points.push_back({ 1, Eigen::Vector3d(0.0, 0.8, 1.0) });
            points.push_back({ 2, Eigen::Vector3d(-2.0, 0.0, 1.0) });
            points.push_back({ 3, Eigen::Vector3d(-1.0, 0.0, -3.0) });
            points.push_back({ 4, Eigen::Vector3d(1.0, 0.0, -1.0) });
        }

        // Produces poses which are transforms from the Rig to the World
        void Trajectory_Circle(std::vector<FramePose> &poses, const int numPoses, const uint32_t radius)
        {
            // "Walk" in a circle around the markers, at radius
            double angleRotated = 2 * PI / static_cast<double>(numPoses);
            auto angleRotated_a = Eigen::AngleAxisd(angleRotated, Eigen::Vector3d::UnitY());
            angleRotated_a = angleRotated_a.inverse(); // Rotate counter-clockwise instead of clockwise
            Eigen::Quaterniond rotY_q = Eigen::Quaterniond::Identity();
            Eigen::Vector3d translation(0, 0, -1.0 * static_cast<double>(radius));

            for (int frame = 0; frame < numPoses; frame++)
            {
                std::shared_ptr<G2D::SE3Quat> pPose(new G2D::SE3Quat);
                pPose->setRotation(rotY_q);
                pPose->setTranslation(translation);
                poses.push_back({ frame, pPose });

                // Perturb the rotation/translation
                translation[0] = static_cast<double>(radius) * std::sin(static_cast<double>(frame + 1) * angleRotated);
                translation[2] = -1.0 * static_cast<double>(radius) * std::cos(static_cast<double>(frame + 1) * angleRotated);
                //auto rotY_r = Eigen::AngleAxisd(rotY_q);
                //rotY_q = rotY_r * angleRotated_a;
                rotY_q =  Eigen::Quaterniond(angleRotated_a.matrix() * rotY_q.matrix());
                if(rotY_q.w() < 0)
                {
                    rotY_q.coeffs() *= -1;
                }
                rotY_q.normalize();
            }
        }

        template <typename ProjModel>
        void Observations(
                std::vector<LidarObservation> &obs,
                const std::vector<LidarMarker> &markers,
                const std::vector<FramePose> &poses,
                const ProjModel* pIntrinsics,
                void* /*pNoiseModel*/)
        {
            size_t width = pIntrinsics->Width();
            size_t height = pIntrinsics->Height();

            // For every pose,
            //   Check which markers would be seen by these markers
            for (size_t p = 0; p < poses.size(); p++)
            {
                const G2D::SE3Quat &r2w = *poses[p].pose;
                const G2D::SE3Quat w2r = r2w.inverse();

                for (size_t m = 0; m < markers.size(); m++)
                {
                    // Transform the marker into the current pose frame of the rig
                    Eigen::Vector3d obs_xyz(markers[m].pos);
                    obs_xyz = w2r * obs_xyz;

                    // Project the transformed point into the camera image space
                    Eigen::Vector2d obs_uv_h;
                    // Convert the marker to homogeneous coords
                    obs_xyz[0] /= obs_xyz[2];
                    obs_xyz[1] /= obs_xyz[2];

#ifdef DEBUG
                    double obs_xyz_d[3];
                    ExtractVec(obs_xyz_d, obs_xyz);
#endif

                    if (pIntrinsics->Project(obs_uv_h.data(), obs_xyz.data()))
                    {
                        Eigen::Vector2d obs_uv_p( { obs_uv_h[0] * width, obs_uv_h[1] * height });

#ifdef DEBUG
                        double obs_uv_p_d[2];
                        obs_uv_p_d[0] = obs_uv_p[0];
                        obs_uv_p_d[1] = obs_uv_p[1];
#endif
                        // TODO: camera noise modeling here
                        obs.push_back({ markers[m].id, poses[p].frame_id, obs_uv_p });
                    }
                    else
                    {
                        // This marker point failed to project into the camera. Must not have been seen at this pose
                    }
                }
            }
        }

        template <typename ProjModel>
        void CameraModel(ProjModel** ppIntrinsics)
        {
            if (nullptr == ppIntrinsics)
            {
                return;
            }

            *ppIntrinsics = new LinearCameraIntrinsics();
            double params[LinearCameraIntrinsics::NUM_PARAMS];
            params[LinearCameraIntrinsics::FX] = 1.5;
            params[LinearCameraIntrinsics::FY] = 1.5;
            params[LinearCameraIntrinsics::PX] = 0.5;
            params[LinearCameraIntrinsics::PY] = 0.5;
            (*ppIntrinsics)->SetParams(params, LinearCameraIntrinsics::NUM_PARAMS, 600, 600);
        }

        void Solution(SE3Quat &lidar2rig)
        {
            double angle = PI / 180;
            Eigen::Vector3d translation(0.0, 0.0, 0.0);
            Eigen::Quaterniond q(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));

            lidar2rig.setRotation(q);
            lidar2rig.setTranslation(translation);
        }
    };

}

#endif /* MOCKDATA_H_ */
