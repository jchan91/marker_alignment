/*
 * CameraState.h
 *
 *  Created on: Mar 31, 2015
 *      Author: jonathan
 */

#ifndef CAMERASTATE_H_
#define CAMERASTATE_H_

#include <vector>

#include "Pose.h"
#include "CameraIntrinsics.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace G2D
{
    struct CameraState
    {
    public:
        Pose pose;

        const CameraIntrinsics* pIntrinsics;

        std::vector<Eigen::Vector2f> MarkerObservations;

    };
}




#endif /* CAMERASTATE_H_ */
