/*
 * SE3.h
 *
 *  Created on: Mar 27, 2015
 *      Author: jonathan
 */

#ifndef POSE_H_
#define POSE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ST
{
    template <typename T>
    class SE3 : public Eigen::Transform<T, 3, Eigen::Isometry>
    {


    };

}

namespace G2D
{
    class Pose
    {
    public:
        Eigen::Vector4f q;
        Eigen::Vector3f t;

        ST::SE3<float> se3;

        ST::SE3<float> MakeSE3()
        {
            // TODO: Make this generate the SE3 from quaternion/translation
            return se3;
        }
    };
}

#endif /* POSE_H_ */
