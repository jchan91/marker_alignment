/*
 * CameraIntrinsics.h
 *
 *  Created on: Mar 31, 2015
 *      Author: jonathan
 */

#ifndef CAMERAINTRINSICS_H_
#define CAMERAINTRINSICS_H_

#include <cassert>

namespace G2D
{
    const size_t CAMERA_INTRINSICS_MAX_PARAMS = 32;

    //--------------------------------------------------------------
    // CameraIntrinsics will assume everything is unitized
    class CameraIntrinsics
    {
    public:
        CameraIntrinsics() :
            m_width(0),
            m_height(0)
        {
        }

        virtual ~CameraIntrinsics()
        {
        }

        const double* GetParams() const
        {
            return m_params;
        }

        size_t Width() const { return m_width; }

        size_t Height() const { return m_height; }

        void SetParams(double* params, size_t numParams, size_t width, size_t height)
        {
            assert(numParams <= CAMERA_INTRINSICS_MAX_PARAMS);

            for(size_t i = 0; i < numParams; ++i)
            {
                m_params[i] = params[i];
            }

            m_width = width;
            m_height = height;

            update_internal();
        }

//        virtual bool Project(double* uv, const double* xy) const = 0;
//
//        virtual bool Unproject(double* xy, const double* uv) const = 0;

    protected:
        virtual void update_internal() = 0;

        double m_params[CAMERA_INTRINSICS_MAX_PARAMS];
        size_t m_width;
        size_t m_height;
    };

    class LinearCameraIntrinsics : public CameraIntrinsics
    {
    public:
        static const size_t PX = 0;
        static const size_t PY = 1;
        static const size_t FX = 2;
        static const size_t FY = 3;

        static const size_t NUM_PARAMS = 4;

        template <typename T>
        bool Project(T* uv, const T* xy) const
        {
            uv[0] = (xy[0] * m_params[FX]) + m_params[PX];
            uv[1] = (xy[1] * m_params[FY]) + m_params[PY];

            // TODO: Should this check really be here...
            T uv_pixel[2] = { uv[0] * (T)m_width, uv[1] * (T)m_height };
            if (uv_pixel[0] < 0.0 || uv_pixel[0] >= (T)m_width || uv_pixel[1] < 0.0 || uv_pixel[1] >= (T)m_height)
            {
                return false;
            }

            return true;
        }

        template <typename T>
        bool Unproject(T* xy, const T* uv) const
        {
            xy[0] = (uv[0] - m_params[PX]) * m_inv_f[0];
            xy[1] = (uv[1] - m_params[PY]) * m_inv_f[1];
            return true;
        }

    protected:
        virtual void update_internal()
        {
            m_inv_f[0] = 1.0 / m_params[FX];
            m_inv_f[1] = 1.0 / m_params[FY];
        }

    private:
        double m_inv_f[2];
    };
}


#endif /* CAMERAINTRINSICS_H_ */
