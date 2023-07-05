/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
#include "utility/basetype.h"
#include <boost/shared_ptr.hpp>

class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum ModelType
    {
        KANNALA_BRANDT,
        MEI,
        PINHOLE
    };
    class Parameters
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        ModelType m_modelType;
        int m_nIntrinsics;
        std::string m_cameraName;
        int m_imageWidth;
        int m_imageHeight;
    };
    virtual void liftProjective( const Eigen::Vector2d& p, Eigen::Vector3d& P ) = 0;
    virtual void spaceToPlane( const Eigen::Vector3d& P, Eigen::Vector2d& p ) = 0;
};
typedef boost::shared_ptr<Camera> CameraPtr;