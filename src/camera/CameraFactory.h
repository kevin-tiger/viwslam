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
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include "Camera.h"
#include "PinholeCamera.h"

class CameraFactory
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CameraFactory();
    static boost::shared_ptr<CameraFactory> instance(void);
    CameraPtr generateCameraFromYamlFile(const std::string& filename);
private:
    static boost::shared_ptr<CameraFactory> m_instance;
};


