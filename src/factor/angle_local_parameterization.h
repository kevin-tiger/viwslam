#pragma once
#include <ceres/ceres.h>
#include "utility/utility_2.h"

class AngleLocalParameterization 
{
public:
    template <typename T>
    bool operator()(const T* theta_radians, const T* delta_theta_radians, T* theta_radians_plus_delta) const 
    {
        *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
        return true;
    }

    static ceres::LocalParameterization* Create() 
    {
        return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,1, 1>);
    }
};