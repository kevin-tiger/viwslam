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
#include "utility/utility.h"
#include "estimator/parameters.h"
#include "integration_base.h"
#include <ceres/ceres.h>

class IMUEncoderFactor : public ceres::SizedCostFunction<18, 7, 9, 7, 9>
{
public:
    IMUEncoderFactor(IntegrationBase* _pre_integration);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
private:
    IntegrationBase* pre_integration;
};

