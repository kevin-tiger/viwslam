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
#include <ceres/ceres.h>

class IntegrationBase
{
public:
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_vel_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg);
    void push_back_wheels(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const Eigen::Vector3d &vel);
    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg);
    void midPointIntegration_encode(double _dt,
                                   const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                                   const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                                   const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                                   const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                                   Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                                   Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian,
                                   const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_vel_1,
                                   const Eigen::Vector3d &delta_p_vel, Eigen::Vector3d &result_delta_p_vel);
    void propagate_wheel(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1 ,const Eigen::Vector3d &_vel_1);
    Eigen::Matrix<double, 18, 1> evaluate_encode(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, 
                                          const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
                                          const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj);
public:
    double sum_dt;
    Eigen::Vector3d delta_p;
    Eigen::Vector3d delta_p_i_vel;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;
    Eigen::AngleAxis<double> delta_angleaxis;
    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;
    Eigen::Matrix<double, 18, 18> jacobian_enc, covariance_enc;
    Eigen::Matrix<double, 24, 24> noise_enc;
private:
    double dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;
    Eigen::Vector3d vel_0, vel_1;
    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;
    std::vector<Eigen::Vector3d> vel_buf;
};