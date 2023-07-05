/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "factor/integration_base.h"

IntegrationBase::IntegrationBase(
    const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0, const Eigen::Vector3d &_vel_0,
    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg
)
:
acc_0{_acc_0}, gyr_0{_gyr_0}, vel_0{_vel_0}, 
linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
jacobian_enc{Eigen::Matrix<double, 18, 18>::Identity()}, 
covariance_enc{Eigen::Matrix<double, 18, 18>::Zero()},
sum_dt{0.0}, 
delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()},
delta_p_i_vel{Eigen::Vector3d::Zero()},delta_angleaxis{0, Eigen::Vector3d ( 0,0,1 ) }
{
    noise_enc = Eigen::Matrix<double, 24, 24>::Zero();
    noise_enc.block<3, 3>(0, 0) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise_enc.block<3, 3>(3, 3) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise_enc.block<3, 3>(6, 6) = (ENC_N * ENC_N) * Eigen::Matrix3d::Identity();
    noise_enc.block<3, 3>(9, 9) = (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
    noise_enc.block<3, 3>(12, 12) = (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
    noise_enc.block<3, 3>(15, 15) = (ENC_N * ENC_N) * Eigen::Matrix3d::Identity();
    noise_enc.block<3, 3>(18, 18) = (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
    noise_enc.block<3, 3>(21, 21) = (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
}

void IntegrationBase::push_back_wheels(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr, const Eigen::Vector3d &vel)
{
    dt_buf.push_back(dt); // this buf data for repropagate if needed later
    acc_buf.push_back(acc);
    gyr_buf.push_back(gyr);
    vel_buf.push_back(vel);
    propagate_wheel(dt, acc, gyr,vel);
}

void IntegrationBase::repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
{
    sum_dt = 0.0;
    acc_0 = linearized_acc;
    gyr_0 = linearized_gyr;
    delta_p.setZero();
    delta_q.setIdentity();
    delta_v.setZero();
    delta_p_i_vel.setZero();
    delta_angleaxis.fromRotationMatrix(delta_q.toRotationMatrix());
    linearized_ba = _linearized_ba;
    linearized_bg = _linearized_bg;
    jacobian_enc.setIdentity();
    covariance_enc.setZero();
    for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
        propagate_wheel(dt_buf[i], acc_buf[i], gyr_buf[i],vel_buf[i]);
}

void IntegrationBase::midPointIntegration_encode(
    double _dt,
    const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
    const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
    const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
    const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
    Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
    Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian,
    const Eigen::Vector3d &_vel_0, const Eigen::Vector3d &_vel_1,
    const Eigen::Vector3d &delta_p_vel, Eigen::Vector3d &result_delta_p_vel
)
{
    Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba); // last acc in world
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg; // last gyro
    Vector3d un_vel_0 = delta_q * RIV[0] * (_vel_0); // last wheel speed in world
    result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2); // get rotation by gyro

    Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba); // current acc in world
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1); // get mid acc that is the average of last acc and current acc
    Vector3d un_vel_1 = result_delta_q * RIV[0]* (_vel_1); //current wheel speed in world
    Vector3d un_vel = 0.5 * (un_vel_0 + un_vel_1); // get mid wheel speed that is the average of last wheel speed and current wheel speed

    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt; // get position by acc
    result_delta_v = delta_v + un_acc * _dt; // get velocity by acc
    result_delta_p_vel = delta_p_vel + un_vel * _dt;  // get position by wheel speed
    result_linearized_ba = linearized_ba;  // acc bias keep the same
    result_linearized_bg = linearized_bg; // gyro bias keep the same

    if(update_jacobian)
    {
        Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        Vector3d a_0_x = _acc_0 - linearized_ba;
        Vector3d a_1_x = _acc_1 - linearized_ba;
        Vector3d e_0_x = RIV[0] * _vel_0;
        Vector3d e_1_x = RIV[0] * _vel_0;
        Matrix3d R_w_x, R_a_0_x, R_a_1_x, R_e_0_x, R_e_1_x;

        R_w_x<<0, -w_x(2), w_x(1),
                w_x(2), 0, -w_x(0),
                -w_x(1), w_x(0), 0;
        R_a_0_x<<0, -a_0_x(2), a_0_x(1),
                a_0_x(2), 0, -a_0_x(0),
                -a_0_x(1), a_0_x(0), 0;
        R_a_1_x<<0, -a_1_x(2), a_1_x(1),
                a_1_x(2), 0, -a_1_x(0),
                -a_1_x(1), a_1_x(0), 0;
        R_e_0_x << 0, -e_0_x(2), e_0_x(1),
                e_0_x(2), 0, -e_0_x(0),
                -e_0_x(1), e_0_x(0), 0;
        R_e_1_x << 0, -e_1_x(2), e_1_x(1),
                e_1_x(2), 0, -e_1_x(0),
                -e_1_x(1), e_1_x(0), 0;

        MatrixXd F = MatrixXd::Zero(18, 18);
        F.block<3, 3>(0, 0) = Matrix3d::Identity();
        F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                                -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
        F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
        F.block<3, 3>(0, 12) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
        F.block<3, 3>(0, 15) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
        F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
        F.block<3, 3>(3, 15) = -1.0 * MatrixXd::Identity(3,3) * _dt;
        F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                                -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
        F.block<3, 3>(6, 6) = Matrix3d::Identity();
        F.block<3, 3>(6, 12) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
        F.block<3, 3>(6, 15) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
        F.block<3, 3>(9, 3) = -0.5 * delta_q.toRotationMatrix() * R_e_0_x * _dt +
                                -0.5 * result_delta_q.toRotationMatrix() * R_e_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt; 
        F.block<3, 3>(9, 9) = Matrix3d::Identity();
        F.block<3, 3>(9, 15) = 0.5 * result_delta_q.toRotationMatrix() * R_e_1_x * _dt * _dt;

        F.block<3, 3>(12, 12) = Matrix3d::Identity();
        F.block<3, 3>(15, 15) = Matrix3d::Identity();

        MatrixXd V = MatrixXd::Zero(18, 24);
        V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
        V.block<3, 3>(0, 9) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
        V.block<3, 3>(0, 12) =  V.block<3, 3>(0, 3);

        V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
        V.block<3, 3>(3, 12) =  0.5 * MatrixXd::Identity(3,3) * _dt;

        V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
        V.block<3, 3>(6, 9) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
        V.block<3, 3>(6, 12) =  V.block<3, 3>(6, 3);

        V.block<3, 3>(9, 3) = -0.25 * result_delta_q.toRotationMatrix() * R_e_1_x * _dt * _dt; 
        V.block<3, 3>(9, 6) = 0.5 * delta_q.toRotationMatrix() * RIV[0] * _dt;
        V.block<3, 3>(9, 12) = V.block<3, 3>(9, 3);
        V.block<3, 3>(9, 15) = 0.5 * result_delta_q.toRotationMatrix() * RIV[0] * _dt;

        V.block<3, 3>(12, 18) = MatrixXd::Identity(3,3) * _dt;
        V.block<3, 3>(15, 21) = MatrixXd::Identity(3,3) * _dt;

        jacobian_enc = F * jacobian_enc;
        covariance_enc = F * covariance_enc * F.transpose() + V * noise_enc * V.transpose();
    }
}

void IntegrationBase::propagate_wheel(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1 ,const Eigen::Vector3d &_vel_1)
{
    dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    vel_1 = _vel_1;
    Vector3d result_delta_p;
    Vector3d result_delta_p_vel;
    Quaterniond result_delta_q;
    Vector3d result_delta_v;
    Vector3d result_linearized_ba;
    Vector3d result_linearized_bg;

    midPointIntegration_encode(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                                    linearized_ba, linearized_bg,
                                    result_delta_p, result_delta_q, result_delta_v,
                                    result_linearized_ba, result_linearized_bg, 1,
                                    vel_0, vel_1, delta_p_i_vel, result_delta_p_vel);

    delta_p = result_delta_p;
    delta_p_i_vel = result_delta_p_vel;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    delta_q.normalize();
    delta_angleaxis.fromRotationMatrix(delta_q.toRotationMatrix()); //轴角
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;
    vel_0 = vel_1;
}

Eigen::Matrix<double, 18, 1> IntegrationBase::evaluate_encode(
    const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, 
    const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
    const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj,
    const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj
)
{
    Eigen::Matrix<double, 18, 1> residuals;
    Eigen::Matrix3d dp_dba = jacobian_enc.block<3, 3>(0, 12);
    Eigen::Matrix3d dp_dbg = jacobian_enc.block<3, 3>(0, 15);
    Eigen::Matrix3d dq_dbg = jacobian_enc.block<3, 3>(3, 15);
    Eigen::Matrix3d dv_dba = jacobian_enc.block<3, 3>(6, 12);
    Eigen::Matrix3d dv_dbg = jacobian_enc.block<3, 3>(6, 15);
    Eigen::Matrix3d do_dbg = jacobian_enc.block<3, 3>(9, 15);
    Eigen::Vector3d dba = Bai - linearized_ba;
    Eigen::Vector3d dbg = Bgi - linearized_bg;
    Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
    Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
    Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;
    Eigen::Vector3d corrected_delta_eta = delta_p_i_vel + do_dbg * dbg;
    residuals.block<3, 1>(0, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    residuals.block<3, 1>(3, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
    residuals.block<3, 1>(6, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(9, 0) = (Qi.inverse() * ((Pj + Qj * TIV[0]) - (Pi + Qi * TIV[0])) - corrected_delta_eta); 
    residuals.block<3, 1>(12, 0) = Baj - Bai;
    residuals.block<3, 1>(15, 0) = Bgj - Bgi;
    return residuals;
}
