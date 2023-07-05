/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "factor/imu_encoder_factor.h"

IMUEncoderFactor::IMUEncoderFactor(IntegrationBase* _pre_integration)
:
pre_integration(_pre_integration)
{
}

bool IMUEncoderFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]); // last position
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]); // last rotation
    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]); // last velocity
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]); // last acc bias
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]); // last gyro bias
    Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]); // current position
    Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]); // current rotation
    Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]); // current velocity
    Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]); // current acc bias
    Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]); // current gyro bias
    Eigen::Map<Eigen::Matrix<double, 18, 1>> residual(residuals);
    residual = pre_integration->evaluate_encode(Pi, Qi, Vi, Bai, Bgi,
                                        Pj, Qj, Vj, Baj, Bgj);
    Eigen::Matrix<double, 18, 18> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 18, 18>>(pre_integration->covariance_enc.inverse()).matrixL().transpose();
    residual = sqrt_info * residual;
    if (jacobians)
    {
        double sum_dt = pre_integration->sum_dt;
        Eigen::Matrix3d dp_dba = pre_integration->jacobian_enc.template block<3, 3>(0, 12);
        Eigen::Matrix3d dp_dbg = pre_integration->jacobian_enc.template block<3, 3>(0, 15);
        Eigen::Matrix3d dq_dbg = pre_integration->jacobian_enc.template block<3, 3>(3, 15);
        Eigen::Matrix3d dv_dba = pre_integration->jacobian_enc.template block<3, 3>(6, 12);
        Eigen::Matrix3d dv_dbg = pre_integration->jacobian_enc.template block<3, 3>(6, 15);
        Eigen::Matrix3d do_dbg = pre_integration->jacobian_enc.template block<3, 3>(9, 15);
        MatrixXd::Index maxRow, maxCol;
        MatrixXd::Index minRow, minCol;
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.setZero();
            jacobian_pose_i.block<3, 3>(0, 0) = -Qi.inverse().toRotationMatrix();
            jacobian_pose_i.block<3, 3>(0, 3) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));
            Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
            jacobian_pose_i.block<3, 3>(3, 3) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
            jacobian_pose_i.block<3, 3>(6, 3) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));
            jacobian_pose_i.block<3, 3>(9, 0) = -Qi.inverse().toRotationMatrix();
            jacobian_pose_i.block<3, 3>(9, 3) = Utility::skewSymmetric(Qi.inverse() * (Pj + Qj * TIV[0] - Pi));
            jacobian_pose_i = sqrt_info * jacobian_pose_i;
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
            jacobian_speedbias_i.setZero();
            jacobian_speedbias_i.block<3, 3>(0, 0) = -Qi.inverse().toRotationMatrix() * sum_dt;
            jacobian_speedbias_i.block<3, 3>(0, 3) = -dp_dba;
            jacobian_speedbias_i.block<3, 3>(0, 6) = -dp_dbg;
            jacobian_speedbias_i.block<3, 3>(3, 6) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
            jacobian_speedbias_i.block<3, 3>(6, 0) = -Qi.inverse().toRotationMatrix();
            jacobian_speedbias_i.block<3, 3>(6, 3) = -dv_dba;
            jacobian_speedbias_i.block<3, 3>(6, 6) = -dv_dbg;
            jacobian_speedbias_i.block<3, 3>(9, 6) = -do_dbg;
            jacobian_speedbias_i.block<3, 3>(12,3) = -Eigen::Matrix3d::Identity();
            jacobian_speedbias_i.block<3, 3>(15, 6) = -Eigen::Matrix3d::Identity();
            jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
            jacobian_pose_j.setZero();
            jacobian_pose_j.block<3, 3>(0, 0) = Qi.inverse().toRotationMatrix();
            Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
            jacobian_pose_j.block<3, 3>(3, 3) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
            jacobian_pose_j.block<3, 3>(9, 0) = Qi.inverse().toRotationMatrix();
            jacobian_pose_j.block<3, 3>(9, 3) = -Qi.inverse().toRotationMatrix() * Qj * Utility::skewSymmetric(TIV[0]);
            jacobian_pose_j = sqrt_info * jacobian_pose_j;
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
            jacobian_speedbias_j.setZero();
            jacobian_speedbias_j.block<3, 3>(6, 0) = Qi.inverse().toRotationMatrix();
            jacobian_speedbias_j.block<3, 3>(12, 3) = Eigen::Matrix3d::Identity();
            jacobian_speedbias_j.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
            jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
        }
    }
    return true;
}


