/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "projectionTwoFrameOneCamFactor.h"

Eigen::Matrix2d ProjectionTwoFrameOneCamFactor::sqrt_info;
ProjectionTwoFrameOneCamFactor::ProjectionTwoFrameOneCamFactor(
    const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, 
    const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
    const double _td_i, const double _td_j
) 
: 
pts_i(_pts_i), pts_j(_pts_j), 
td_i(_td_i), td_j(_td_j)
{
    velocity_i.x() = _velocity_i.x();
    velocity_i.y() = _velocity_i.y();
    velocity_i.z() = 0;
    velocity_j.x() = _velocity_j.x();
    velocity_j.y() = _velocity_j.y();
    velocity_j.z() = 0;
};

bool ProjectionTwoFrameOneCamFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]); // position on frame i
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]); // rotation on frame i
    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]); // position on frame j
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]); // rotation on frame j
    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]); // tic
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]); // ric
    double inv_dep_i = parameters[3][0]; // inverse depth for feature
    double td = parameters[4][0]; // delta time between imu and camera
    Eigen::Vector3d pts_i_td, pts_j_td;
    pts_i_td = pts_i - (td - td_i) * velocity_i; // remove the affect of delta time
    pts_j_td = pts_j - (td - td_j) * velocity_j;
    Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i; // get the coordinate in camera i
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic; // get the coordinate in imu i
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi; // get the coordinate in world
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj); // get the coordinate in imu j
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic); // get the coordinate in camera j
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>(); // get residual in frame j
    residual = sqrt_info * residual; // multiple the information matrix
    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3); // for dr/dpj
        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
            0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
        reduce = sqrt_info * reduce; // multiple the information matrix
        if (jacobians[0]) // for frame i pose
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose(); // dpj/dt_i
            jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i); // dpj/dr_i
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i; // chain deduce
            jacobian_pose_i.rightCols<1>().setZero();
        }
        if (jacobians[1]) // for frame j pose
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose(); // dpj/dt_j
            jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j); // dpj/dr_j
            jacobian_pose_j.leftCols<6>() = reduce * jaco_j; // chain deduce
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[2]) // for tic ric
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
            Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) + Utility::skewSymmetric(tmp_r * pts_camera_i) +
                                     Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3]) // for feature
        {
            Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i_td * -1.0 / (inv_dep_i * inv_dep_i);
        }
        if (jacobians[4]) // for td
        {
            Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[4]);
            jacobian_td = reduce * ric.transpose() * Rj.transpose() * Ri * ric * velocity_i / inv_dep_i * -1.0  + sqrt_info * velocity_j.head(2);
        }
    }
    return true;
}
