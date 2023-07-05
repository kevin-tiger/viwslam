/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "PinholeCamera.h"

const PinholeCamera::Parameters& PinholeCamera::getParameters(void) const
{
    return mParameters;
}

void PinholeCamera::setParameters(const PinholeCamera::Parameters& parameters)
{
    mParameters = parameters;
    // cout << "mParameters.k1() = " << mParameters.k1() << endl;
    // cout << "mParameters.k2() = " << mParameters.k2() << endl;
    // cout << "mParameters.p1() = " << mParameters.p1() << endl;
    // cout << "mParameters.p2() = " << mParameters.p2() << endl;
    // cout << "mParameters.fx() = " << mParameters.fx() << endl;
    // cout << "mParameters.fy() = " << mParameters.fy() << endl;
    // cout << "mParameters.cx() = " << mParameters.cx() << endl;
    // cout << "mParameters.cy() = " << mParameters.cy() << endl;
    if ((mParameters.k1() == 0.0) &&
        (mParameters.k2() == 0.0) &&
        (mParameters.p1() == 0.0) &&
        (mParameters.p2() == 0.0))
    {
        m_noDistortion = true;
    }
    else
    {
        m_noDistortion = false;
    }
    m_inv_K11 = 1.0 / mParameters.fx();
    m_inv_K13 = -mParameters.cx() / mParameters.fx();
    m_inv_K22 = 1.0 / mParameters.fy();
    m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

void PinholeCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P)
{
    double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    // Lift points to normalised plane
    mx_d = m_inv_K11 * p(0) + m_inv_K13;
    my_d = m_inv_K22 * p(1) + m_inv_K23;
    if (m_noDistortion)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    else
    {
            // Recursive distortion model
            int n = 8;
            Eigen::Vector2d d_u;
            distortion(Eigen::Vector2d(mx_d, my_d), d_u);
            // Approximate value
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);
            for (int i = 1; i < n; ++i)
            {
                distortion(Eigen::Vector2d(mx_u, my_u), d_u);
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1);
            }
    }        
    // Obtain a projective ray
    P << mx_u, my_u, 1.0;
}

void PinholeCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p)
{
    Eigen::Vector2d p_u, p_d;
    // Project points to the normalised plane
    p_u << P(0) / P(2), P(1) / P(2);
    if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = p_u + d_u;
    }
    // Apply generalised projection matrix
    p << mParameters.fx() * p_d(0) + mParameters.cx(),
         mParameters.fy() * p_d(1) + mParameters.cy();
}

void PinholeCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u)
{
    double k1 = mParameters.k1();
    double k2 = mParameters.k2();
    double p1 = mParameters.p1();
    double p2 = mParameters.p2();
    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;
    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

bool PinholeCamera::Parameters::readFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    m_modelType = PINHOLE;
    fs["camera_name"] >> m_cameraName;
    m_imageWidth = static_cast<int>(fs["image_width"]);
    m_imageHeight = static_cast<int>(fs["image_height"]);
    cv::FileNode n = fs["distortion_parameters"];
    m_k1 = static_cast<double>(n["k1"]);
    m_k2 = static_cast<double>(n["k2"]);
    m_p1 = static_cast<double>(n["p1"]);
    m_p2 = static_cast<double>(n["p2"]);
    n = fs["projection_parameters"];
    m_fx = static_cast<double>(n["fx"]);
    m_fy = static_cast<double>(n["fy"]);
    m_cx = static_cast<double>(n["cx"]);
    m_cy = static_cast<double>(n["cy"]);
    return true;
}

double PinholeCamera::Parameters::k1()
{
    return m_k1;
}

double PinholeCamera::Parameters::k2()
{
    return m_k2;
}

double PinholeCamera::Parameters::p1()
{
    return m_p1;
}

double PinholeCamera::Parameters::p2()
{
    return m_p2;
}

double PinholeCamera::Parameters::fx()
{
    return m_fx;
}

double PinholeCamera::Parameters::fy()
{
    return m_fy;
}

double PinholeCamera::Parameters::cx()
{
    return m_cx;
}

double PinholeCamera::Parameters::cy()
{
    return m_cy;
}
