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
#include "Camera.h"

class PinholeCamera: public Camera
{
public:
    class Parameters: public Camera::Parameters
    {
    public:
        bool readFromYamlFile(const std::string& filename);
        double k1();
        double k2();
        double p1();
        double p2();
        double fx();
        double fy();
        double cx();
        double cy();
    private:
        double m_k1;
        double m_k2;
        double m_p1;
        double m_p2;
        double m_fx;
        double m_fy;
        double m_cx;
        double m_cy;
    };
public:
    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P);
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p);
private:
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u);
private:
    Parameters mParameters;
    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};
typedef boost::shared_ptr<PinholeCamera> PinholeCameraPtr;