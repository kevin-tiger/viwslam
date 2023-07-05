/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

const double for_average_parallax = 1280;
double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;
double ENC_N; 
std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
std::vector<Eigen::Matrix3d> RIV;
std::vector<Eigen::Vector3d> TIV;
Eigen::Matrix3d RIO; 
Eigen::Vector3d TIO; 
Eigen::Vector3d G{0.0, 0.0, 9.8};
double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
double MAX_ANGLE_VEL;
double MAX_CNT_1;
double MAX_ANGVEL_BIAS;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
map<int, Eigen::Vector3d> pts_gt;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

void readParameters(std::string config_file)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];
    USE_IMU = fsSettings["imu"];
    if(USE_IMU)
    {
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        ENC_N = fsSettings["enc_n"]; 
        G.z() = fsSettings["g_norm"];
    }
    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
    MAX_ANGLE_VEL = fsSettings["max_ang_vel"];
    MAX_CNT_1 = fsSettings["max_cnt_1"];
    MAX_ANGVEL_BIAS = fsSettings["max_angvel_bias"];
    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 0) 
    {
        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        // cout << "imu_T_cam0 = \n" << T << endl; 
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
        cv::Mat cv_T_vel;
        Eigen::Matrix4d T_vel;
        fsSettings["body_T_vel"] >> cv_T_vel;
        cv::cv2eigen(cv_T_vel, T_vel);
        // cout << "imu_T_vel = \n" << T_vel << endl;
        RIV.push_back(T_vel.block<3, 3>(0, 0));
        TIV.push_back(T_vel.block<3, 1>(0, 3));
    } 
    NUM_OF_CAM = fsSettings["num_of_cam"];
    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    // cout << "cam0Path = " << cam0Path << endl;
    CAM_NAMES.push_back(cam0Path);
    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;
    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    fsSettings.release();
}
