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
#include <ceres/ceres.h>
#include "estimator/parameters.h"
#include "estimator/feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include "factor/imu_encoder_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/marginalization_factor.h"
#include "factor/projectionTwoFrameOneCamFactor.h"
#include "featureTracker/feature_tracker.h"
#include "viewer/viewer.h"
#include "loop_fusion/keyframe.h"
#include "loop_fusion/pose_graph.h"

class Estimator
{
public:
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
public:
    Estimator();
    void setParameter();
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void inputVEL(double t, const Eigen::Vector3d &velVec);
    void inputImage(double t, const cv::Mat &_img);
    void calcFeatureTrackTime();
private:
    void processIMU_with_wheel(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity,const Eigen::Vector3d vel);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header);
    void vio_process_loop();
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();
    bool associateVelForImuData( double t, vector<pair<double, Eigen::Vector3d>> &VelVector);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void getVelInWorldFrame(Eigen::Vector3d &v);

    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates();
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    Eigen::Vector3d toEuler(const Eigen::Matrix3d &R);
    void command();
    void process();
    void pubKeyframe();
    void process_loop_close(KeyframeData keyframe_data);
    void posegraph_loop();
    void data_process_loop();
public:
    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double Headers[(WINDOW_SIZE + 1)];
    FeatureManager f_manager;
    Matrix3d ric[2];
    Vector3d tic[2];
    Matrix3d riv;
    Vector3d tiv;
    double td;
    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;
    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;
private:
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
    queue<pair<double, Eigen::Vector3d>> velBuf;
    queue<pair<double, cv::Mat>> imgBuf;
    pair<double, Eigen::Vector3d> temp_vel;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
    double prevTime, curTime;
    bool openExEstimation;
    FeatureTracker featureTracker;
    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0,vel_0;
    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> vel_velocity_buf[(WINDOW_SIZE + 1)];
    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;
    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];
    int loop_window_index;
    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;
    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;
    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;
    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;
    bool initFirstPoseFlag;
    bool initThreadFlag;
    bool initResult=false;
    double BiasCnt = 0;
    int frame_num = 0;
    int keyframe_num = 0;
    vector<double> mvec_feature_track_time;
    std::thread vio_process_thread;
    std::thread keyboard_command_process;
    std::thread posegraph_thread;
    std::thread data_process_thread;
    queue<KeyframeData> mque_keyframes;
    shared_ptr<Viewer> viewer_ptr;
    cv::Mat m_curr_img;
    int sequence = 1;
    PoseGraph posegraph;
    const bool LOAD_PREVIOUS_POSE_GRAPH = false;
    const bool enable_loopclose = true;
    std::mutex m_data_mutex;
};
