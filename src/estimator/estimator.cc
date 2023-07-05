/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "estimator.h"

Estimator::Estimator()
{
    data_process_thread = std::thread(&Estimator::data_process_loop, this);
    vio_process_thread = std::thread(&Estimator::vio_process_loop, this);
    keyboard_command_process = std::thread(&Estimator::command, this);
    posegraph_thread = std::thread(&Estimator::posegraph_loop, this);
    viewer_ptr = make_shared<Viewer>();
    featureTracker.SetViewerPtr(viewer_ptr);
    posegraph.SetViewerPtr(viewer_ptr);
    if(LOAD_PREVIOUS_POSE_GRAPH)
    {
        TicToc loadPoseGraph_time;
        posegraph.loadPoseGraph();
        cout << "loadPoseGraph cost : " << loadPoseGraph_time.toc()/1000 << "s" << endl;
    }
    prevTime = -1;
    curTime = 0;
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i]; 
    }
    tiv = TIV[0];
    riv = RIV[0];
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    g = G;
    featureTracker.readIntrinsicParameter(CAM_NAMES);
}

void Estimator::inputImage(double t, const cv::Mat &_img)
{
    m_data_mutex.lock();
    m_curr_img = _img.clone();
    imgBuf.push(make_pair(t, _img));
    m_data_mutex.unlock();
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    m_data_mutex.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    m_data_mutex.unlock();
}

void Estimator::inputVEL(double t, const Eigen::Vector3d &velVec)
{
    m_data_mutex.lock();
    velBuf.push(make_pair(t, velVec));
    m_data_mutex.unlock();
}

void Estimator::data_process_loop()
{
    cout << "data_process_thread start" << endl;
    cv::Mat curr_img;
    double curr_timestamp;
    while(1)
    {
        if(!imgBuf.empty())
        {
            m_data_mutex.lock();
            curr_img = imgBuf.front().second.clone();
            curr_timestamp = imgBuf.front().first;
            imgBuf.pop();
            m_data_mutex.unlock();
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
            featureFrame = featureTracker.trackImage(curr_timestamp, curr_img);
            m_data_mutex.lock();
            featureBuf.push(make_pair(curr_timestamp, featureFrame)); 
            m_data_mutex.unlock();
        }
        usleep(10);  // 10us
    }
}

void Estimator::vio_process_loop()
{
    while (1)
    {
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        vector<pair<double, Eigen::Vector3d>> velVector;
        if(!featureBuf.empty())
        {
            while(1)
            {
                if(!velBuf.empty() && featureBuf.front().first <= velBuf.back().first)
                    break;
                else
                {
                    // cout << "wait for wheel data..." << endl;
                    usleep(10);
                }
            }
            while(1)
            {
                if(!accBuf.empty() && featureBuf.front().first <= accBuf.back().first)
                    break;
                else
                {
                    // cout << "wait for imu data..." << endl;
                    usleep(10);
                }
            }
            m_data_mutex.lock();
            feature = featureBuf.front();
            curTime = feature.first;
            // cout << fixed << setprecision(3) << "curTime = " << curTime << endl;
            featureBuf.pop();
            // get imu data between last image and current image.
            while (accBuf.front().first <= prevTime)
            {
                accBuf.pop();
                gyrBuf.pop();
            }
            while (accBuf.front().first < curTime)
            {
                accVector.push_back(accBuf.front());
                accBuf.pop();
                gyrVector.push_back(gyrBuf.front());
                gyrBuf.pop();
            }
            accVector.push_back(accBuf.front()); // not pop but do push_back here
            gyrVector.push_back(gyrBuf.front());
            for(size_t i = 0; i < accVector.size(); i++)
            {
                double dt;
                if(i == 0)
                    dt = accVector[i].first - prevTime;
                else if (i == accVector.size() - 1)
                    dt = curTime - accVector[i - 1].first;
                else
                    dt = accVector[i].first - accVector[i - 1].first;
                // for every imu data, associate a wheel speed.
                associateVelForImuData(accVector[i].first, velVector);
                // pre-integrate the imu and wheel data.
                processIMU_with_wheel(accVector[i].first, dt, accVector[i].second, gyrVector[i].second, velVector[i].second);
            }
            processImage(feature.second, curTime);
            prevTime = curTime;
            pubKeyframe();
            m_data_mutex.unlock();
        }
        usleep(10); // 10us
    }
}

void Estimator::calcFeatureTrackTime()
{
    cout << "mvec_feature_track_time.size() = " << mvec_feature_track_time.size() << endl;
    sort(mvec_feature_track_time.begin(), mvec_feature_track_time.end());
    double max = *max_element(mvec_feature_track_time.begin(), mvec_feature_track_time.end());
    double min = *min_element(mvec_feature_track_time.begin(), mvec_feature_track_time.end());
    double sum = accumulate(begin(mvec_feature_track_time), end(mvec_feature_track_time), 0.0); 
    double mean =  sum / mvec_feature_track_time.size();
    double mid_value = mvec_feature_track_time[mvec_feature_track_time.size()/2];
    cout << "feature_track_time, max = " << max << "ms" << endl;
    cout << "feature_track_time, min = " << min << "ms" << endl;
    cout << "feature_track_time, mean = " << mean << "ms" << endl;
    cout << "feature_track_time, mid_value = " << mid_value << "ms" << endl;
    // for(int i=0; i<mvec_feature_track_time.size(); i++)
    // {
    //     cout << "feature_track_time, image_" << i << " --> " << mvec_feature_track_time[i] << "ms" << endl;
    // }
}

bool Estimator::associateVelForImuData(double t, vector<pair<double, Eigen::Vector3d>> &VelVector)
{
    double imu_time = t;
    // if wheel time is bigger than imu time, then we choose the wheel data which time is bigger than imu time.
    if(velBuf.back().first >= imu_time) 
    {
        // pop out the wheel data which time is smaller than imu time.
        while (velBuf.front().first < imu_time)  
        {
            velBuf.pop();
        }
        // get the wheel data which time is bigger than imu time.
        VelVector.push_back(velBuf.front());
    }
    else
    {
        // cout << __LINE__ << " --> wait for wheel data" << endl;
        // if wheel time is smaller than imu time, then we choose the wheel data which time is just smaller than imu time.
        VelVector.push_back(velBuf.back());
    }
    return true;
}

void Estimator::pubKeyframe()
{
    // pub camera pose, 2D-3D points of keyframe
    if (solver_flag == SolverFlag::NON_LINEAR && marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        KeyframeData keyframe_data;
        keyframe_data.index = keyframe_num;
        keyframe_data.timestamp = curTime;
        keyframe_data.image = m_curr_img;
        keyframe_data.q = Quaterniond(Rs[i]);
        keyframe_data.t =  Ps[i];
        vector<Vector3d> vec_pointcloud;
        for (auto &it_per_id : f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {
                keyframe_data.ids.push_back(it_per_id.feature_id);
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = Rs[imu_i] * (ric[0] * pts_i + tic[0])+ Ps[imu_i];
                vec_pointcloud.push_back(w_pts_i);
                cv::Point3f p_3d;
                p_3d.x = w_pts_i.x();
                p_3d.y = w_pts_i.y();
                p_3d.z = w_pts_i.z();
                keyframe_data.pts_w.push_back(p_3d);
                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                cv::Point2f pts_point2d;
                pts_point2d.x = it_per_id.feature_per_frame[imu_j].point.x();
                pts_point2d.y = it_per_id.feature_per_frame[imu_j].point.y();
                keyframe_data.pts_point2d.push_back(pts_point2d);
                cv::Point2f pts_uv;
                pts_uv.x = it_per_id.feature_per_frame[imu_j].uv.x();
                pts_uv.y = it_per_id.feature_per_frame[imu_j].uv.y();
                keyframe_data.pts_uv.push_back(pts_uv);
            }
        }
        Eigen::Isometry3d Twr(keyframe_data.q);
        Twr.pretranslate(keyframe_data.t);
        viewer_ptr->UpdateOdoPose(Twr);
        viewer_ptr->UpdatePointCloud(vec_pointcloud);
        if(enable_loopclose)
        {
            // process_loop_close(keyframe_data);
        }
    }
}

void Estimator::process_loop_close(KeyframeData keyframe_data)
{
    Matrix3d R = keyframe_data.q.toRotationMatrix();
    KeyFrame* keyframe = new KeyFrame(keyframe_data.timestamp, keyframe_data.index, keyframe_data.t, R, keyframe_data.image,
                                   keyframe_data.pts_w, keyframe_data.pts_uv, keyframe_data.pts_point2d, keyframe_data.ids, sequence);
    posegraph.addKeyFrame(keyframe, 1);//第二个参数代表是需要回环检测detect_loop 提取的FAST特征点
}

void Estimator::processIMU_with_wheel(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity ,const Eigen::Vector3d vel)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        vel_0 = vel;
    }
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, vel_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        Eigen::Vector3d velVec=vel;
        pre_integrations[frame_count]->push_back_wheels(dt, linear_acceleration, angular_velocity , velVec);
        tmp_pre_integration->push_back_wheels(dt, linear_acceleration, angular_velocity , velVec);
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        vel_velocity_buf[frame_count].push_back(velVec);
        // mid-integrate provide init value for status
        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    vel_0 = vel;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
{
    frame_num++;
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        keyframe_num++;
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
    }
    // cout << fixed << setprecision(3) << "processImage --> " << header << ", frame_count = " << frame_count << ", keyframe = " << 1-marginalization_flag << ", keyframe_num = " << keyframe_num 
    // << ", frame_num = " << frame_num << endl;
    Headers[frame_count] = header; 
    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, vel_0, Bas[frame_count], Bgs[frame_count]};
    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = initialStructure();
            if(result)
            {
                optimization();
                updateLatestStates();
                solver_flag = NON_LINEAR;
                slideWindow();
                cout << "Initialization_finish, frame_num = " << frame_num << ", keyframe_num = " << keyframe_num << endl;
            }
            else
                slideWindow();
        }
        if(frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
        }
    }
    else
    {
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric); // triangulate feature point
        optimization();
        set<int> removeIndex;
        outliersRejection(removeIndex); // remove the point which reprojection is big
        f_manager.removeOutlier(removeIndex);
        featureTracker.removeOutliers(removeIndex);
        slideWindow();
        updateLatestStates();
    }
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        if(var < 0.25)
        {
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        marginalization_flag = MARGIN_OLD;
        return false;
    }
    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;        
        if((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i])
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);
        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end()) 
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if(pts_3_vector.size() < 6)
        {
            return false;
        }
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            return false;
        }
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;       
    }
    if (visualInitialAlign())
        return true;
    else
    {
        return false;
    }
}

bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        return false;
    }
    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }
    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1, kfv=-1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        kv++;
        if(frame_i->second.is_key_frame)
        {
            kfv++;
            Vs[kfv] = frame_i->second.R * x.segment<3>(kv * 3);           
        }
    }
    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i]; 
        Vs[i] = rot_diff * Vs[i];
    }
    f_manager.clearDepth();
    f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
    return true;
}

bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * for_average_parallax > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;                
                return true;
            }
        }
    }
    return false;
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x(); // position
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z(); 
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x(); // rotation
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();
        para_SpeedBias[i][0] = Vs[i].x(); // velocity
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();
        para_SpeedBias[i][3] = Bas[i].x(); // acc bias
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();
        para_SpeedBias[i][6] = Bgs[i].x(); // gyro bias
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x(); // tic
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x(); // ric
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i); // depth
    para_Td[0][0] = td; // td
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]); // get rotation for the first frame in sliding window
    Vector3d origin_P0 = Ps[0]; // get position for the first frame in sliding window
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],para_Pose[0][3],para_Pose[0][4],para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x(); // get delta yaw
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],para_Pose[0][3],para_Pose[0][4],para_Pose[0][5]).toRotationMatrix().transpose();
    }
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix(); // for rotation
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],para_Pose[i][1] - para_Pose[0][1],para_Pose[i][2] - para_Pose[0][2]) + origin_P0; // for position
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],para_SpeedBias[i][1],para_SpeedBias[i][2]); // for velocity
        Bas[i] = Vector3d(para_SpeedBias[i][3],para_SpeedBias[i][4],para_SpeedBias[i][5]); // for acc bias
        Bgs[i] = Vector3d(para_SpeedBias[i][6],para_SpeedBias[i][7],para_SpeedBias[i][8]); // for gyro bias
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]); // for tic
        ric[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3], para_Ex_Pose[i][4], para_Ex_Pose[i][5]).normalized().toRotationMatrix(); // for ric
    }
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0]; // for feature depth
    f_manager.setDepth(dep);
    td = para_Td[0][0]; // for td
}

bool Estimator::failureDetection()
{
    return false;
}

void Estimator::optimization()
{
    vector2double();
    TicToc optimize_time;
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization); // add status for position and rotation
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS); // add status for velocity acc bias and gyro bias
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization); // add status for tic and ric
        problem.SetParameterBlockConstant(para_Ex_Pose[i]); // fix tic and ric to not to optimize them
    }
    problem.AddParameterBlock(para_Td[0], 1); // add status for td
    problem.SetParameterBlockConstant(para_Td[0]); // fix td to not to optimize it
    if (last_marginalization_info && last_marginalization_info->valid) // for marginalization
    {
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,last_marginalization_parameter_blocks); // add marginalization residual
    }
    for (int i = 0; i < frame_count; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0) // if pre_integration too long, its value is not very accurate
            continue;
        IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j],para_SpeedBias[j]); // add imu pre_integration residual
    }
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        ++feature_index;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]); // add reprojection residual
            }
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    // cout << fixed << setprecision(3) << "sliding window optimization cost : " << optimize_time.toc() << "ms" << endl;
    double2vector();
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();
        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        if (pre_integrations[1]->sum_dt < 10.0)
        {
            IMUEncoderFactor *imu_factor = new IMUEncoderFactor(pre_integrations[1]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,vector<double *>{para_Pose[0],para_SpeedBias[0],para_Pose[1],para_SpeedBias[1]},vector<int>{0, 1});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;
            ++feature_index;
            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            if (imu_i != 0)
                continue;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                if(imu_i != imu_j)
                {
                    Vector3d pts_j = it_per_frame.point;
                    ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                    vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                    vector<int>{0, 3});
                    marginalization_info->addResidualBlockInfo(residual_block_info);
                }
            }
        }
        marginalization_info->preMarginalize();
        marginalization_info->marginalize();
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if (last_marginalization_info &&std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {
            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
            marginalization_info->preMarginalize();
            marginalization_info->marginalize();
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);
                if(USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
                    vel_velocity_buf[i].swap(vel_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if(USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0,vel_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
                vel_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if(USE_IMU)
            {
                double velNormal = 0;
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    Vector3d tmp_vel_velocity = vel_velocity_buf[frame_count][i];
                    velNormal = velNormal  + tmp_vel_velocity.norm();
                }
                if(1)
                    velNormal = 1;
                for (unsigned int i = 0; (i < dt_buf[frame_count].size()) && (velNormal!=0); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
                    Vector3d tmp_vel_velocity = vel_velocity_buf[frame_count][i];
                    pre_integrations[frame_count - 1]->push_back_wheels(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity,tmp_vel_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                    vel_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }
                if(velNormal!=0)
                {
                    Vs[frame_count - 1] = Vs[frame_count];
                    Bas[frame_count - 1] = Bas[frame_count];
                    Bgs[frame_count - 1] = Bgs[frame_count];
                }

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, vel_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
                vel_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getVelInWorldFrame(Eigen::Vector3d &v)
{
    v=Vs[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    if(frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if(it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            if((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi; // convert the point in camera_i to world
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj); // convert the point from world to camera_ j
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>(); // calculate the residual in camera_ j
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
            }
        }
        double ave_err = err / errCnt;
        // if reprojection error is bigger than 3 pixel
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);
    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates()
{
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    while(!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
}

Eigen::Vector3d Estimator::toEuler(const Eigen::Matrix3d &R)
{
    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );
    bool singular = sy < 1e-6; 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    Eigen::Vector3d v_euler;
    v_euler.x() = x;
    v_euler.y() = y;
    v_euler.z() = z;
    return v_euler;
}

void Estimator::posegraph_loop()
{
    cout << "posegraph_thread start" << endl;
    while(1)
    {
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void Estimator::command()
{
    cout << "command thread start" << endl;
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            cout << "press key : " << c << endl;
            TicToc savePoseGraph_time;
            posegraph.savePoseGraph();
            cout << "savePoseGraph cost : " << savePoseGraph_time.toc()/1000 << "s" << endl;
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void Estimator::process()
{
    cout << "process thread start" << endl;
    while (true)
    {
    }
}
