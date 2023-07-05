#include "pose_graph.h"

PoseGraph::PoseGraph()
{
    t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    global_index = 0;
    sequence_cnt = 0;
    sequence_loop.push_back(0);
    base_sequence = 1;
    use_imu = 0;
    // loadVocabulary(vocabulary_file);
}

PoseGraph::~PoseGraph()
{
}

void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    //shift to base frame
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    if (sequence_cnt != cur_kf->sequence)//如果sequence_cnt != cur_kf->sequence，则新建一个新的图像序列;
    {
        // cout << "sequence_cnt = " << sequence_cnt << endl;
        // cout << "cur_kf->sequence = " << cur_kf->sequence << endl;
        sequence_cnt++;
        sequence_loop.push_back(0);
        w_t_vio = Eigen::Vector3d(0, 0, 0);// w_t_vio,w_r_vio描述的就是当前序列的第一帧，与世界坐标系之间的转换关系。
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }
    //获取当前帧的位姿vio_P_cur、vio_R_cur并更新
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;
	int loop_index = -1;
    if (flag_detect_loop)
    {
        //回环检测，返回回环候选帧的索引
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
    //得到匹配上关键帧后，经过计算相对位姿，并把当前帧号记录到全局优化内
    //如果存在回环候选帧，将当前帧与回环帧进行描述子匹配并计算位姿，并执行优化
	if (loop_index != -1)
	{
        // cout << "cur_kf_"<< cur_kf->index << " detect loop with old_kf_" << loop_index << endl;
        KeyFrame* old_kf = getKeyFrame(loop_index);//返回对应关键帧的地址,//获取回环候选帧
        // findConnection 是为了计算相对位姿，最主要的就是利用了PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old)函数，
        //并且它负责把匹配好的点发送到estimator节点中去
        if (cur_kf->findConnection(old_kf))////当前帧与回环候选帧进行描述子匹配  来确定是否是一个真正的闭环
        {
            // cout << "cur_kf->findConnection(old_kf) == true" << endl;
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)//earliest_loop_index为最早的回环候选帧
                earliest_loop_index = loop_index;
            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);
            ////获取当前帧与回环帧的相对位姿relative_q、relative_t
            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            //重新计算当前帧位姿w_P_cur、w_R_cur
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;

            //回环得到的位姿和VIO位姿之间的偏移量shift_r、shift_t
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            // 根据old frame 和相对位姿能计算出当前帧位姿，也就能得出和已知当前帧位姿的差别
            //分别计算出shift_r, shift_t，用来更新其他帧位姿
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur; 
            // shift vio pose of whole sequence to the world frame将整个序列的vio姿势转移到世界坐标系
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)//正常回环检测不执行这个
            {  
                w_r_vio = shift_r;
                w_t_vio = shift_t;
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;//这里是不是多了？
                vio_R_cur = w_r_vio *  vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                list<KeyFrame*>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)   
                {
                    if((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio *  vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                }
                sequence_loop[cur_kf->sequence] = 1;
            }
            // m_optimize_buf.lock();
            // // cout << "optimize_buf.push(cur_kf->index) --> " << cur_kf->index << endl;
            // optimize_buf.push(cur_kf->index);  
            // m_optimize_buf.unlock();
        }
    }
    Vector3d P;
    Matrix3d R;

    cur_kf->getVioPose(P, R); //获取VIO当前帧的位姿P、R，根据偏移量得到实际位姿
    P = r_drift * P + t_drift;//在optimize6DoF线程中进行了赋值
    R = r_drift * R;
    // std::cout<<"r_drift=\n"<<r_drift<<"\nt_drift="<<t_drift.transpose() <<std::endl;
    cur_kf->updatePose(P, R);//更新当前帧的位姿P、R到T_w_i R_w_i

    // cout << "w_t_vio = " << w_t_vio.transpose() << endl; // offline map affect
    // cout << "t_drift = " << t_drift.transpose() << endl; // loop close affect
    // Eigen::Isometry3d Twr(R);
    // Twr.pretranslate(P);
    // viewer_ptr->UpdateOdoPose(Twr);
    keyframelist.push_back(cur_kf);
    m_optimize_buf.lock();
    // cout << "optimize_buf.push(cur_kf->index) --> " << cur_kf->index << endl;
    optimize_buf.push(cur_kf->index);  
    m_optimize_buf.unlock();
}

void PoseGraph::optimize4DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            TicToc optimize_time;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);
            int max_length = cur_index + 1;
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.max_num_iterations = 50;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

            list<KeyFrame*>::iterator it;
            int i = 0;
            // cout << "keyframelist.size() = " << keyframelist.size() << endl;
            // cout << "first_looped_index = " << first_looped_index << endl;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                // cout << i << " --> (*it)->index = " << (*it)->index << endl;
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i; // local_index for optimization
                Quaterniond tmp_q; 
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r); // get vio pose which is in offline map coordinate
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;
                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();
                sequence_array[i] = (*it)->sequence;
                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
                // cout << i << ", " << (*it)->index << ", " <<  (*it)->sequence << endl;
                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {   
                    problem.SetParameterBlockConstant(euler_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }
                else
                {
                    // cout << "AddParameterBlock to non constant --> " << i << endl;
                }
                //add sequence edge
                for (int j = 1; j < 5; j++)
                {
                  if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                  {
                    // cout << "add_sequence_edge_between " << i << " and " << i-j << endl;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j], 
                                            t_array[i-j], 
                                            euler_array[i], 
                                            t_array[i]);
                  }
                }
                //add loop edge
                if((*it)->has_loop)
                {
                    assert((*it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    // cout << "add_loop_edge_between " << (*it)->local_index << " and " << connected_index << endl;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    double relative_yaw = (*it)->getLoopRelativeYaw();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index], 
                                                                  t_array[connected_index], 
                                                                  euler_array[i], 
                                                                  t_array[i]);
                }
                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();
            ceres::Solve(options, &problem, &summary);
            // cout << summary.BriefReport() << endl;
            // cout << "summary.FullReport() : " << endl << summary.FullReport() << endl;
            // cout << fixed << setprecision(3) << "optimize_pose_graph --> kf_" << cur_kf->index << ", cost " << optimize_time.toc() << "ms" << endl;
            m_keyframelist.lock();
            // update the kf pose by optimized pose
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r); // update kf pose
                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r); // get current pose which optimized by loop closing
            cur_kf->getVioPose(vio_t, vio_r); // get vio pose which already converted in offline map coordinate
            m_drift.lock();
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x(); // get yaw drift
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0)); // get r drift
            t_drift = cur_t - r_drift * vio_t; // get t drift
            m_drift.unlock();
            // for the other kf that is newer than current kf
            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift; // convert by drift
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
    return;
}

void PoseGraph::updatePath()
{
    list<KeyFrame*>::iterator it;
    vector<Isometry3d> vec_loop_pose;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        if ((*it)->sequence == 0) continue; // not send the offline map path
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Eigen::Isometry3d Twr(R);
        Twr.pretranslate(P);
        vec_loop_pose.push_back(Twr);
    }
    // cout << "vec_loop_pose.size() = " << vec_loop_pose.size() << endl;
    // cout << __LINE__ << " --> viewer_ptr->UpdateLoopPose(vec_loop_pose)" << endl;
    viewer_ptr->UpdateLoopPose(vec_loop_pose);
}

void PoseGraph::publish()
{
}

void PoseGraph::savePoseGraph()
{
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    FILE *pFile = fopen (file_path.c_str(),"w");
    list<KeyFrame*>::iterator it;
    cout << "savePoseGraph start, keyframelist.size() = " << keyframelist.size() << endl;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string brief_path, keypoints_path;
        Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
        Quaterniond PG_tmp_Q{(*it)->R_w_i};
        Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
        Vector3d PG_tmp_T = (*it)->T_w_i;
        fprintf (pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d\n",(*it)->index, (*it)->time_stamp, 
                                    VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(), 
                                    PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(), 
                                    VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(), 
                                    PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(), 
                                    (*it)->loop_index, 
                                    (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                                    (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
                                    (int)(*it)->keypoints.size());
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;
            fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y, 
                                                     (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();
        fclose(keypoints_file);
    }
    fclose(pFile);
}

void PoseGraph::loadPoseGraph()
{
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    cout << "loadPoseGraph from file : " << file_path << endl;
    pFile = fopen (file_path.c_str(),"r");
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp, 
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz, 
                                    &PG_Tx, &PG_Ty, &PG_Tz, 
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz, 
                                    &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz, 
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3, 
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num) != EOF) 
    {
        cv::Mat image;
        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;
        if (loop_index != -1)
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }
        }
        // load keypoints, brief_descriptors   
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                cout << "load pose graph fail" << endl;
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);
        // cout << "new keyframe, index = " << index << ", loop_index = " << loop_index << endl;
        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        loadKeyFrame(keyframe);
        // if (cnt % 20 == 0)
        {
            Eigen::Isometry3d Twr(PG_R);
            Twr.pretranslate(PG_T);
            viewer_ptr->UpdateOfflineMapPose(Twr);
        }
        cnt++;
    }
    fclose (pFile);
    base_sequence = 0;
}

void PoseGraph::SetViewerPtr(shared_ptr<Viewer> ptr)
{
    viewer_ptr = ptr;
}

void PoseGraph::loadKeyFrame(KeyFrame* cur_kf)
{
    cur_kf->index = global_index;
    global_index++;
    addKeyFrameIntoVoc(cur_kf);
    keyframelist.push_back(cur_kf);
}

KeyFrame* PoseGraph::getKeyFrame(int index)
{
//    unique_lock<mutex> lock(m_keyframelist);
    // cout << "keyframelist.size() = " << keyframelist.size() << endl;
    // for(list<KeyFrame*>::iterator it = keyframelist.begin(); it != keyframelist.end(); it++)
    // {
    //     cout << "(*it)->index = " << (*it)->index << endl;
    // }

    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

void PoseGraph::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    db.add(keyframe->brief_descriptors);
}

int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index)//输入关键帧和关键帧的索引
{
    // cout << "detectLoop..." << endl;
    //first query; then add this frame into database! //首先查询；然后将此坐标系添加到数据库中！
    QueryResults ret;//    查询的多个结果
    //第一个参数是描述子，第二个是检测结果，第三个是结果个数，第四个是结果帧号必须小于此  ret=1 result:<EntryId: 18, Score: 0.113851>
    // cout << "keyframe->brief_descriptors = \n" << keyframe->brief_descriptors << endl;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);
    db.add(keyframe->brief_descriptors);//属于namespace DBoW2
    bool find_loop = false;
    // a good match with its nerghbour    //找到最小帧号的匹配帧
    if (ret.size() >= 1 &&ret[0].Score > 0.05)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            if (ret[i].Score > 0.015)
            {          
                find_loop = true;
                int tmp_index = ret[i].Id;
            }
        }
    }
    if (find_loop && frame_index > 50)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;
}