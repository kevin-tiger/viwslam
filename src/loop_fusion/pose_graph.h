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

#pragma once
#include "utility/basetype.h"
#include "loop_fusion/keyframe.h"
#include "thirdparty/DBoW/DBoW2.h"
#include "thirdparty/DVision/DVision.h"
#include "thirdparty/DBoW/TemplatedDatabase.h"
#include "thirdparty/DBoW/TemplatedVocabulary.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "viewer/viewer.h"
#include "factor/angle_local_parameterization.h"
#include "factor/four_dof_factor.h"
#include "factor/four_dof_weight_factor.h"

using namespace DVision;
using namespace DBoW2;

class PoseGraph
{
public:
	PoseGraph();
	~PoseGraph();
    void addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
	void savePoseGraph();
	void loadPoseGraph();
	void SetViewerPtr(shared_ptr<Viewer> ptr);
private:
	void loadVocabulary(std::string voc_path);
    void addKeyFrameIntoVoc(KeyFrame* keyframe);
    int detectLoop(KeyFrame* keyframe, int frame_index);
	KeyFrame* getKeyFrame(int index);
	void optimize4DoF();
	void updatePath();
	void publish();
	void loadKeyFrame(KeyFrame* cur_kf);
public:
	Vector3d t_drift;
	double yaw_drift;
	Matrix3d r_drift;
	// world frame( base sequence or first sequence)<----> cur sequence frame  
	Vector3d w_t_vio;
	Matrix3d w_r_vio;
private:
	list<KeyFrame*> keyframelist;
	std::mutex m_keyframelist;
	std::mutex m_optimize_buf;
	std::mutex m_path;
	std::mutex m_drift;
	std::thread t_optimization;
	std::queue<int> optimize_buf;

	int global_index;
	int sequence_cnt;
	vector<bool> sequence_loop;
	map<int, cv::Mat> image_pool;
	int earliest_loop_index;
	int base_sequence;
	bool use_imu;
	BriefDatabase db;
	BriefVocabulary* voc;
	const string vocabulary_file = "support_files/brief_k10L6.bin";
	const string POSE_GRAPH_SAVE_PATH = "output/pose_graph/";
	shared_ptr<Viewer> viewer_ptr;
};
