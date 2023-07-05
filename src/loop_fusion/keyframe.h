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
#include "thirdparty/DBoW/DBoW2.h"
#include "thirdparty/DBoW/FBrief.h"
#include "thirdparty/DVision/DVision.h"
#include "camera/Camera.h"
#include "camera/CameraFactory.h"
#include "estimator/parameters.h"

using namespace DVision;

class BriefExtractor
{
public:
  virtual void operator()(const cv::Mat &im, vector<cv::KeyPoint> &keys, vector<BRIEF::bitset> &descriptors) const;
  BriefExtractor(const std::string &pattern_file);
  DVision::BRIEF m_brief;
};

class KeyFrame
{
public:
	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, cv::Mat &_image,
			 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_point_2d_uv, vector<cv::Point2f> &_point_2d_normal, 
			 vector<double> &_point_id, int _sequence);
    KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
					cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info,
					vector<cv::KeyPoint> &_keypoints, vector<cv::KeyPoint> &_keypoints_norm, vector<BRIEF::bitset> &_brief_descriptors);
	void getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	bool findConnection(KeyFrame* old_kf);
	Eigen::Vector3d getLoopRelativeT();
	Eigen::Quaterniond getLoopRelativeQ();
	double getLoopRelativeYaw();
private:
	void computeWindowBRIEFPoint();
	void computeBRIEFPoint();
	void searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
									std::vector<cv::Point2f> &matched_2d_old_norm,
									std::vector<uchar> &status,
									const std::vector<BRIEF::bitset> &descriptors_old,
									const std::vector<cv::KeyPoint> &keypoints_old,
									const std::vector<cv::KeyPoint> &keypoints_old_norm);
	int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);
	bool searchInAera(const BRIEF::bitset window_descriptor,
								const std::vector<BRIEF::bitset> &descriptors_old,
								const std::vector<cv::KeyPoint> &keypoints_old,
								const std::vector<cv::KeyPoint> &keypoints_old_norm,
								cv::Point2f &best_match,
								cv::Point2f &best_match_norm);
	void PnPRANSAC(const vector<cv::Point2f> &matched_2d_old_norm,
                         const std::vector<cv::Point3f> &matched_3d,
                         std::vector<uchar> &status,
                         Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old);
public:
	int sequence;
	int index;
	vector<cv::KeyPoint> keypoints;	// fast角点的像素坐标
	vector<BRIEF::bitset> brief_descriptors;	// 额外提取的fast特征点的描述子
	vector<cv::KeyPoint> keypoints_norm;	// fast角点对应的归一化相机系坐标
	int local_index;
	int loop_index;
	bool has_loop;
	Eigen::Vector3d vio_T_w_i; 
	Eigen::Matrix3d vio_R_w_i; 
	Eigen::Vector3d T_w_i;
	Eigen::Matrix3d R_w_i;
	Eigen::Matrix<double, 8, 1 > loop_info;
	double time_stamp;
private:
	 
	Eigen::Vector3d origin_vio_T;		// 原始VIO结果的位姿
	Eigen::Matrix3d origin_vio_R;
	cv::Mat image;
	cv::Mat thumbnail;
	vector<cv::Point3f> point_3d; 
	vector<cv::Point2f> point_2d_uv;
	vector<cv::Point2f> point_2d_norm;
	vector<double> point_id;
	
	
	vector<cv::KeyPoint> window_keypoints;
	vector<BRIEF::bitset> window_brief_descriptors;	// 原来光流追踪的特征点的描述子
	bool has_fast_point;
	
	
	
	const string BRIEF_PATTERN_FILE = "support_files/brief_pattern.yml";
	CameraPtr m_camera;
	const int MIN_LOOP_NUM = 25;
	Eigen::Vector3d tic;
	Eigen::Matrix3d qic;
};