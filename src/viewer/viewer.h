#pragma once
#include "utility/basetype.h"
#include <pangolin/pangolin.h>

class Viewer
{
public:
    struct Config
    {
        float view_point_x = 0.0;
        float view_point_y = 0.0;
        float view_point_z = 750.0;
        float view_point_f = 500.0;
        float view_center_x = 0.0;
        float view_center_y = 0.0;
        float view_center_z = 0.0;
        float view_up_x = 0.0;
        float view_up_y = 1.0;
        float view_up_z = 0.0;
    };
    Viewer();
    void UpdateOdoPose(Isometry3d& pos);
    void UpdateLoopPose(vector<Isometry3d>& vec_loop_pose);
    void UpdateOfflineMapPose(Isometry3d& pos);
    void UpdateImage(cv::Mat& img);
    void UpdatePointCloud(vector<Vector3d> point_cloud);
private:
    void ViewerLoop();
    void ShowPoses(vector<Isometry3d>& vec_pos);
    void ShowLoopPoses(vector<Isometry3d>& vec_pos);
    void ShowOfflineMapPoses(vector<Isometry3d>& vec_pos);
    void ShowImage(cv::Mat image, double scale);
    void ShowPointcloud(vector<Vector3d> point_cloud);
private:
    Config m_config;
    std::thread* m_viewer_thread;
    std::mutex m_data_mutex;
    vector<Isometry3d> mvec_odo_pose;
    vector<Isometry3d> mvec_loop_pose;
    vector<Isometry3d> mvec_offlinemap_pose;
    cv::Mat m_image;
    vector<Vector3d> m_pointCloud;
};