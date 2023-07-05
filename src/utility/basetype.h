#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <vector>
#include <queue>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <thread>
#include <mutex>
#include <cmath>
#include <unistd.h>
#include <random>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "tic_toc.h"

using namespace std;
using namespace cv;
using namespace Eigen;

struct ImageData
{
    double timestamp;
    string img_path;
};

struct ImuData
{
    double timestamp;
    Vector3d acc;
    Vector3d gyro;
};

struct WheelData
{
    double timestamp;
    double left_count;
    double right_count;
    double left_speed;
    double right_speed;
    double velocity;
};

struct KeyframeData
{
    int index;
    double timestamp;
    cv::Mat image;
    Quaterniond q;
    Vector3d t;
    vector<double> ids;
    vector<cv::Point3f> pts_w;
    vector<cv::Point2f> pts_point2d;
    vector<cv::Point2f> pts_uv;
};