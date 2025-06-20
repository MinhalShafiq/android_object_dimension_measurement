#pragma once

#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

struct OBBData {
    pcl::PointXYZRGB obb_min;
    pcl::PointXYZRGB obb_max;
    pcl::PointXYZRGB obb_pos; 
    Eigen::Matrix3f rot_matrix;
    bool valid = false;
    
    // Add timing information for OBB management
    std::chrono::steady_clock::time_point last_update_time;
    int frames_since_valid_detection = 0;
    float confidence_at_creation = 0.0f;
};

struct SharedData {
    std::mutex data_mutex;
    std::vector<CloudT::Ptr> clusters; 
    OBBData obb;
    std::atomic<bool> new_data_available{false};

    std::mutex video_mutex;
    int video_client_fd = -1;

    std::mutex pointCloud_mutex;
    int pointCloud_client_fd = -1;
};