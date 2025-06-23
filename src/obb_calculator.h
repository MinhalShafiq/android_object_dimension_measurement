#pragma once

#include "structures.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <string>
#include <vector>
#include <iomanip>
#include <chrono>

// Validation structures
struct ObjectValidationResult {
    bool is_valid_object;
    float confidence_score;
    std::string rejection_reason;
};

// OBB calculation and manipulation functions
OBBData findMinimumOBB(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

OBBData temporallyStableOBB(
    const OBBData &prev, 
    const OBBData &current, 
    float alpha = 0.2f
);

OBBData smoothOBB(
    const OBBData &prev, 
    const OBBData &current, 
    float alpha
);

OBBData extendOBBToGround(
    const OBBData& obb, 
    const pcl::ModelCoefficients& ground_coefficients
);

// Enhanced validation and selection functions
ObjectValidationResult validateObjectCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

size_t selectBestObjectCluster(
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cluster_clouds,
    const OBBData& previous_obb
);

OBBData computeValidatedOBB(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    const pcl::ModelCoefficients& ground_coefficients,
    const OBBData& previous_obb
);

// Enhanced OBB computation with confidence tracking
OBBData computeValidatedOBBWithConfidence(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    const pcl::ModelCoefficients& ground_coefficients,
    const OBBData& previous_obb,
    float confidence_score
);

// OBB timeout and management functions
bool shouldInvalidateOBB(const OBBData& obb, int max_frames_without_detection = 5);

OBBData updateOBBWithDetection(
    const OBBData& previous_obb, 
    const OBBData& new_obb, 
    float confidence_score
);

OBBData getInvalidOBB();

// Statistics and debugging functions
void printValidationStatistics(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cluster_clouds);

// Helper functions for optimized minimum OBB calculation
float computeOBBAreaAtAngle(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Vector3f& centroid,
    const Eigen::Vector3f& verticalAxis,
    float angle
);

Eigen::Matrix3f computeBasisAtAngle(
    const Eigen::Vector3f& verticalAxis,
    float angle
);

// Enhanced validation and stability functions
OBBData validateAndOptimizeOBB(
    const OBBData& obb,
    float confidence_threshold = 0.7f
);

OBBData applyJitterReduction(
    const std::vector<OBBData>& recent_obbs,
    int window_size = 5
);