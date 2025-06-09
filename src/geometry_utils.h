#pragma once

#include <vector>
#include <string>
#include <tuple>
#include <Eigen/Dense>
#include "structures.h"

// 2D geometry utility functions
float distance2D(const Eigen::Vector2f &a, const Eigen::Vector2f &b);

std::vector<Eigen::Vector2f> filterBoundaryOutliers(
    const std::vector<Eigen::Vector2f>& points, 
    float multiplier = 2.0f
);

std::vector<Eigen::Vector2f> bidirectionalSort2D(
    const std::vector<Eigen::Vector2f>& points
);

std::vector<std::vector<Eigen::Vector2f>> splitPolygon2D(
    const std::vector<Eigen::Vector2f>& polygon, 
    float l_split
);

Eigen::Vector2f segmentDirection(const std::vector<Eigen::Vector2f>& seg);

std::tuple<int, int, float, std::string, std::string> findClosestSegments2D(
    const std::vector<std::vector<Eigen::Vector2f>>& segments, 
    float angleThresholdRadians = 0.5f
);

void reverseSegment2D(std::vector<Eigen::Vector2f>& seg);

std::vector<Eigen::Vector2f> connectSegments2D(
    const std::vector<Eigen::Vector2f>& segA,
    const std::vector<Eigen::Vector2f>& segB,
    const std::string& endpointA,
    const std::string& endpointB
);

bool isClosed2D(const std::vector<Eigen::Vector2f>& seg, float epsilon = 1e-3f);

std::vector<Eigen::Vector2f> PSR_SegmentBoundary2D(
    const std::vector<Eigen::Vector2f>& boundary,
    float k, 
    float q
);

// 3D geometry utility functions
void buildOrthonormalBasis(const Eigen::Vector3f& normal, Eigen::Matrix3f& basis);
std::vector<Eigen::Vector3f> calculateOBBVertices(const OBBData& obb);