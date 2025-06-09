#include "geometry_utils.h"
#include "structures.h"
#include <algorithm>
#include <numeric>
#include <limits>
#include <cmath>

float distance2D(const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
    return (a - b).norm();
}

std::vector<Eigen::Vector2f> filterBoundaryOutliers(const std::vector<Eigen::Vector2f>& points, float multiplier) {
    if (points.empty()) return {};
    // Compute centroid.
    Eigen::Vector2f centroid(0.0f, 0.0f);
    for (const auto &p : points)
        centroid += p;
    centroid /= static_cast<float>(points.size());

    // Compute distances from centroid.
    std::vector<float> dists;
    dists.reserve(points.size());
    for (const auto &p : points)
        dists.push_back((p - centroid).norm());

    // Compute mean and standard deviation.
    float sum = std::accumulate(dists.begin(), dists.end(), 0.0f);
    float mean = sum / dists.size();
    float var = 0.0f;
    for (float d : dists)
        var += (d - mean) * (d - mean);
    float std_dev = std::sqrt(var / dists.size());

    float threshold = mean + multiplier * std_dev;
    std::vector<Eigen::Vector2f> filtered;
    filtered.reserve(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        if (dists[i] <= threshold) {
            filtered.push_back(points[i]);
        }
    }
    return filtered;
}

std::vector<Eigen::Vector2f> bidirectionalSort2D(const std::vector<Eigen::Vector2f>& points) {
    if(points.empty()) return {};
    std::vector<Eigen::Vector2f> sorted;
    std::vector<bool> used(points.size(), false);
    sorted.push_back(points[0]);
    used[0] = true;
    size_t remaining = points.size() - 1;
    while(remaining > 0) {
        Eigen::Vector2f front = sorted.front();
        Eigen::Vector2f back = sorted.back();
        float minDistFront = std::numeric_limits<float>::max();
        float minDistBack = std::numeric_limits<float>::max();
        int idxFront = -1, idxBack = -1;
        for(size_t i = 0; i < points.size(); i++){
            if(used[i]) continue;
            float dFront = distance2D(front, points[i]);
            if(dFront < minDistFront) { minDistFront = dFront; idxFront = i; }
            float dBack = distance2D(back, points[i]);
            if(dBack < minDistBack) { minDistBack = dBack; idxBack = i; }
        }
        if(minDistFront < minDistBack) {
            sorted.insert(sorted.begin(), points[idxFront]);
            used[idxFront] = true;
        } else {
            sorted.push_back(points[idxBack]);
            used[idxBack] = true;
        }
        remaining--;
    }
    return sorted;
}

std::vector<std::vector<Eigen::Vector2f>> splitPolygon2D(const std::vector<Eigen::Vector2f>& polygon, float l_split) {
    std::vector<std::vector<Eigen::Vector2f>> segments;
    std::vector<Eigen::Vector2f> current;
    for(size_t i = 0; i < polygon.size(); i++){
        current.push_back(polygon[i]);
        size_t next = (i + 1) % polygon.size();
        if(distance2D(polygon[i], polygon[next]) > l_split) {
            segments.push_back(current);
            current.clear();
        }
    }
    if(!current.empty()){
        segments.push_back(current);
    }
    return segments;
}

Eigen::Vector2f segmentDirection(const std::vector<Eigen::Vector2f>& seg) {
    if(seg.size() < 2) return Eigen::Vector2f(0, 0);
    return (seg.back() - seg.front()).normalized();
}

std::tuple<int, int, float, std::string, std::string>
findClosestSegments2D(const std::vector<std::vector<Eigen::Vector2f>>& segments, float angleThresholdRadians) {
    float minDist = std::numeric_limits<float>::max();
    int segA = -1, segB = -1;
    std::string endpointA = "", endpointB = "";
    for(size_t i = 0; i < segments.size(); i++){
        for(size_t j = i+1; j < segments.size(); j++){
            std::vector<std::pair<std::string, Eigen::Vector2f>> endpoints_i = {
                {"head", segments[i].front()}, {"tail", segments[i].back()}
            };
            std::vector<std::pair<std::string, Eigen::Vector2f>> endpoints_j = {
                {"head", segments[j].front()}, {"tail", segments[j].back()}
            };
            for (auto &ei : endpoints_i) {
                for (auto &ej : endpoints_j) {
                    float d = distance2D(ei.second, ej.second);
                    // Check directional consistency.
                    Eigen::Vector2f dir_i = segmentDirection(segments[i]);
                    Eigen::Vector2f dir_j = segmentDirection(segments[j]);
                    // Adjust direction based on endpoint labels.
                    if (ei.first == "head") dir_i = -dir_i;
                    if (ej.first == "head") dir_j = -dir_j;
                    float dotVal = std::clamp(dir_i.dot(dir_j), -1.0f, 1.0f);
                    float angle = std::acos(dotVal);
                    if (d < minDist && angle < angleThresholdRadians) {
                        minDist = d;
                        segA = i;
                        segB = j;
                        endpointA = ei.first;
                        endpointB = ej.first;
                    }
                }
            }
        }
    }
    return std::make_tuple(segA, segB, minDist, endpointA, endpointB);
}

void reverseSegment2D(std::vector<Eigen::Vector2f>& seg) {
    std::reverse(seg.begin(), seg.end());
}

std::vector<Eigen::Vector2f> connectSegments2D(const std::vector<Eigen::Vector2f>& segA,
                                               const std::vector<Eigen::Vector2f>& segB,
                                               const std::string& endpointA,
                                               const std::string& endpointB) {
    std::vector<Eigen::Vector2f> newSeg;
    if(endpointA == "tail" && endpointB == "head") {
        newSeg = segA;
        newSeg.insert(newSeg.end(), segB.begin(), segB.end());
    } else if(endpointA == "head" && endpointB == "tail") {
        newSeg = segB;
        newSeg.insert(newSeg.end(), segA.begin(), segA.end());
    } else if(endpointA == "head" && endpointB == "head") {
        std::vector<Eigen::Vector2f> segA_rev = segA;
        reverseSegment2D(segA_rev);
        newSeg = segA_rev;
        newSeg.insert(newSeg.end(), segB.begin(), segB.end());
    } else if(endpointA == "tail" && endpointB == "tail") {
        std::vector<Eigen::Vector2f> segB_rev = segB;
        reverseSegment2D(segB_rev);
        newSeg = segA;
        newSeg.insert(newSeg.end(), segB_rev.begin(), segB_rev.end());
    }
    return newSeg;
}

bool isClosed2D(const std::vector<Eigen::Vector2f>& seg, float epsilon) {
    if(seg.empty()) return false;
    return (distance2D(seg.front(), seg.back()) < epsilon);
}

std::vector<Eigen::Vector2f> PSR_SegmentBoundary2D(const std::vector<Eigen::Vector2f>& boundary,
    float k, float q) {
        if (boundary.size() < 3) {
        return boundary;  // Not enough points to form a valid polygon.
        }

        // Sort the boundary points.
        std::vector<Eigen::Vector2f> sortedPolygon = bidirectionalSort2D(boundary);

        // Compute edge lengths and statistics.
        std::vector<float> edgeLengths;
        edgeLengths.reserve(sortedPolygon.size());
        for (size_t i = 0; i < sortedPolygon.size(); i++) {
        size_t next = (i + 1) % sortedPolygon.size();
        edgeLengths.push_back(distance2D(sortedPolygon[i], sortedPolygon[next]));
        }
        float sum = std::accumulate(edgeLengths.begin(), edgeLengths.end(), 0.0f);
        float meanEdge = sum / edgeLengths.size();
        float var = 0.0f;
        for (float d : edgeLengths)
        var += (d - meanEdge) * (d - meanEdge);
        float stdEdge = std::sqrt(var / edgeLengths.size());

        float l_split = meanEdge + k * stdEdge;

        // Split the polygon at abnormal (long) edges.
        std::vector<std::vector<Eigen::Vector2f>> segments = splitPolygon2D(sortedPolygon, l_split);

        // Iteratively recombine segments using both spatial and directional constraints.
        float connectionThreshold = q * l_split;
        while (segments.size() > 1) {
        int segA_idx, segB_idx;
        float minDist;
        std::string epA, epB;
        std::tie(segA_idx, segB_idx, minDist, epA, epB) = findClosestSegments2D(segments, 0.5f);
        if (minDist > connectionThreshold) {
        break;
        }
        std::vector<Eigen::Vector2f> newSeg = connectSegments2D(segments[segA_idx], segments[segB_idx], epA, epB);
        if (segA_idx > segB_idx)
        std::swap(segA_idx, segB_idx);
        segments.erase(segments.begin() + segB_idx);
        segments.erase(segments.begin() + segA_idx);
        segments.push_back(newSeg);
        }

        // Choose the longest closed segment as the refined boundary.
        std::vector<Eigen::Vector2f> best;
        size_t bestSize = 0;
        for (const auto& seg : segments) {
        if (isClosed2D(seg) && seg.size() > bestSize) {
            best = seg;
            bestSize = seg.size();
        }
    }
        return best;
}

void buildOrthonormalBasis(const Eigen::Vector3f& normal, Eigen::Matrix3f& basis)
{
    Eigen::Vector3f zAxis = normal.normalized();
    Eigen::Vector3f arbitrary = (std::fabs(zAxis.x()) < 0.9f)
                              ? Eigen::Vector3f::UnitX()
                              : Eigen::Vector3f::UnitY();
    Eigen::Vector3f xAxis = zAxis.cross(arbitrary).normalized();
    Eigen::Vector3f yAxis = zAxis.cross(xAxis).normalized();

    basis.col(0) = xAxis;
    basis.col(1) = yAxis;
    basis.col(2) = zAxis;
}

std::vector<Eigen::Vector3f> calculateOBBVertices(const OBBData& obb)
{
    std::vector<Eigen::Vector3f> vertices(8);
    const float hx = (obb.obb_max.x - obb.obb_min.x) * 0.5f;
    const float hy = (obb.obb_max.y - obb.obb_min.y) * 0.5f;
    const float hz = (obb.obb_max.z - obb.obb_min.z) * 0.5f;
    Eigen::Vector3f center(obb.obb_pos.x, obb.obb_pos.y, obb.obb_pos.z);

    vertices[0] = center + obb.rot_matrix * Eigen::Vector3f(-hx, -hy, +hz);
    vertices[1] = center + obb.rot_matrix * Eigen::Vector3f(+hx, -hy, +hz);
    vertices[2] = center + obb.rot_matrix * Eigen::Vector3f(+hx, +hy, +hz);
    vertices[3] = center + obb.rot_matrix * Eigen::Vector3f(-hx, +hy, +hz);
    vertices[4] = center + obb.rot_matrix * Eigen::Vector3f(-hx, -hy, -hz);
    vertices[5] = center + obb.rot_matrix * Eigen::Vector3f(+hx, -hy, -hz);
    vertices[6] = center + obb.rot_matrix * Eigen::Vector3f(+hx, +hy, -hz);
    vertices[7] = center + obb.rot_matrix * Eigen::Vector3f(-hx, +hy, -hz);

    return vertices;
}