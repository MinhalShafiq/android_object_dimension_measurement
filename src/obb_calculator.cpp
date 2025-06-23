#include "obb_calculator.h"
#include "geometry_utils.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <iostream>
#include <limits>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <pcl/common/io.h>

ObjectValidationResult validateObjectCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    ObjectValidationResult result;
    result.is_valid_object = false;
    result.confidence_score = 0.0f;
    
    if (!cloud || cloud->empty()) {
        result.rejection_reason = "Empty cloud";
        return result;
    }
    
    // Get bounding box dimensions
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    float width = max_pt[0] - min_pt[0];
    float depth = max_pt[1] - min_pt[1];
    float height = max_pt[2] - min_pt[2];
    
    // ADAPTIVE SIZE CONSTRAINTS - much more permissive
    const float MAX_DIMENSION = 2500.0f; // Keep 2.5m as absolute maximum
    const float MIN_DIMENSION = 50.0f;   // Reduced to 2cm minimum (was 5cm)
    
    if (width > MAX_DIMENSION || depth > MAX_DIMENSION || height > MAX_DIMENSION) {
        result.rejection_reason = "Object too large: " + std::to_string(std::max({width, depth, height})) + "mm";
        return result;
    }
    
    if (width < MIN_DIMENSION && depth < MIN_DIMENSION && height < MIN_DIMENSION) {
        result.rejection_reason = "Object too small (all dimensions < " + std::to_string(MIN_DIMENSION) + "mm)";
        return result;
    }
    
    // ADAPTIVE ASPECT RATIO - more permissive based on object size
    float max_dim = std::max({width, depth, height});
    float min_dim = std::min({width, depth, height});
    float aspect_ratio = max_dim / min_dim;
    
    // Allow higher aspect ratios for smaller objects (could be thin objects)
    // Allow lower aspect ratios for larger objects (likely walls if too elongated)
    float max_allowed_aspect_ratio;
    if (max_dim < 200.0f) {        // Objects smaller than 20cm
        max_allowed_aspect_ratio = 50.0f;  // Very permissive
    } else if (max_dim < 500.0f) { // Objects 20-50cm
        max_allowed_aspect_ratio = 25.0f;  // Moderately permissive
    } else if (max_dim < 1000.0f) { // Objects 50cm-1m
        max_allowed_aspect_ratio = 15.0f;  // Standard restriction
    } else {                       // Objects larger than 1m
        max_allowed_aspect_ratio = 8.0f;   // Strict (likely walls if too elongated)
    }
    
    if (aspect_ratio > max_allowed_aspect_ratio) {
        result.rejection_reason = "Invalid aspect ratio: " + std::to_string(aspect_ratio) + 
                                 " (max allowed: " + std::to_string(max_allowed_aspect_ratio) + 
                                 " for size " + std::to_string(max_dim) + "mm)";
        return result;
    }
    
    // ADAPTIVE POINT DENSITY - much more realistic thresholds
    float volume = width * depth * height;
    float point_density = cloud->size() / volume; // points per cubic mm
    
    // Calculate adaptive density thresholds based on object size and typical sensor characteristics
    float min_density, max_density;
    
    if (max_dim < 100.0f) {        // Very small objects (< 10cm)
        min_density = 0.00001f;    // Very permissive for small objects
        max_density = 0.5f;        // Allow dense small objects
    } else if (max_dim < 300.0f) { // Small objects (10-30cm)
        min_density = 0.00005f;    // Slightly more restrictive
        max_density = 0.2f;        
    } else if (max_dim < 800.0f) { // Medium objects (30-80cm)
        min_density = 0.0001f;     // Standard restriction
        max_density = 0.1f;        
    } else {                       // Large objects (> 80cm)
        min_density = 0.00005f;    // Large objects can be sparse
        max_density = 0.05f;       // But not too dense (likely noise if very dense)
    }
    
    if (point_density < min_density) {
        result.rejection_reason = "Point density too low: " + std::to_string(point_density) + 
                                 " (min required: " + std::to_string(min_density) + 
                                 " for size " + std::to_string(max_dim) + "mm)";
        return result;
    }
    
    if (point_density > max_density) {
        result.rejection_reason = "Point density too high: " + std::to_string(point_density) + 
                                 " (max allowed: " + std::to_string(max_density) + 
                                 " for size " + std::to_string(max_dim) + "mm)";
        return result;
    }
    
    // ADAPTIVE HEIGHT DISTRIBUTION - more permissive for smaller objects
    std::vector<float> z_values;
    z_values.reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        z_values.push_back(pt.z);
    }
    
    if (z_values.size() > 4) { // Only check if we have enough points
        std::sort(z_values.begin(), z_values.end());
        float height_p25 = z_values[z_values.size() / 4];
        float height_p75 = z_values[3 * z_values.size() / 4];
        float height_iqr = height_p75 - height_p25;
        
        // Adaptive height distribution check based on object height
        float min_height_ratio;
        if (height < 50.0f) {      // Very flat objects (< 5cm height)
            min_height_ratio = 0.02f; // Very permissive (2%)
        } else if (height < 150.0f) { // Moderately flat objects (5-15cm)
            min_height_ratio = 0.05f; // Moderately permissive (5%)
        } else {                   // Taller objects
            min_height_ratio = 0.1f;  // Standard restriction (10%)
        }
        
        if (height_iqr < height * min_height_ratio) {
            result.rejection_reason = "Object too flat (height IQR: " + std::to_string(height_iqr) + 
                                     "mm, " + std::to_string(100.0f * height_iqr / height) + 
                                     "% of height, min required: " + std::to_string(100.0f * min_height_ratio) + "%)";
            return result;
        }
    }
    
    // ENHANCED CONFIDENCE SCORING with adaptive weights
    float size_score, aspect_score, density_score;
    
    // Size score - favor objects in the sweet spot (10cm - 1m)
    if (max_dim < 100.0f) {
        size_score = 0.6f + 0.4f * (max_dim / 100.0f); // 0.6-1.0 for small objects
    } else if (max_dim < 1000.0f) {
        size_score = 1.0f; // Perfect score for medium objects
    } else {
        size_score = std::max(0.2f, 1.0f - (max_dim - 1000.0f) / 1500.0f); // Decline for large objects
    }
    
    // Aspect score - penalize extreme aspect ratios but be more forgiving
    float normalized_aspect = (aspect_ratio - 1.0f) / (max_allowed_aspect_ratio - 1.0f);
    aspect_score = std::max(0.0f, 1.0f - normalized_aspect);
    
    // Density score - favor densities in the middle of the allowed range
    float density_range = max_density - min_density;
    float normalized_density = (point_density - min_density) / density_range;
    // Peak score at 30% of the range, then decline
    if (normalized_density < 0.3f) {
        density_score = normalized_density / 0.3f;
    } else {
        density_score = 1.0f - 0.5f * (normalized_density - 0.3f) / 0.7f;
    }
    density_score = std::max(0.1f, std::min(1.0f, density_score));
    
    // ADAPTIVE CONFIDENCE THRESHOLD
    float base_confidence = (size_score + aspect_score + density_score) / 3.0f;
    
    // Lower confidence threshold for smaller objects (they're harder to detect perfectly)
    float confidence_threshold;
    if (max_dim < 100.0f) {
        confidence_threshold = 0.25f; // Very permissive for small objects
    } else if (max_dim < 300.0f) {
        confidence_threshold = 0.35f; // Moderately permissive
    } else if (max_dim < 800.0f) {
        confidence_threshold = 0.45f; // Standard threshold
    } else {
        confidence_threshold = 0.55f; // Stricter for large objects
    }
    
    result.confidence_score = base_confidence;
    result.is_valid_object = result.confidence_score > confidence_threshold;
    
    if (!result.is_valid_object) {
        result.rejection_reason = "Low confidence score: " + std::to_string(result.confidence_score) + 
                                 " (threshold: " + std::to_string(confidence_threshold) + 
                                 " for size " + std::to_string(max_dim) + "mm)";
    }
    
    // DEBUG OUTPUT for tuning
    std::cout << "  Validation details - Size: " << width << "x" << depth << "x" << height 
              << "mm, Density: " << point_density << " pts/mm³, Aspect: " << aspect_ratio
              << ", Scores: size=" << size_score << " aspect=" << aspect_score 
              << " density=" << density_score << " final=" << result.confidence_score 
              << " (threshold=" << confidence_threshold << ")" << std::endl;
    
    return result;
}

void printValidationStatistics(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cluster_clouds) {
    if (cluster_clouds.empty()) return;
    
    std::cout << "\n=== Validation Statistics ===" << std::endl;
    
    for (size_t i = 0; i < cluster_clouds.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cluster_clouds[i], *xyz_cloud);
        
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*xyz_cloud, min_pt, max_pt);
        
        float width = max_pt[0] - min_pt[0];
        float depth = max_pt[1] - min_pt[1];
        float height = max_pt[2] - min_pt[2];
        float volume = width * depth * height;
        float density = xyz_cloud->size() / volume;
        float max_dim = std::max({width, depth, height});
        float min_dim = std::min({width, depth, height});
        float aspect = max_dim / min_dim;
        
        std::cout << "Cluster " << i << ": " << xyz_cloud->size() << " points, "
                  << width << "x" << depth << "x" << height << "mm, "
                  << "density=" << density << ", aspect=" << aspect << std::endl;
    }
    
    std::cout << "============================\n" << std::endl;
}

// Enhanced cluster selection that considers multiple factors
struct ClusterCandidate {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    size_t original_index;
    float distance_to_origin;
    float confidence_score;
    bool is_valid;
    std::string rejection_reason;
};

size_t selectBestObjectCluster(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cluster_clouds,
                               const OBBData& previous_obb) {
    
    // Print detailed statistics first
    printValidationStatistics(cluster_clouds);
    
    std::vector<ClusterCandidate> candidates;
    candidates.reserve(cluster_clouds.size());
    
    // Evaluate all clusters
    for (size_t i = 0; i < cluster_clouds.size(); ++i) {
        ClusterCandidate candidate;
        candidate.cloud = cluster_clouds[i];
        candidate.original_index = i;
        
        // Convert to XYZ for validation
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cluster_clouds[i], *xyz_cloud);
        
        // Validate the cluster
        ObjectValidationResult validation = validateObjectCluster(xyz_cloud);
        candidate.is_valid = validation.is_valid_object;
        candidate.confidence_score = validation.confidence_score;
        candidate.rejection_reason = validation.rejection_reason;
        
        // Calculate 2D distance to origin
        double x_sum = 0.0, y_sum = 0.0;
        for (const auto &pt : cluster_clouds[i]->points) {
            x_sum += pt.x;
            y_sum += pt.y;
        }
        double count = static_cast<double>(cluster_clouds[i]->size());
        double centroidX = x_sum / count;
        double centroidY = y_sum / count;
        candidate.distance_to_origin = std::sqrt(centroidX * centroidX + centroidY * centroidY);
        
        candidates.push_back(candidate);
        
        std::cout << "Cluster " << i << ": " 
                  << (candidate.is_valid ? "VALID" : "INVALID") 
                  << " (score: " << std::fixed << std::setprecision(3) << candidate.confidence_score 
                  << ", dist: " << std::fixed << std::setprecision(1) << candidate.distance_to_origin << "mm)";
        if (!candidate.is_valid) {
            std::cout << " - " << candidate.rejection_reason;
        }
        std::cout << std::endl;
    }
    
    // Filter valid candidates
    std::vector<ClusterCandidate*> valid_candidates;
    for (auto& candidate : candidates) {
        if (candidate.is_valid) {
            valid_candidates.push_back(&candidate);
        }
    }
    
    if (valid_candidates.empty()) {
        std::cout << "No valid object clusters found! Consider adjusting validation parameters." << std::endl;
        
        // FALLBACK: If no clusters are valid, try to find the "best bad" cluster
        // This prevents the system from completely failing
        std::cout << "Attempting fallback selection of least-bad cluster..." << std::endl;
        
        ClusterCandidate* best_fallback = nullptr;
        float best_fallback_score = -1.0f;
        
        for (auto& candidate : candidates) {
            // Skip clusters that are too large (definitely walls/noise)
            if (candidate.rejection_reason.find("too large") != std::string::npos) {
                continue;
            }
            
            // Prefer clusters with higher confidence scores even if they failed validation
            if (candidate.confidence_score > best_fallback_score) {
                best_fallback_score = candidate.confidence_score;
                best_fallback = &candidate;
            }
        }
        
        if (best_fallback) {
            std::cout << "FALLBACK: Selected cluster " << best_fallback->original_index 
                      << " with score " << best_fallback->confidence_score 
                      << " (reason for rejection: " << best_fallback->rejection_reason << ")" << std::endl;
            return best_fallback->original_index;
        }
        
        return SIZE_MAX; // Still no valid clusters
    }
    
    std::cout << "Found " << valid_candidates.size() << " valid clusters" << std::endl;
    
    // If we have a previous OBB, prefer clusters close to the previous position
    if (previous_obb.valid) {
        Eigen::Vector3f prev_center(previous_obb.obb_pos.x, previous_obb.obb_pos.y, previous_obb.obb_pos.z);
        
        float best_combined_score = -1.0f;
        ClusterCandidate* best_candidate = nullptr;
        
        for (auto* candidate : valid_candidates) {
            // Calculate distance to previous OBB center
            double x_sum = 0.0, y_sum = 0.0, z_sum = 0.0;
            for (const auto &pt : candidate->cloud->points) {
                x_sum += pt.x;
                y_sum += pt.y;
                z_sum += pt.z;
            }
            double count = static_cast<double>(candidate->cloud->size());
            Eigen::Vector3f curr_center(x_sum/count, y_sum/count, z_sum/count);
            
            float distance_to_prev = (curr_center - prev_center).norm();
            
            // Combined score: prefer high confidence + close to previous position
            float distance_score = 1.0f / (1.0f + distance_to_prev / 1000.0f); // Normalize by 1m
            float combined_score = 0.7f * candidate->confidence_score + 0.3f * distance_score;
            
            std::cout << "Cluster " << candidate->original_index 
                      << " combined score: " << std::fixed << std::setprecision(3) << combined_score 
                      << " (conf: " << candidate->confidence_score 
                      << ", dist to prev: " << std::fixed << std::setprecision(1) << distance_to_prev << "mm)" << std::endl;
            
            if (combined_score > best_combined_score) {
                best_combined_score = combined_score;
                best_candidate = candidate;
            }
        }
        
        if (best_candidate) {
            std::cout << "Selected cluster " << best_candidate->original_index 
                      << " based on temporal continuity (score: " << best_combined_score << ")" << std::endl;
            return best_candidate->original_index;
        }
    }
    
    // Fallback: select the valid cluster with highest confidence score
    auto best_it = std::max_element(valid_candidates.begin(), valid_candidates.end(),
        [](const ClusterCandidate* a, const ClusterCandidate* b) {
            return a->confidence_score < b->confidence_score;
        });
    
    std::cout << "Selected cluster " << (*best_it)->original_index 
              << " with highest confidence score: " << std::fixed << std::setprecision(3) 
              << (*best_it)->confidence_score << std::endl;
    
    return (*best_it)->original_index;
}

bool shouldInvalidateOBB(const OBBData& obb, int max_frames_without_detection) {
    if (!obb.valid) return false;
    
    // If we haven't seen a valid detection for too many frames, invalidate
    if (obb.frames_since_valid_detection > max_frames_without_detection) {
        return true;
    }
    
    // Also check time-based timeout (optional - frames are usually more reliable)
    auto now = std::chrono::steady_clock::now();
    auto time_since_update = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - obb.last_update_time).count();
    
    // If no update for more than 2 seconds, invalidate
    if (time_since_update > 2000) {
        return true;
    }
    
    return false;
}

OBBData updateOBBWithDetection(const OBBData& previous_obb, const OBBData& new_obb, 
                                float confidence_score) {
    OBBData updated_obb;
    
    if (new_obb.valid) {
        // We have a new valid detection
        if (previous_obb.valid) {
            // Apply temporal smoothing
            updated_obb = temporallyStableOBB(previous_obb, new_obb);
        } else {
            // First detection or previous was invalid
            updated_obb = new_obb;
        }
        
        // Reset timing information
        updated_obb.last_update_time = std::chrono::steady_clock::now();
        updated_obb.frames_since_valid_detection = 0;
        updated_obb.confidence_at_creation = confidence_score;
        updated_obb.valid = true;
        
        std::cout << "OBB updated with new detection (confidence: " << confidence_score << ")" << std::endl;
    } else {
        // No valid detection this frame
        if (previous_obb.valid) {
            // Keep previous OBB but increment frame counter
            updated_obb = previous_obb;
            updated_obb.frames_since_valid_detection++;
            
            // Check if we should invalidate due to timeout
            if (shouldInvalidateOBB(updated_obb, 5)) { // 5 frames timeout
                std::cout << "OBB invalidated due to timeout (" 
                          << updated_obb.frames_since_valid_detection 
                          << " frames without detection)" << std::endl;
                updated_obb.valid = false;
                updated_obb.frames_since_valid_detection = 0;
            } else {
                std::cout << "Keeping previous OBB (" 
                          << updated_obb.frames_since_valid_detection 
                          << " frames without detection)" << std::endl;
            }
        } else {
            // No previous OBB and no new detection
            updated_obb.valid = false;
            updated_obb.frames_since_valid_detection = 0;
        }
    }
    
    return updated_obb;
}

OBBData computeValidatedOBBWithConfidence(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                         const pcl::ModelCoefficients& ground_coefficients,
                                         const OBBData& previous_obb,
                                         float confidence_score) {
    OBBData invalid_obb;
    invalid_obb.valid = false;
    
    if (!cloud || cloud->empty()) {
        std::cout << "Cannot compute OBB: empty cloud" << std::endl;
        return invalid_obb;
    }
    
    // Validate the cluster first
    ObjectValidationResult validation = validateObjectCluster(cloud);
    if (!validation.is_valid_object) {
        std::cout << "Rejecting OBB computation: " << validation.rejection_reason << std::endl;
        return invalid_obb;
    }
    
    std::cout << "Computing OBB for valid object (confidence: " 
              << validation.confidence_score << ")" << std::endl;
    
    // Compute the minimal OBB
    OBBData obb = findMinimumOBB(cloud);
    if (!obb.valid) {
        std::cout << "Failed to compute minimal OBB" << std::endl;
        return invalid_obb;
    }
    
    // Extend to ground
    obb = extendOBBToGround(obb, ground_coefficients);
    
    // Set timing and confidence information
    obb.last_update_time = std::chrono::steady_clock::now();
    obb.frames_since_valid_detection = 0;
    obb.confidence_at_creation = confidence_score;
    
    return obb;
}

OBBData computeValidatedOBB(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                            const pcl::ModelCoefficients& ground_coefficients,
                            const OBBData& previous_obb) {
    return computeValidatedOBBWithConfidence(cloud, ground_coefficients, previous_obb, 0.0f);
}

OBBData getInvalidOBB() {
    OBBData invalid_obb;
    invalid_obb.valid = false;
    invalid_obb.frames_since_valid_detection = 0;
    invalid_obb.confidence_at_creation = 0.0f;
    return invalid_obb;
}

OBBData findMinimumOBB(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    try {    
        OBBData obb;
        obb.valid = false;
        
        if (!cloud || cloud->empty()) {
            std::cerr << "[findMinimumOBB] Input cloud is empty!\n";
            return obb;
        }
        
        // First compute centroid
        Eigen::Vector4f centroid4;
        pcl::compute3DCentroid(*cloud, centroid4);
        Eigen::Vector3f centroid = centroid4.head<3>();
        
        // Use PCA to find the vertical axis
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, centroid4, covariance);
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covariance, Eigen::ComputeEigenvectors);
        if (eigenSolver.info() != Eigen::Success) {
            std::cerr << "[findMinimumOBB] Eigen decomposition failed!\n";
            return obb;
        }
        Eigen::Matrix3f eigenVectors = eigenSolver.eigenvectors();
        
        // Preserve the vertical axis from PCA (assuming column 0 is the most vertical)
        Eigen::Vector3f globalUp(0.0f, 1.0f, 0.0f);
        Eigen::Vector3f verticalAxis = eigenVectors.col(0);
        if (verticalAxis.dot(globalUp) < 0)
            verticalAxis = -verticalAxis;
        
        // Now we'll search for the minimum area by rotating around the vertical axis
        float minArea = std::numeric_limits<float>::max();
        float bestAngle = 0.0f;
        Eigen::Vector3f minPt_best, maxPt_best;
        
        // Search in 5-degree increments (can be adjusted for precision vs performance)
        const float angleStep = 5.0f * M_PI / 180.0f; // 5 degrees in radians
        
        for (float angle = 0.0f; angle < M_PI/2; angle += angleStep) {
            // Create rotation matrix around vertical axis
            Eigen::AngleAxisf rotation(angle, verticalAxis);
            Eigen::Matrix3f rotMatrix = rotation.toRotationMatrix();
            
            // Create a basis with vertical axis as Z and rotated X,Y
            Eigen::Matrix3f basis;
            Eigen::Vector3f xAxis = (Eigen::Matrix3f::Identity() - verticalAxis * verticalAxis.transpose()) * 
                                    Eigen::Vector3f::UnitX();
            xAxis.normalize();
            
            // Apply the rotation to the X axis
            xAxis = rotMatrix * xAxis;
            
            // Create Y axis orthogonal to both vertical and rotated X
            Eigen::Vector3f yAxis = verticalAxis.cross(xAxis).normalized();
            
            // Create basis matrix
            basis.col(0) = xAxis;
            basis.col(1) = yAxis;
            basis.col(2) = verticalAxis;
            
            // Find min/max in this basis
            Eigen::Vector3f minPt(std::numeric_limits<float>::infinity(),
                                std::numeric_limits<float>::infinity(),
                                std::numeric_limits<float>::infinity());
            Eigen::Vector3f maxPt(-std::numeric_limits<float>::infinity(),
                                -std::numeric_limits<float>::infinity(),
                                -std::numeric_limits<float>::infinity());
            
            for (const auto &pt : cloud->points) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                Eigen::Vector3f localP = basis.transpose() * (p - centroid);
                minPt = minPt.cwiseMin(localP);
                maxPt = maxPt.cwiseMax(localP);
            }
            
            // Calculate area of the base (XY plane)
            float width = maxPt.x() - minPt.x();
            float depth = maxPt.y() - minPt.y();
            float area = width * depth;
            
            if (area < minArea) {
                minArea = area;
                bestAngle = angle;
                minPt_best = minPt;
                maxPt_best = maxPt;
            }
        }
        
        // Now create the final basis with the best angle
        Eigen::AngleAxisf bestRotation(bestAngle, verticalAxis);
        Eigen::Matrix3f bestRotMatrix = bestRotation.toRotationMatrix();
        
        Eigen::Matrix3f R;
        Eigen::Vector3f xAxis = (Eigen::Matrix3f::Identity() - verticalAxis * verticalAxis.transpose()) * 
                                Eigen::Vector3f::UnitX();
        xAxis.normalize();
        xAxis = bestRotMatrix * xAxis;
        Eigen::Vector3f yAxis = verticalAxis.cross(xAxis).normalized();
        
        R.col(0) = xAxis;
        R.col(1) = yAxis;
        R.col(2) = verticalAxis;
        
        // Recompute min/max with best basis
        Eigen::Vector3f minPt(std::numeric_limits<float>::infinity(),
                            std::numeric_limits<float>::infinity(),
                            std::numeric_limits<float>::infinity());
        Eigen::Vector3f maxPt(-std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity());
        
        for (const auto &pt : cloud->points) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f localP = R.transpose() * (p - centroid);
            minPt = minPt.cwiseMin(localP);
            maxPt = maxPt.cwiseMax(localP);
        }
        
        Eigen::Vector3f extent = maxPt - minPt;
        
        // Check if any dimension exceeds the maximum allowed size
        const float MAX_DIMENSION = 2500.0f;
        if (extent.x() > MAX_DIMENSION || extent.y() > MAX_DIMENSION || extent.z() > MAX_DIMENSION) {
            std::cout << "Warning: Bounding box exceeds maximum allowed size of " << MAX_DIMENSION 
                      << "mm. Current dimensions: " << extent.x() << "x" << extent.y() 
                      << "x" << extent.z() << "mm. BB will not be displayed." << std::endl;
            obb.valid = false;
            return obb;
        }
        
        Eigen::Vector3f localCenter = minPt + extent * 0.5f;
        Eigen::Vector3f worldCenter = centroid + R * localCenter;
        
        obb.obb_pos.x = worldCenter.x();
        obb.obb_pos.y = worldCenter.y();
        obb.obb_pos.z = worldCenter.z();
        
        obb.obb_min.x = -extent.x() / 2.0f;
        obb.obb_min.y = -extent.y() / 2.0f;
        obb.obb_min.z = -extent.z() / 2.0f;
        obb.obb_max.x = extent.x() / 2.0f;
        obb.obb_max.y = extent.y() / 2.0f;
        obb.obb_max.z = extent.z() / 2.0f;
        
        obb.rot_matrix = R;
        obb.valid = true;
        
        std::cout << "Minimum OBB found with base area: " << minArea << " mm²" << std::endl;
        std::cout << "Best rotation angle: " << (bestAngle * 180.0f / M_PI) << " degrees" << std::endl;
        
        return obb;
    }
    catch (const std::exception& e) {
        std::cerr << "Error in findMinimumOBB: " << e.what() << std::endl;
    }
    return OBBData();
}

OBBData temporallyStableOBB(const OBBData &prev, const OBBData &current, float alpha) {
    if (!prev.valid) return current;
    if (!current.valid) return prev;
    
    OBBData smoothed;
    smoothed.valid = true;
    
    // Smooth position
    Eigen::Vector3f prev_center(prev.obb_pos.x, prev.obb_pos.y, prev.obb_pos.z);
    Eigen::Vector3f curr_center(current.obb_pos.x, current.obb_pos.y, current.obb_pos.z);
    Eigen::Vector3f center = alpha * curr_center + (1.0f - alpha) * prev_center;
    smoothed.obb_pos.x = center.x();
    smoothed.obb_pos.y = center.y();
    smoothed.obb_pos.z = center.z();
    
    // Smooth dimensions with hysteresis - only allow small increases but larger decreases
    float prev_dim_x = prev.obb_max.x - prev.obb_min.x;
    float prev_dim_y = prev.obb_max.y - prev.obb_min.y;
    float prev_dim_z = prev.obb_max.z - prev.obb_min.z;
    float curr_dim_x = current.obb_max.x - current.obb_min.x;
    float curr_dim_y = current.obb_max.y - current.obb_min.y;
    float curr_dim_z = current.obb_max.z - current.obb_min.z;
    
    // Asymmetric smoothing factors
    float alpha_grow = 0.1f;    // Slow to grow
    float alpha_shrink = 0.3f;  // Faster to shrink
    
    float dim_x, dim_y, dim_z;
    
    // Apply asymmetric smoothing
    dim_x = (curr_dim_x > prev_dim_x) ? 
        alpha_grow * curr_dim_x + (1.0f - alpha_grow) * prev_dim_x :
        alpha_shrink * curr_dim_x + (1.0f - alpha_shrink) * prev_dim_x;
        
    dim_y = (curr_dim_y > prev_dim_y) ? 
        alpha_grow * curr_dim_y + (1.0f - alpha_grow) * prev_dim_y :
        alpha_shrink * curr_dim_y + (1.0f - alpha_shrink) * prev_dim_y;
        
    dim_z = (curr_dim_z > prev_dim_z) ? 
        alpha_grow * curr_dim_z + (1.0f - alpha_grow) * prev_dim_z :
        alpha_shrink * curr_dim_z + (1.0f - alpha_shrink) * prev_dim_z;
    
    smoothed.obb_min.x = -dim_x / 2.0f;
    smoothed.obb_min.y = -dim_y / 2.0f;
    smoothed.obb_min.z = -dim_z / 2.0f;
    smoothed.obb_max.x = dim_x / 2.0f;
    smoothed.obb_max.y = dim_y / 2.0f;
    smoothed.obb_max.z = dim_z / 2.0f;
    
    // Smooth orientation with orientation correction
    // First check if the orientations are similar
    Eigen::Quaternionf prev_quat(prev.rot_matrix);
    Eigen::Quaternionf curr_quat(current.rot_matrix);
    
    // Ensure we're interpolating along the shortest path
    if (prev_quat.dot(curr_quat) < 0)
        curr_quat.coeffs() = -curr_quat.coeffs();
    
    // Get the angle between the quaternions
    float angle = std::acos(std::min(1.0f, std::abs(prev_quat.dot(curr_quat))));
    
    // If the rotation is too large (> 45 degrees), reduce its influence
    float rotation_alpha = alpha;
    if (angle > M_PI/4) {
        rotation_alpha *= 0.5f; // Reduce the effect of large rotations
    }
    
    Eigen::Quaternionf smooth_quat = prev_quat.slerp(rotation_alpha, curr_quat);
    smooth_quat.normalize(); // Ensure it's a valid rotation
    smoothed.rot_matrix = smooth_quat.toRotationMatrix();
    
    return smoothed;
}

OBBData smoothOBB(const OBBData &prev, const OBBData &current, float alpha) {
    OBBData smoothed;
    smoothed.valid = true;
    // Smooth center position.
    Eigen::Vector3f prev_center(prev.obb_pos.x, prev.obb_pos.y, prev.obb_pos.z);
    Eigen::Vector3f curr_center(current.obb_pos.x, current.obb_pos.y, current.obb_pos.z);
    Eigen::Vector3f center = alpha * curr_center + (1.0f - alpha) * prev_center;
    smoothed.obb_pos.x = center.x();
    smoothed.obb_pos.y = center.y();
    smoothed.obb_pos.z = center.z();

    // Smooth dimensions.
    float prev_dim_x = prev.obb_max.x - prev.obb_min.x;
    float prev_dim_y = prev.obb_max.y - prev.obb_min.y;
    float prev_dim_z = prev.obb_max.z - prev.obb_min.z;
    float curr_dim_x = current.obb_max.x - current.obb_min.x;
    float curr_dim_y = current.obb_max.y - current.obb_min.y;
    float curr_dim_z = current.obb_max.z - current.obb_min.z;
    float dim_x = alpha * curr_dim_x + (1.0f - alpha) * prev_dim_x;
    float dim_y = alpha * curr_dim_y + (1.0f - alpha) * prev_dim_y;
    float dim_z = alpha * curr_dim_z + (1.0f - alpha) * prev_dim_z;
    smoothed.obb_min.x = -dim_x / 2.0f;
    smoothed.obb_min.y = -dim_y / 2.0f;
    smoothed.obb_min.z = -dim_z / 2.0f;
    smoothed.obb_max.x = dim_x / 2.0f;
    smoothed.obb_max.y = dim_y / 2.0f;
    smoothed.obb_max.z = dim_z / 2.0f;

    // Smooth orientation using quaternion slerp.
    Eigen::Quaternionf prev_quat(prev.rot_matrix);
    Eigen::Quaternionf curr_quat(current.rot_matrix);
    Eigen::Quaternionf smooth_quat = prev_quat.slerp(alpha, curr_quat);
    smoothed.rot_matrix = smooth_quat.toRotationMatrix();

    return smoothed;
}

OBBData extendOBBToGround(const OBBData& obb, 
    const pcl::ModelCoefficients& ground_coefficients) {
    if (!obb.valid) return obb;

    OBBData extended_obb = obb;

    // Get ground plane equation: ax + by + cz + d = 0
    float a = ground_coefficients.values[0];
    float b = ground_coefficients.values[1];
    float c = ground_coefficients.values[2];
    float d = ground_coefficients.values[3];

    // Normalize the plane equation
    float norm = std::sqrt(a*a + b*b + c*c);
    a /= norm; b /= norm; c /= norm; d /= norm;

    // Calculate OBB vertices manually (since calculateOBBVertices might not be available)
    std::vector<Eigen::Vector3f> vertices(8);
    const float hx = (obb.obb_max.x - obb.obb_min.x) * 0.5f;
    const float hy = (obb.obb_max.y - obb.obb_min.y) * 0.5f;
    const float hz = (obb.obb_max.z - obb.obb_min.z) * 0.5f;
    Eigen::Vector3f center(obb.obb_pos.x, obb.obb_pos.y, obb.obb_pos.z);

    // Top vertices (indices 0-3)
    vertices[0] = center + obb.rot_matrix * Eigen::Vector3f(-hx, -hy, +hz);
    vertices[1] = center + obb.rot_matrix * Eigen::Vector3f(+hx, -hy, +hz);
    vertices[2] = center + obb.rot_matrix * Eigen::Vector3f(+hx, +hy, +hz);
    vertices[3] = center + obb.rot_matrix * Eigen::Vector3f(-hx, +hy, +hz);
    
    // Bottom vertices (indices 4-7)
    vertices[4] = center + obb.rot_matrix * Eigen::Vector3f(-hx, -hy, -hz);
    vertices[5] = center + obb.rot_matrix * Eigen::Vector3f(+hx, -hy, -hz);
    vertices[6] = center + obb.rot_matrix * Eigen::Vector3f(+hx, +hy, -hz);
    vertices[7] = center + obb.rot_matrix * Eigen::Vector3f(-hx, +hy, -hz);

    // Get the 4 bottom vertices of the current OBB
    std::vector<Eigen::Vector3f> bottom_vertices;
    for (int i = 4; i < 8; i++) {
        bottom_vertices.push_back(vertices[i]);
    }

    // Extend each bottom vertex by 10cm in X and Y directions only
    std::vector<Eigen::Vector3f> extended_corners;
    const float extension = 100.0f; // 10cm extension

    // For each bottom vertex, create extended points in the OBB's local X and Y directions
    Eigen::Vector3f x_axis = obb.rot_matrix.col(0); // Local X axis
    Eigen::Vector3f y_axis = obb.rot_matrix.col(1); // Local Y axis

    for (const auto& corner : bottom_vertices) {
        // Create extended points in 4 diagonal directions using OBB's local axes
        extended_corners.push_back(corner + extension * x_axis + extension * y_axis);
        extended_corners.push_back(corner - extension * x_axis + extension * y_axis);
        extended_corners.push_back(corner + extension * x_axis - extension * y_axis);
        extended_corners.push_back(corner - extension * x_axis - extension * y_axis);
        
        // Also include the original corner
        extended_corners.push_back(corner);
    }

    // Calculate the Z value on the ground plane for each extended point
    float min_ground_z = std::numeric_limits<float>::max();

    for (const auto& point : extended_corners) {
        // For a point (x,y,?), solve for z in the plane equation ax + by + cz + d = 0
        // => z = -(ax + by + d) / c
        if (std::abs(c) > 1e-6) { // Make sure we don't divide by zero
            float ground_z = -(a * point.x() + b * point.y() + d) / c;
            min_ground_z = std::min(min_ground_z, ground_z);
        }
    }

    // Find current lowest point of the OBB
    float current_lowest_z = std::numeric_limits<float>::max();
    for (const auto& vertex : bottom_vertices) {
        current_lowest_z = std::min(current_lowest_z, vertex.z());
    }

    // Check if extension is needed
    const float ground_threshold = 7.5f; // 5mm tolerance
    if (std::abs(current_lowest_z - min_ground_z) <= ground_threshold) {
        std::cout << "Object already at ground level (within tolerance)" << std::endl;
        return obb;
    }

    // Calculate extension needed
    float height_extension = current_lowest_z - min_ground_z;

    // Only extend if ground is below the object
    if (height_extension > 0) {
        std::cout << "Current lowest z: " << current_lowest_z << ", Ground z: " << min_ground_z << std::endl;

        // Extend the OBB downward in its local coordinate system
        // Adjust the min Z coordinate (extend downward)
        extended_obb.obb_min.z -= height_extension;

        // Adjust the center position (move down by half the extension)
        Eigen::Vector3f z_axis = obb.rot_matrix.col(2);
        Eigen::Vector3f center_adjustment = z_axis * (-height_extension / 2.0f);
        extended_obb.obb_pos.x += center_adjustment.x();
        extended_obb.obb_pos.y += center_adjustment.y();
        extended_obb.obb_pos.z += center_adjustment.z();

        std::cout << "Extended OBB height by " << height_extension << "mm" << std::endl;
        std::cout << "New OBB height: " << (extended_obb.obb_max.z - extended_obb.obb_min.z) << "mm" << std::endl;
    } else {
        std::cout << "Ground is above object bottom, no extension needed" << std::endl;
    }

    return extended_obb;
}