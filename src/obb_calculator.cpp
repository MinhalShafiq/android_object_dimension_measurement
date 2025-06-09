#include "obb_calculator.h"
#include "geometry_utils.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <iostream>
#include <limits>

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

    // Get the 4 bottom vertices of the current OBB
    std::vector<Eigen::Vector3f> vertices = calculateOBBVertices(obb);
    std::vector<Eigen::Vector3f> bottom_vertices;
    for (int i = 4; i < 8; i++) {
    bottom_vertices.push_back(vertices[i]);
    }

    // Extend each bottom vertex by 2cm in X and Y directions only
    std::vector<Eigen::Vector3f> extended_corners;
    const float extension = 100.0f; // 2cm

    // For each bottom vertex
    for (const auto& corner : bottom_vertices) {
    // Create extended points in 4 diagonal directions
    extended_corners.push_back(corner + Eigen::Vector3f(extension, extension, 0));
    extended_corners.push_back(corner + Eigen::Vector3f(-extension, extension, 0));
    extended_corners.push_back(corner + Eigen::Vector3f(extension, -extension, 0));
    extended_corners.push_back(corner + Eigen::Vector3f(-extension, -extension, 0));
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
    const float ground_threshold = 5.0f; // 5mm tolerance
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
    float current_height = obb.obb_max.z - obb.obb_min.z;

    // Adjust the min Z coordinate (extend downward)
    extended_obb.obb_min.z -= height_extension;

    // Adjust the center position (move down by half the extension)
    Eigen::Vector3f z_axis = obb.rot_matrix.col(2);
    Eigen::Vector3f center_adjustment = z_axis * (-height_extension / 2.0f);
    extended_obb.obb_pos.x += center_adjustment.x();
    extended_obb.obb_pos.y += center_adjustment.y();
    extended_obb.obb_pos.z += center_adjustment.z();

    float new_height = current_height + height_extension;
    std::cout << "Extended OBB height by " << height_extension << "mm" << std::endl;
    std::cout << "New OBB height: " << new_height << "mm" << std::endl;
    } else {
    std::cout << "Ground is above object bottom, no extension needed" << std::endl;
    }

    return extended_obb;
}