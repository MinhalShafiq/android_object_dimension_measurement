#include "point_cloud_processing.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <algorithm>
#include <numeric>

pcl::PointCloud<pcl::PointXYZ>::Ptr removeShadowTail(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    
    if (input_cloud->empty() || input_cloud->size() < 10) {
        return input_cloud;
    }
    
    std::cout << "Starting height-preserving shadow tail removal, input size: " << input_cloud->size() << std::endl;
    
    // Step 1: Find the bounding box to determine height range
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*input_cloud, min_pt, max_pt);
    float min_height = min_pt[2]; // Z-coordinate represents height
    float max_height = max_pt[2];
    float height_range = max_height - min_height;
    
    std::cout << "Object height: " << height_range << "mm, from " << min_height << " to " << max_height << std::endl;
    
    // Step 2: Compute the centroid for reference
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*input_cloud, centroid);
    
    // Step 3: Apply a modified outlier removal approach that preserves points at height extremes
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    // We'll use statistical outlier removal but protect top and bottom points
    std::vector<int> indices_to_keep;
    const float height_protect_margin = height_range * 0.1f; // Protect 10% at top and bottom
    
    // Step 3a: Identify height-critical points to protect
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        const auto& pt = input_cloud->points[i];
        // Protect points near the top or bottom of the object
        if (pt.z > max_height - height_protect_margin || pt.z < min_height + height_protect_margin) {
            indices_to_keep.push_back(i);
        }
    }
    
    std::cout << "Protected " << indices_to_keep.size() << " height-critical points" << std::endl;
    
    // Step 3b: Apply statistical outlier removal to non-protected points
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr protected_indices(new pcl::PointIndices);
    protected_indices->indices = indices_to_keep;
    
    // Extract non-protected points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(protected_indices);
    extract.setNegative(true); // Get points NOT in indices_to_keep
    extract.filter(*temp_cloud);
    
    // Apply statistical outlier removal to non-protected points
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(temp_cloud);
    sor.setMeanK(20);
    sor.setStddevMulThresh(1.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_non_protected(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered_non_protected);
    
    // Step 3c: Combine protected points with filtered non-protected points
    pcl::PointCloud<pcl::PointXYZ>::Ptr protected_points(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(false); // Get points IN indices_to_keep
    extract.filter(*protected_points);
    
    *filtered = *filtered_non_protected;
    *filtered += *protected_points;
    
    std::cout << "After protected statistical filtering: " << filtered->size() << " points" << std::endl;
    
    // Step 4: Apply a modified radius outlier removal that's gentler on extremities
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Instead of using PCL's radius outlier removal, we'll implement our own
    // that varies the required neighbor count based on position
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(filtered);
    
    radius_filtered->reserve(filtered->size());
    
    const float radius_search = 15.0f; // Base search radius
    const int base_min_neighbors = 5;  // Base minimum neighbor requirement
    
    for (size_t i = 0; i < filtered->size(); ++i) {
        const auto& pt = filtered->points[i];
        
        // Calculate distance from centroid (horizontal plane only)
        float dx = pt.x - centroid[0];
        float dy = pt.y - centroid[1];
        float dist_from_center = std::sqrt(dx*dx + dy*dy);
        
        // Calculate relative height position
        float rel_height = (pt.z - min_height) / height_range;
        
        // Adjust neighbor requirements based on position
        int required_neighbors = base_min_neighbors;
        
        // Points at extremities need fewer neighbors to be kept
        if (rel_height < 0.1f || rel_height > 0.9f) {
            required_neighbors = std::max(2, base_min_neighbors - 3);
        }
        // Points at the edges need fewer neighbors too
        if (dist_from_center > (max_pt[0] - min_pt[0]) * 0.4f) {
            required_neighbors = std::max(2, required_neighbors - 1);
        }
        
        // Search for neighbors
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_distances;
        int found_neighbors = tree->radiusSearch(pt, radius_search, neighbor_indices, neighbor_distances);
        
        // Keep point if it has enough neighbors
        if (found_neighbors >= required_neighbors) {
            radius_filtered->push_back(pt);
        }
    }
    
    radius_filtered->width = radius_filtered->size();
    radius_filtered->height = 1;
    radius_filtered->is_dense = false;
    
    std::cout << "After adaptive radius filtering: " << radius_filtered->size() << " points" << std::endl;
    
    // Step 5: Now use a modified shadow detection approach
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(radius_filtered);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(20.0); // Smaller radius than before
    ne.compute(*normals);
    
    // Custom shadow detection that's more conservative
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    final_cloud->reserve(radius_filtered->size());
    
    // Vector pointing upward
    Eigen::Vector3f view_direction(0.0f, 0.0f, 1.0f);
    
    // Shadow detection parameters
    const float normal_threshold = 0.3f; // More permissive than before (was 0.1f)
    
    for (size_t i = 0; i < radius_filtered->size(); ++i) {
        const auto& pt = radius_filtered->points[i];
        const auto& normal = normals->points[i];
        
        // Calculate dot product between normal and view direction
        Eigen::Vector3f n_vec(normal.normal_x, normal.normal_y, normal.normal_z);
        float dp = std::abs(n_vec.dot(view_direction));
        
        // Calculate relative height position
        float rel_height = (pt.z - min_height) / height_range;
        
        // Points at height extremes are kept regardless of normal
        if (rel_height < 0.1f || rel_height > 0.9f) {
            final_cloud->push_back(pt);
        } 
        // Otherwise, use normal-based shadow detection but with more permissive threshold
        else if (dp > normal_threshold) {
            final_cloud->push_back(pt);
        }
    }
    
    final_cloud->width = final_cloud->size();
    final_cloud->height = 1;
    final_cloud->is_dense = false;
    
    // Calculate percentage of points removed in this step
    float shadow_removal_percent = 100.0f * (radius_filtered->size() - final_cloud->size()) / 
                                  static_cast<float>(radius_filtered->size());
    std::cout << "After conservative shadow filtering: " << final_cloud->size() << " points ("
              << shadow_removal_percent << "% shadow points removed)" << std::endl;
    
    // Report total reduction
    float total_percent = 100.0f * (input_cloud->size() - final_cloud->size()) / 
                          static_cast<float>(input_cloud->size());
    std::cout << "Total reduction: " << total_percent << "% (" 
              << input_cloud->size() << " -> " << final_cloud->size() << ")" << std::endl;
    
    // Step 6: Verify we've preserved the height range
    pcl::getMinMax3D(*final_cloud, min_pt, max_pt);
    float new_min_height = min_pt[2];
    float new_max_height = max_pt[2];
    float new_height_range = new_max_height - new_min_height;
    
    std::cout << "Preserved height: " << new_height_range << "mm, from " << new_min_height << " to " << new_max_height << std::endl;
    std::cout << "Height difference: " << (height_range - new_height_range) << "mm (" 
              << (100.0f * (height_range - new_height_range) / height_range) << "%)" << std::endl;
    
    return final_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeRemainingOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty() || cloud->size() < 10) {
        return cloud;
    }
    
    std::cout << "Removing remaining outliers, input size: " << cloud->size() << std::endl;
    
    // Step 1: Use statistical outlier removal with conservative parameters
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(30);  // Larger neighborhood
        sor.setStddevMulThresh(2.0);  // More permissive threshold
        sor.filter(*sor_filtered);
    }
    
    // Step 2: Apply radius outlier removal with adaptive radius
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    {
        // Find an appropriate radius based on point density
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(sor_filtered);
        
        // Compute centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*sor_filtered, centroid);
        
        // Sample some points to estimate average distance to neighbors
        const int num_samples = std::min(50, static_cast<int>(sor_filtered->size()));
        float avg_distance = 0.0f;
        int valid_samples = 0;
        
        for (int i = 0; i < num_samples; i++) {
            int idx = (i * sor_filtered->size()) / num_samples;
            std::vector<int> indices;
            std::vector<float> distances;
            if (tree->nearestKSearch(sor_filtered->points[idx], 6, indices, distances) > 1) {
                // Use the average distance to the 5 nearest neighbors
                float sum_dist = 0.0f;
                for (size_t j = 1; j < indices.size(); j++) { // Start from 1 to skip self
                    sum_dist += std::sqrt(distances[j]);
                }
                avg_distance += sum_dist / (indices.size() - 1);
                valid_samples++;
            }
        }
        
        if (valid_samples > 0) {
            avg_distance /= valid_samples;
        } else {
            avg_distance = 30.0f; // Fallback if sampling fails
        }
        
        // Use 3x the average distance as the radius
        float radius = avg_distance * 3.0f;
        std::cout << "Adaptive radius for outlier removal: " << radius << "mm" << std::endl;
        
        // Apply radius outlier removal
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(sor_filtered);
        ror.setRadiusSearch(radius);
        ror.setMinNeighborsInRadius(5);
        ror.filter(*final_filtered);
    }
    
    std::cout << "After statistical and radius filtering: " << final_filtered->size() 
              << " points (removed " << (cloud->size() - final_filtered->size()) 
              << " outliers)" << std::endl;
    
    return final_filtered;
}