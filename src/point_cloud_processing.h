#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Point cloud filtering and processing functions
pcl::PointCloud<pcl::PointXYZ>::Ptr removeShadowTail(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud
);

pcl::PointCloud<pcl::PointXYZ>::Ptr removeRemainingOutliers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
);