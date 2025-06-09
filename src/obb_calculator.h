#pragma once

#include "structures.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

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