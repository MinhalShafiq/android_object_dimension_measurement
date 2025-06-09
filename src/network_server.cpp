#include "network_server.h"
#include "point_cloud_processing.h"
#include "obb_calculator.h"
#include "DBSCAN_kdtree.h"
#include <iostream>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>

void processClient(int client_fd, SharedData& shared) {
    try {
        // Processing parameters.
        const float voxel_size        = 12.0f; 
        const int mean_k              = 20;
        const float stddev_thresh     = 1.0f;
        const float distance_thresh   = 25.0f;  
        const float cluster_tolerance = 20.0f;  
        const int min_cluster_size    = 500;
        const int max_cluster_size    = 307200;
        const float ground_margin     = -2.0f;   

        while (true) {
            try {
                uint32_t numFloats;
                ssize_t ret = recv(client_fd, &numFloats, sizeof(numFloats), MSG_WAITALL);
                if (ret != sizeof(numFloats)) break;
                numFloats = ntohl(numFloats);
                if (numFloats % 3 != 0 || numFloats == 0) break;

                std::vector<float> buffer(numFloats);
                size_t bytesToRead = numFloats * sizeof(float);
                size_t totalRead = 0;
                while (totalRead < bytesToRead) {
                    ssize_t received = recv(client_fd,
                                            reinterpret_cast<char*>(buffer.data()) + totalRead,
                                            bytesToRead - totalRead,
                                            MSG_WAITALL);
                    if (received <= 0) {
                        std::cerr << "Socket error or connection closed while reading point cloud data." << std::endl;
                        break;
                    }
                    totalRead += received;
                }
                if (totalRead != bytesToRead) {
                    std::cerr << "Incomplete data received; skipping this frame." << std::endl;
                    continue; 
                }

                // Build the input cloud.
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                cloud->width = numFloats / 3;
                cloud->height = 1;
                cloud->is_dense = false;
                cloud->points.resize(cloud->width);
                for (size_t i = 0; i < cloud->size(); ++i) {
                    cloud->points[i].x = buffer[i*3 + 0];
                    cloud->points[i].y = buffer[i*3 + 1];
                    cloud->points[i].z = buffer[i*3 + 2];
                }

                // Remove NaNs.
                pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                std::vector<int> indices;
                pcl::removeNaNFromPointCloud(*cloud, *cleaned_cloud, indices);
                if (cleaned_cloud->empty()) continue;

                // Downsample using a voxel grid.
                pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
                {
                    pcl::VoxelGrid<pcl::PointXYZ> voxel;
                    voxel.setInputCloud(cleaned_cloud);
                    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
                    voxel.filter(*downsampled);
                }

                // Statistical outlier removal.
                pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                {
                    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                    sor.setInputCloud(downsampled);
                    sor.setMeanK(mean_k);
                    sor.setStddevMulThresh(stddev_thresh);
                    sor.filter(*sor_cloud);
                }

                // Plane segmentation.
                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setMaxIterations(1000);
                seg.setDistanceThreshold(distance_thresh);
                seg.setInputCloud(sor_cloud);

                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                pcl::ModelCoefficients coefficients;
                seg.segment(*inliers, coefficients);
                if (inliers->indices.empty()) continue;
                if (coefficients.values[2] > 0) {
                    for (auto &v : coefficients.values)
                        v = -v;
                }

                // Extract objects above the plane.
                pcl::PointCloud<pcl::PointXYZ>::Ptr objects_above(new pcl::PointCloud<pcl::PointXYZ>);
                {
                    pcl::ExtractIndices<pcl::PointXYZ> extract;
                    extract.setInputCloud(sor_cloud);
                    extract.setIndices(inliers);
                    extract.setNegative(true);

                    pcl::PointCloud<pcl::PointXYZ> all_objs;
                    extract.filter(all_objs);
                    for (const auto& pt : all_objs) {
                        float dist_plane = coefficients.values[0] * pt.x +
                                           coefficients.values[1] * pt.y +
                                           coefficients.values[2] * pt.z +
                                           coefficients.values[3];
                        if (dist_plane > ground_margin)
                            objects_above->push_back(pt);
                    }
                    objects_above->width = objects_above->size();
                    objects_above->height = 1;
                    objects_above->is_dense = true;
                }
                if (objects_above->empty()) continue;
                
                // Build a KD-tree for clustering.
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(objects_above);

                // DBSCAN clustering using the KD-tree.
                DBSCANKdtreeCluster<pcl::PointXYZ> ec;
                ec.setCorePointMinPts(10);
                ec.setClusterTolerance(cluster_tolerance);
                ec.setMinClusterSize(min_cluster_size);
                ec.setMaxClusterSize(max_cluster_size);
                ec.setSearchMethod(tree);
                ec.setInputCloud(objects_above);

                std::vector<pcl::PointIndices> cluster_indices;
                ec.extract(cluster_indices);
                if (cluster_indices.empty()) continue;

                // For each cluster, compute the 2D centroid (x, y) using all points.
                uint32_t bestLabel = 0;
                double minDistance2 = std::numeric_limits<double>::max();
                std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_clouds;
                for (size_t i = 0; i < cluster_indices.size(); ++i) {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::copyPointCloud(*objects_above, cluster_indices[i].indices, *cluster_rgb);
                    cluster_clouds.push_back(cluster_rgb);

                    double x_sum = 0.0, y_sum = 0.0;
                    for (const auto &pt : cluster_rgb->points) {
                        x_sum += pt.x;
                        y_sum += pt.y;
                    }
                    double count = static_cast<double>(cluster_rgb->size());
                    double centroidX = x_sum / count;
                    double centroidY = y_sum / count;
                    double dist2 = centroidX * centroidX + centroidY * centroidY;
                    if (dist2 < minDistance2) {
                        minDistance2 = dist2;
                        bestLabel = i;
                    }
                }
                std::cout << "Best cluster label (closest to origin): " << bestLabel << std::endl;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr best_cluster = cluster_clouds[bestLabel];

                // Color the best cluster purple.
                for (auto &pt : *best_cluster) {
                    pt.r = 128;
                    pt.g = 0;
                    pt.b = 128;
                }

                // Color the entire sor_cloud white for visualization.
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::copyPointCloud(*sor_cloud, *sor_cloud_colored);
                for (auto &p : *sor_cloud_colored) {
                    p.r = 255;
                    p.g = 255;
                    p.b = 255;
                }
                
                // Apply shadow removal to the best cluster
                pcl::PointCloud<pcl::PointXYZ>::Ptr best_xyz(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*best_cluster, *best_xyz);
                
                // Call our optimized shadow removal function
                std::cout << "Removing shadow points from the best cluster..." << std::endl;
                size_t original_size = best_xyz->size();
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_xyz = removeShadowTail(best_xyz);
                filtered_xyz = removeRemainingOutliers(filtered_xyz);
                std::cout << "Original cluster size: " << original_size 
                          << ", After shadow removal: " << filtered_xyz->size() 
                          << " (" << (original_size - filtered_xyz->size()) << " shadow points removed)" << std::endl;
                
                // Create an RGB version of the filtered cloud
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_best_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::copyPointCloud(*filtered_xyz, *filtered_best_cluster);
                
                // Color the filtered cluster purple
                for (auto &pt : *filtered_best_cluster) {
                    pt.r = 128;
                    pt.g = 0;
                    pt.b = 128;
                }
                
                // Replace the best_cluster with the filtered version
                best_cluster = filtered_best_cluster;

                // Compute the Oriented Bounding Box (OBB) for the filtered best cluster.
                OBBData hull_obb = findMinimumOBB(filtered_xyz);
                if (hull_obb.valid) {
                    std::cout << "\nHull-based minimal OBB computed after shadow point removal!\n";
                    hull_obb = extendOBBToGround(hull_obb, coefficients);
                }

                // Update shared data (with proper locking).
                {
                    std::vector<CloudT::Ptr> finals;
                    finals.push_back(sor_cloud_colored);
                    finals.push_back(best_cluster);
                    std::lock_guard<std::mutex> lock(shared.data_mutex);
                    // If a previous OBB exists, apply our enhanced temporal stability
                    if (shared.obb.valid && hull_obb.valid) {
                        hull_obb = temporallyStableOBB(shared.obb, hull_obb);
                    }
                    shared.clusters = finals;
                    shared.obb = hull_obb;
                    shared.new_data_available = true;
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Error in processClient loop: " << e.what() << std::endl;
            }
            catch (...) {
                std::cerr << "Unknown error in processClient loop." << std::endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Client error: " << e.what() << std::endl;
    }
    close(client_fd);
}

void videoServer(SharedData& shared) {
    try {
        const int PORT = 9081;
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            std::cerr << "Failed to create video server socket" << std::endl;
            return;
        }
        int opt = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "setsockopt(SO_REUSEADDR) failed for video server" << std::endl;
        }
#ifdef SO_REUSEPORT
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
            std::cerr << "setsockopt(SO_REUSEPORT) failed for video server" << std::endl;
        }
#endif

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(PORT);

        if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Video server bind failed" << std::endl;
            close(server_fd);
            return;
        }

        if (listen(server_fd, 5) < 0) {
            std::cerr << "Video server listen failed" << std::endl;
            close(server_fd);
            return;
        }

        std::cout << "Video server listening on port " << PORT << std::endl;

        while (true) {
            sockaddr_in client_addr{};
            socklen_t len = sizeof(client_addr);
            int client_fd = accept(server_fd, (sockaddr*)&client_addr, &len);
            if (client_fd < 0) {
                std::cerr << "Video server accept failed" << std::endl;
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(shared.video_mutex);
                if (shared.video_client_fd >= 0) {
                    std::cout << "Closing existing video client connection." << std::endl;
                    close(shared.video_client_fd);
                }
                shared.video_client_fd = client_fd;
            }
            std::cout << "New video client connected\n";
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Video server thread error: " << e.what() << std::endl;
    }
}

void pointCloudServer(SharedData& shared) {
    try {
        const int PORT = 9080;
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            std::cerr << "Failed to create point cloud server socket" << std::endl;
            return;
        }
        int opt = 1;
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "setsockopt(SO_REUSEADDR) failed for point cloud server" << std::endl;
        }
#ifdef SO_REUSEPORT
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) < 0) {
            std::cerr << "setsockopt(SO_REUSEPORT) failed for point cloud server" << std::endl;
        }
#endif

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(PORT);

        if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Point cloud server bind failed" << std::endl;
            close(server_fd);
            return;
        }

        if (listen(server_fd, 5) < 0) {
            std::cerr << "Point cloud server listen failed" << std::endl;
            close(server_fd);
            return;
        }

        std::cout << "Point cloud server listening on port " << PORT << std::endl;

        while (true) {
            sockaddr_in client_addr{};
            socklen_t len = sizeof(client_addr);
            int client_fd = accept(server_fd, (sockaddr*)&client_addr, &len);
            if (client_fd < 0) {
                std::cerr << "Failed to accept point cloud client connection" << std::endl;
                continue;
            }
            
            // duplicate point cloud connection.
            {
                std::lock_guard<std::mutex> lock(shared.pointCloud_mutex);
                if (shared.pointCloud_client_fd >= 0) {
                    std::cerr << "Point cloud client already connected. Rejecting new connection." << std::endl;
                    close(client_fd);
                    continue;
                }
                shared.pointCloud_client_fd = client_fd;
            }
            
            std::thread([client_fd, &shared]() {
                processClient(client_fd, shared);
                // Reset the point cloud connection flag after the client disconnects.
                {
                    std::lock_guard<std::mutex> lock(shared.pointCloud_mutex);
                    shared.pointCloud_client_fd = -1;
                }
            }).detach();
            std::cout << "New point cloud client connected\n";
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Point Cloud server thread error: " << e.what() << std::endl;
    }
}