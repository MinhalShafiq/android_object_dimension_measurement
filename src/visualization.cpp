#include "visualization.h"
#include "geometry_utils.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

void visualizePointCloud(SharedData& shared) {
    try {
        pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
        viewer.setBackgroundColor(0, 0, 0);
        viewer.loadCameraParameters("CameraParams.yaml"); 
        viewer.setShowFPS(false);
        viewer.setSize(720,720);

        // Override default keyboard callback to swallow key events and avoid runtime crashes.
        viewer.registerKeyboardCallback([](const pcl::visualization::KeyboardEvent &event) {
            return;
        });

        std::vector<std::string> cluster_ids;
        std::vector<std::string> shape_ids;

        while (true) {
            try {
                // Local variables to store copied data (no shared access during processing)
                bool has_new_data = false;
                std::vector<CloudT::Ptr> local_clusters;
                OBBData local_obb;
                
                // ATOMIC READ of shared data - very brief lock
                {
                    std::lock_guard<std::mutex> lock(shared.data_mutex);
                    if (shared.new_data_available) {
                        local_clusters = shared.clusters;  // Copy the data
                        local_obb = shared.obb;           // Copy the OBB
                        shared.new_data_available = false;
                        has_new_data = true;
                    }
                } // Mutex released immediately
                
                // Process visualization data WITHOUT any mutex locks
                if (has_new_data) {
                    // Remove existing clusters
                    for (const auto& id : cluster_ids) {
                        if (viewer.contains(id)) {
                            viewer.removePointCloud(id);
                        }
                    }
                    cluster_ids.clear();

                    // Remove existing shapes
                    for (const auto& id : shape_ids) {
                        if (viewer.contains(id)) {
                            viewer.removeShape(id);
                        }
                    }
                    shape_ids.clear();

                    // Add new clusters
                    for (size_t i = 0; i < local_clusters.size(); ++i) {
                        std::string id = "cluster_" + std::to_string(i);
                        viewer.addPointCloud(local_clusters[i], id);
                        viewer.setPointCloudRenderingProperties(
                            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
                        cluster_ids.push_back(id);
                    }

                    // Add OBB visualization if valid
                    if (local_obb.valid) {
                        float obb_size_x = local_obb.obb_max.x - local_obb.obb_min.x;
                        float obb_size_y = local_obb.obb_max.y - local_obb.obb_min.y;
                        float obb_size_z = local_obb.obb_max.z - local_obb.obb_min.z;

                        Eigen::Vector3f obb_center(
                            local_obb.obb_pos.x,
                            local_obb.obb_pos.y,
                            local_obb.obb_pos.z
                        );
                        Eigen::Quaternionf obb_quat(local_obb.rot_matrix);

                        std::string obb_id = "OBB_cube";
                        viewer.addCube(obb_center, obb_quat,
                                    obb_size_x, obb_size_y, obb_size_z,
                                    obb_id);

                        viewer.setShapeRenderingProperties(
                            pcl::visualization::PCL_VISUALIZER_COLOR,
                            0.0, 1.0, 0.0,
                            obb_id
                        );
                        viewer.setShapeRenderingProperties(
                            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                            obb_id
                        );
                        viewer.setShapeRenderingProperties(
                            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                            3,
                            obb_id
                        );
                        shape_ids.push_back(obb_id);

                        // Add vertex spheres
                        std::vector<Eigen::Vector3f> obb_vertices = calculateOBBVertices(local_obb);
                        for (size_t i = 0; i < obb_vertices.size(); ++i) {
                            std::string vertex_id = "vertex_" + std::to_string(i);
                            viewer.addSphere(pcl::PointXYZ(obb_vertices[i].x(), obb_vertices[i].y(), obb_vertices[i].z()),
                                            5.0,
                                            1.0, 1.0, 0.0,
                                            vertex_id);
                            shape_ids.push_back(vertex_id);
                        }

                        // Console output
                        float obb_size_x_cm = obb_size_x / 10.0f;
                        float obb_size_y_cm = obb_size_y / 10.0f;
                        float obb_size_z_cm = obb_size_z / 10.0f;

                        std::cout << "\n--- Minimal OBB (Hull-based) ---\n";
                        std::cout << "OBB dims: " 
                                << obb_size_x_cm << " x "
                                << obb_size_y_cm << " x "
                                << obb_size_z_cm << " cm\n";
                    }
                }

                // Update PCL viewer (no shared access)
                viewer.spinOnce(10);

                // VIDEO DATA TRANSMISSION - separate mutex for video
                try {
                    // Get current OBB for video transmission (separate atomic read)
                    OBBData current_obb_for_video;
                    bool obb_valid_for_video = false;
                    {
                        std::lock_guard<std::mutex> lock(shared.data_mutex);
                        current_obb_for_video = shared.obb;
                        obb_valid_for_video = current_obb_for_video.valid;
                    } // Mutex released immediately
                    
                    // Prepare video data WITHOUT any locks
                    uint32_t header_dims = htonl(4);
                    float obb_size_x = obb_valid_for_video ? (current_obb_for_video.obb_max.x - current_obb_for_video.obb_min.x) : 0.0f;
                    float obb_size_y = obb_valid_for_video ? (current_obb_for_video.obb_max.y - current_obb_for_video.obb_min.y) : 0.0f;
                    float obb_size_z = obb_valid_for_video ? (current_obb_for_video.obb_max.z - current_obb_for_video.obb_min.z) : 0.0f;
                    float obb_size_x_cm = obb_size_x / 10.0f;
                    float obb_size_y_cm = obb_size_y / 10.0f;
                    float obb_size_z_cm = obb_size_z / 10.0f;
                    float obb_volume = obb_valid_for_video ? (obb_size_x * obb_size_y * obb_size_z) : 0.0f;
                    float obb_data[4] = {obb_size_x_cm, obb_size_y_cm, obb_size_z_cm, obb_volume};
                    
                    // Get video client connection (separate mutex for video)
                    int video_client = -1;
                    {
                        std::lock_guard<std::mutex> lock(shared.video_mutex);
                        video_client = shared.video_client_fd;
                    }
                    
                    // Send video data without any locks
                    if (video_client >= 0) {
                        // Send dimensions
                        if (send(video_client, &header_dims, sizeof(header_dims), 0) < 0) {
                            std::cerr << "Video send failed (header), closing connection" << std::endl;
                            std::lock_guard<std::mutex> lock(shared.video_mutex);
                            close(shared.video_client_fd);
                            shared.video_client_fd = -1;
                        } else if (send(video_client, reinterpret_cast<char*>(obb_data), sizeof(obb_data), 0) < 0) {
                            std::cerr << "Video send failed (data), closing connection" << std::endl;
                            std::lock_guard<std::mutex> lock(shared.video_mutex);
                            close(shared.video_client_fd);
                            shared.video_client_fd = -1;
                        } else {
                            // Send vertex data
                            uint32_t header_verts = htonl(24);
                            if (send(video_client, &header_verts, sizeof(header_verts), 0) >= 0) {
                                if (obb_valid_for_video) {
                                    std::vector<Eigen::Vector3f> obb_vertices = calculateOBBVertices(current_obb_for_video);
                                    for (const auto &vertex : obb_vertices) {
                                        float vertex_data[3] = {vertex.x(), vertex.y(), vertex.z()};
                                        if (send(video_client, reinterpret_cast<char*>(vertex_data), sizeof(vertex_data), 0) < 0) {
                                            std::cerr << "Video send failed (vertex), closing connection" << std::endl;
                                            std::lock_guard<std::mutex> lock(shared.video_mutex);
                                            close(shared.video_client_fd);
                                            shared.video_client_fd = -1;
                                            break;
                                        }
                                    }
                                } else {
                                    // Send zero vertices
                                    float zero_vertex[3] = {0.0f, 0.0f, 0.0f};
                                    for (int i = 0; i < 8; ++i) {
                                        if (send(video_client, reinterpret_cast<char*>(zero_vertex), sizeof(zero_vertex), 0) < 0) {
                                            std::cerr << "Video send failed (zero vertex), closing connection" << std::endl;
                                            std::lock_guard<std::mutex> lock(shared.video_mutex);
                                            close(shared.video_client_fd);
                                            shared.video_client_fd = -1;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                catch (const std::exception& e) {
                    std::cerr << "Video transmission error: " << e.what() << std::endl;
                }
                
                // Brief sleep without any locks held
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
                
            } catch (const std::exception& e) {
                std::cerr << "Visualization loop error: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } catch (...) {
                std::cerr << "Unknown visualization loop error" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal visualization error: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "Fatal unknown visualization error" << std::endl;
    }
}