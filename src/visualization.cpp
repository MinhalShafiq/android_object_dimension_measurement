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
                // Wrap each iteration in a try/catch to protect against runtime errors.
                try {
                    bool new_data = false;
                    std::vector<CloudT::Ptr> clusters_local;
                    OBBData obb_local;

                    {
                        std::lock_guard<std::mutex> lock(shared.data_mutex);
                        if (shared.new_data_available) {
                            clusters_local = shared.clusters;
                            obb_local = shared.obb;
                            shared.new_data_available = false;
                            new_data = true;
                        }
                    }

                    if (new_data) {
                        // Remove clusters if they exist.
                        for (const auto& id : cluster_ids) {
                            if (viewer.contains(id))
                                viewer.removePointCloud(id);
                        }
                        cluster_ids.clear();

                        for (const auto& id : shape_ids) {
                            if (viewer.contains(id))
                                viewer.removeShape(id);
                        }
                        shape_ids.clear();

                        for (size_t i = 0; i < clusters_local.size(); ++i) {
                            std::string id = "cluster_" + std::to_string(i);
                            viewer.addPointCloud(clusters_local[i], id);
                            viewer.setPointCloudRenderingProperties(
                                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);
                            cluster_ids.push_back(id);
                        }

                        if (obb_local.valid) {
                            float obb_size_x = obb_local.obb_max.x - obb_local.obb_min.x;
                            float obb_size_y = obb_local.obb_max.y - obb_local.obb_min.y;
                            float obb_size_z = obb_local.obb_max.z - obb_local.obb_min.z;

                            Eigen::Vector3f obb_center(
                                obb_local.obb_pos.x,
                                obb_local.obb_pos.y,
                                obb_local.obb_pos.z
                            );
                            Eigen::Quaternionf obb_quat(obb_local.rot_matrix);

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

                            std::vector<Eigen::Vector3f> obb_vertices = calculateOBBVertices(obb_local);
                            for (size_t i = 0; i < obb_vertices.size(); ++i) {
                                std::string vertex_id = "vertex_" + std::to_string(i);
                                viewer.addSphere(pcl::PointXYZ(obb_vertices[i].x(), obb_vertices[i].y(), obb_vertices[i].z()),
                                                5.0,
                                                1.0, 1.0, 0.0,
                                                vertex_id);
                                shape_ids.push_back(vertex_id);
                            }

                            float obb_size_x_cm = obb_size_x / 10.0f;
                            float obb_size_y_cm = obb_size_y / 10.0f;
                            float obb_size_z_cm = obb_size_z / 10.0f;
                            float obb_volume = obb_size_x * obb_size_y * obb_size_z;

                            std::cout << "\n--- Minimal OBB (Hull-based) ---\n";
                            std::cout << "OBB dims: " 
                                    << obb_size_x_cm << " x "
                                    << obb_size_y_cm << " x "
                                    << obb_size_z_cm << " cm\n";
                        }
                    }

                    viewer.spinOnce(10);
                }
                catch (const std::exception& e) {
                    std::cerr << "Error in visualization loop: " << e.what() << std::endl;
                }

                // Section to send data to video client.
                try {
                    {
                        std::lock_guard<std::mutex> lock(shared.video_mutex);
                        if (shared.video_client_fd >= 0) {
                            OBBData currentObb;
                            {
                                std::lock_guard<std::mutex> dataLock(shared.data_mutex);
                                currentObb = shared.obb;
                            }
                    
                            uint32_t header_dims = htonl(4);
                            if (send(shared.video_client_fd, &header_dims, sizeof(header_dims), 0) < 0) {
                                std::cerr << "Send header_dims failed, closing video client connection." << std::endl;
                                close(shared.video_client_fd);
                                shared.video_client_fd = -1;
                                continue;
                            }
                    
                            float obb_size_x = currentObb.valid ? (currentObb.obb_max.x - currentObb.obb_min.x) : 0.0f;
                            float obb_size_y = currentObb.valid ? (currentObb.obb_max.y - currentObb.obb_min.y) : 0.0f;
                            float obb_size_z = currentObb.valid ? (currentObb.obb_max.z - currentObb.obb_min.z) : 0.0f;
                            float obb_size_x_cm = obb_size_x / 10.0f;
                            float obb_size_y_cm = obb_size_y / 10.0f;
                            float obb_size_z_cm = obb_size_z / 10.0f;
                            float obb_volume = currentObb.valid ? (obb_size_x * obb_size_y * obb_size_z) : 0.0f;
                            float obb_data[4] = {obb_size_x_cm, obb_size_y_cm, obb_size_z_cm, obb_volume};
                            if (send(shared.video_client_fd, reinterpret_cast<char*>(obb_data), sizeof(obb_data), 0) < 0) {
                                std::cerr << "Send obb_data failed, closing video client connection." << std::endl;
                                close(shared.video_client_fd);
                                shared.video_client_fd = -1;
                                continue;
                            }
                    
                            if (currentObb.valid) {
                                std::vector<Eigen::Vector3f> obb_vertices = calculateOBBVertices(currentObb);
                                uint32_t header_verts = htonl(24);
                                if (send(shared.video_client_fd, &header_verts, sizeof(header_verts), 0) < 0) {
                                    std::cerr << "Send header_verts failed, closing video client connection." << std::endl;
                                    close(shared.video_client_fd);
                                    shared.video_client_fd = -1;
                                    continue;
                                }
                                for (const auto &vertex : obb_vertices) {
                                    float vertex_data[3] = {vertex.x(), vertex.y(), vertex.z()};
                                    if (send(shared.video_client_fd, reinterpret_cast<char*>(vertex_data), sizeof(vertex_data), 0) < 0) {
                                        std::cerr << "Send vertex_data failed, closing video client connection." << std::endl;
                                        close(shared.video_client_fd);
                                        shared.video_client_fd = -1;
                                        break;
                                    }
                                }
                            } else {
                                uint32_t header_verts = htonl(24);
                                if (send(shared.video_client_fd, &header_verts, sizeof(header_verts), 0) < 0) {
                                    std::cerr << "Send header_verts (zero) failed, closing video client connection." << std::endl;
                                    close(shared.video_client_fd);
                                    shared.video_client_fd = -1;
                                    continue;
                                }
                                float zero_vertex[3] = {0.0f, 0.0f, 0.0f};
                                for (int i = 0; i < 8; ++i) {
                                    if (send(shared.video_client_fd, reinterpret_cast<char*>(zero_vertex), sizeof(zero_vertex), 0) < 0) {
                                        std::cerr << "Send zero_vertex failed, closing video client connection." << std::endl;
                                        close(shared.video_client_fd);
                                        shared.video_client_fd = -1;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
                catch (const std::exception& e) {
                    std::cerr << "Error when sending video data: " << e.what() << std::endl;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
            catch (...) {
                std::cerr << "Unknown error in visualization loop." << std::endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Viewer error: " << e.what() << std::endl;
    }
}