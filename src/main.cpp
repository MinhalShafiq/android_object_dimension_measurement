#include "structures.h"
#include "visualization.h"
#include "network_server.h"
#include "signal_handler.h"
#include <iostream>
#include <thread>
#include <csignal>
#include <exception>
#include <chrono>

int main() {
    // Signal handlers.
    std::signal(SIGSEGV, signalHandler);
    std::signal(SIGABRT, signalHandler);

    try {
        SharedData shared;
        
        std::cout << "Starting point cloud processing application..." << std::endl;
        
        // Start server threads first
        std::thread videoThread(videoServer, std::ref(shared));
        std::thread pointCloudThread(pointCloudServer, std::ref(shared));
        
        // Give servers time to start up
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Start visualization thread last
        std::thread visualizationThread(visualizePointCloud, std::ref(shared));

        // Wait for visualization thread (main UI thread)
        visualizationThread.join();
        
        std::cout << "Visualization thread ended, shutting down..." << std::endl;
        
        // Detach server threads (they will clean up when clients disconnect)
        videoThread.detach(); 
        pointCloudThread.detach();

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal error in main: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "Unknown fatal error in main" << std::endl;
        return 1;
    }
}