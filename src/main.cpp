#include "structures.h"
#include "visualization.h"
#include "network_server.h"
#include "signal_handler.h"
#include <iostream>
#include <thread>
#include <csignal>
#include <exception>

int main() {
    // Signal handlers.
    std::signal(SIGSEGV, signalHandler);
    std::signal(SIGABRT, signalHandler);

    try {
        SharedData shared;
        
        std::thread videoThread(videoServer, std::ref(shared));
        std::thread pointCloudThread(pointCloudServer, std::ref(shared));
        std::thread visualizationThread(visualizePointCloud, std::ref(shared));

        // Wait for threads to finish
        visualizationThread.join();
        videoThread.detach(); 
        pointCloudThread.detach();

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal error in main: " << e.what() << std::endl;
        return 1;
    }
}