#include "signal_handler.h"
#include <iostream>
#include <cstdlib>

void signalHandler(int signal) {
    std::cerr << "Caught signal " << signal << ". Exiting gracefully." << std::endl;
    std::_Exit(1); 
}