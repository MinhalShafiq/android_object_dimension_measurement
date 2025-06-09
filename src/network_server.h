#pragma once

#include "structures.h"

// Network server functions
void processClient(int client_fd, SharedData& shared);
void videoServer(SharedData& shared);
void pointCloudServer(SharedData& shared);