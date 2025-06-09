# Point Cloud Processing Application

A modular C++ application for processing point clouds with real-time visualization and network communication capabilities.

## Architecture

The codebase has been split into multiple focused modules for better maintainability:

### Core Modules

- **`structures.h`** - Core data structures (OBBData, SharedData)
- **`geometry_utils.h/cpp`** - 2D/3D geometric utility functions
- **`point_cloud_processing.h/cpp`** - Point cloud filtering and shadow removal
- **`obb_calculator.h/cpp`** - Oriented Bounding Box calculations
- **`visualization.h/cpp`** - PCL-based 3D visualization
- **`network_server.h/cpp`** - TCP server implementations
- **`signal_handler.h/cpp`** - Signal handling utilities
- **`main.cpp`** - Application entry point

## Key Features

- **Modular Architecture**: Each component has a single responsibility
- **Thread-Safe Operations**: Proper mutex handling for shared data
- **Advanced Point Cloud Processing**: Shadow removal, outlier filtering
- **Real-time Visualization**: PCL-based 3D viewer
- **Network Communication**: TCP servers for point cloud and video data
- **Temporal Stability**: Smoothed OBB calculations

## Build System

```bash
# Install dependencies
make install-deps

# Build the application
make

# Clean build artifacts
make clean
```

## File Organization Benefits

1. **Maintainability**: Each file focuses on one specific functionality
2. **Testability**: Individual components can be tested in isolation
3. **Reusability**: Modules can be reused across different projects
4. **Collaboration**: Multiple developers can work on different modules
5. **Clear Dependencies**: Header files make dependencies explicit

## Dependencies

- PCL (Point Cloud Library)
- Eigen3 (Linear algebra)
- VTK (Visualization Toolkit)
- Boost libraries
- Standard C++17 features

## Usage

The application runs three main threads:
1. Point cloud processing server (port 9080)
2. Video data server (port 9081)
3. Real-time visualization

Each module is designed to be as independent as possible while maintaining clean interfaces between components.