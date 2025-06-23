# 3D Object Detection Algorithm Using a TOF Camera

A comprehensive C++ application for real-time point cloud processing with 3D visualization and network communication capabilities. This system processes point cloud data from Time-of-Flight (ToF) sensors, detects objects, calculates oriented bounding boxes (OBB), and provides real-time visualization.

## What This Application Does

- **Real-time Point Cloud Processing**: Receives 3D point data from ToF sensors via network
- **Object Detection**: Identifies objects in 3D space using advanced clustering algorithms
- **Bounding Box Calculation**: Computes precise 3D bounding boxes around detected objects
- **Live Visualization**: Shows processed point clouds and bounding boxes in real-time
- **Network Communication**: Supports multiple client connections for data streaming
- **Shadow Removal**: Advanced filtering to improve object detection accuracy

## System Requirements

- **Operating System**: Ubuntu 18.04 or later (tested on Ubuntu 20.04)
- **Memory**: Minimum 4GB RAM (8GB recommended)
- **Network**: Local network access for client-server communication
- **Android Device**: For using the ToF Test app (optional)

## Complete Installation Guide

### Step 1: Update Your System

```bash
sudo apt-get update
sudo apt-get upgrade -y
```

### Step 2: Install Build Tools

```bash
# Install essential build tools
sudo apt-get install -y build-essential cmake git

# Install specific compiler version (recommended)
sudo apt-get install -y gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90
```

### Step 3: Install Required Libraries with Specific Versions

#### PCL (Point Cloud Library) 1.10
```bash
# Install PCL 1.10 and its dependencies
sudo apt-get install -y libpcl-dev=1.10.0+dfsg-5ubuntu1
sudo apt-get install -y libpcl-common1.10 libpcl-io1.10 libpcl-features1.10
sudo apt-get install -y libpcl-filters1.10 libpcl-segmentation1.10
sudo apt-get install -y libpcl-visualization1.10 libpcl-search1.10
sudo apt-get install -y libpcl-surface1.10 libpcl-kdtree1.10

# If the specific version is not available, install the latest:
# sudo apt-get install -y libpcl-dev
```

#### Eigen3 (Linear Algebra Library)
```bash
# Install Eigen3 for matrix operations
sudo apt-get install -y libeigen3-dev

# Verify installation
pkg-config --modversion eigen3
```

#### VTK (Visualization Toolkit) 7.1
```bash
# Install VTK 7.1 for visualization
sudo apt-get install -y libvtk7-dev libvtk7.1 libvtk7.1-qt
sudo apt-get install -y vtk7

# Install additional VTK components
sudo apt-get install -y libvtk7-qt-dev
```

#### Boost Libraries
```bash
# Install Boost libraries for threading and system operations
sudo apt-get install -y libboost-all-dev

# Or install specific Boost components:
sudo apt-get install -y libboost-thread-dev libboost-chrono-dev
sudo apt-get install -y libboost-system-dev libboost-filesystem-dev
```

#### Additional Dependencies
```bash
# Install other required libraries
sudo apt-get install -y pkg-config
sudo apt-get install -y libflann-dev
sudo apt-get install -y libqhull-dev
```

### Step 4: Verify Library Installations

```bash
# Check PCL installation
pkg-config --modversion pcl_common-1.10

# Check Eigen3
pkg-config --modversion eigen3

# Check VTK
pkg-config --modversion vtk-7.1

# List installed Boost libraries
dpkg -l | grep libboost
```

## Building the Application

### Option 1: Quick Build (Recommended)

```bash
# Clone or navigate to your project directory
cd /path/to/your/project
git clone https://github.com/MinhalShafiq/android_object_dimension_measurement.git
cd android_object_dimension_measurement

# Install dependencies automatically
make install-deps

# Build the application
make

# The executable will be created as 'point_cloud_app'
```

### Option 2: Manual Build

```bash
# Create build directory
mkdir -p obj

# Compile manually
g++ -std=c++17 -O2 -Wall \
    -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -I/usr/include/vtk-7.1 \
    src/*.cpp \
    -lpcl_common -lpcl_io -lpcl_features -lpcl_filters -lpcl_segmentation \
    -lpcl_visualization -lpthread \
    -lvtkCommonCore-7.1 -lvtkCommonDataModel-7.1 -lvtkCommonTransforms-7.1 \
    -lvtkRenderingCore-7.1 -lvtkRenderingOpenGL2-7.1 -lvtkInteractionStyle-7.1 \
    -lboost_thread -lboost_chrono -lboost_system \
    -o point_cloud_app
```

## Running the Application

### Start the Server

```bash
# Make sure you're in the project directory
cd /path/to/your/project

# Run the application
./point_cloud_app
```

You should see output like:
```
Starting point cloud processing application...
Point cloud server listening on port 9080
Video server listening on port 9081
```

### Network Ports Used

- **Port 9080**: Point cloud data reception
- **Port 9081**: Video/visualization data streaming

## Android App Setup (ToF Test)

### Prerequisites
- Android device with ToF sensor (recommended) or any Android device
- ToF Test app installed on your device
- Both devices (computer and Android) on the same network

### Configuration Steps

1. **Find Your Computer's IP Address**
   ```bash
   # On your computer, find the IP address
   ip addr show
   # Or use:
   hostname -I
   # Look for an address like 192.168.1.xxx or 10.0.0.xxx
   ```

2. **Configure the Android App**
   - Open the ToF Test app on your Android device
   - Go to Settings or Connection settings
   - Enter your computer's IP address (from step 1)
   - Set the port to **9080**
   - Save the settings

3. **Connect to the Server**
   - Make sure your C++ application is running
   - In the ToF Test app, tap "Connect" or "Start"
   - You should see "New point cloud client connected" in your terminal

4. **Start Data Streaming**
   - The app should now be sending point cloud data to your computer
   - You'll see real-time processing output in the terminal
   - The PCL visualizer window will show the 3D point cloud data

### Troubleshooting Connection Issues

**App won't connect:**
- Verify both devices are on the same Wi-Fi network
- Check if firewall is blocking the ports:
  ```bash
  sudo ufw allow 9080
  sudo ufw allow 9081
  ```
- Test connectivity:
  ```bash
  netstat -ln | grep 9080
  ```

**No data received:**
- Check the app's ToF sensor permissions
- Restart both the app and the C++ application
- Verify the IP address is correct

## Understanding the Output

### Terminal Output
When running successfully, you'll see:
- Point cloud processing statistics
- Object detection results
- Bounding box calculations
- Performance metrics

### Visualization Window
- **White points**: Background/environment
- **Purple points**: Detected objects
- **Green wireframe**: Calculated bounding box
- **Yellow spheres**: Bounding box vertices

### Example Output
```
Processing 3 clusters
Selected cluster 0 with highest confidence score: 0.847
Valid OBB computed successfully!
OBB dims: 25.3 x 18.7 x 42.1 cm
```

## File Structure and Architecture

### Core Modules
- **`main.cpp`** - Application entry point and thread management
- **`structures.h`** - Core data structures and shared memory
- **`network_server.h/cpp`** - TCP server for point cloud and video data
- **`point_cloud_processing.h/cpp`** - Point cloud filtering and shadow removal
- **`obb_calculator.h/cpp`** - Oriented bounding box calculations
- **`visualization.h/cpp`** - Real-time 3D visualization
- **`geometry_utils.h/cpp`** - 2D/3D geometric utility functions

### DBSCAN Clustering
- **`DBSCAN_simple.h`** - Basic DBSCAN implementation
- **`DBSCAN_kdtree.h`** - KD-tree optimized DBSCAN
- **`DBSCAN_precomp.h`** - Precomputed distance DBSCAN

## Advanced Configuration

### Camera Parameters
Edit `CameraParams.yaml` to adjust the visualization camera:
```
12.694,12694/0,0,1/0,0,-954.594/0,1,0/0.8575/960,540/82,109
```

### Processing Parameters
Key parameters can be modified in `network_server.cpp`:
- `voxel_size`: Point cloud downsampling (default: 12.0mm)
- `cluster_tolerance`: DBSCAN clustering distance (default: 20.0mm)
- `min_cluster_size`: Minimum points per cluster (default: 500)
- `distance_thresh`: Ground plane detection threshold (default: 25.0mm)

## Performance Optimization

### For Better Performance
```bash
# Increase thread priority
sudo nice -n -10 ./point_cloud_app

# Monitor system resources
htop
```

### Memory Usage
- Typical RAM usage: 200-500MB
- Large point clouds may require more memory

## Troubleshooting

### Build Issues
```bash
# Clear build cache
make clean

# Reinstall dependencies
make install-deps

# Check library versions
pkg-config --list-all | grep -E "(pcl|vtk|eigen)"
```

### Runtime Issues
```bash
# Check if ports are available
netstat -tulpn | grep -E "(9080|9081)"

# Monitor application logs
./point_cloud_app 2>&1 | tee app.log
```

### Common Problems

**"Failed to create server socket"**
- Port already in use
- Run: `sudo lsof -i :9080` to find conflicting process

**"Segmentation fault"**
- Usually indicates library version mismatch
- Reinstall PCL and VTK with correct versions

**"No clusters found"**
- Adjust clustering parameters
- Check input data quality

## Development and Testing

### For Developers
```bash
# Debug build
make clean
g++ -g -DDEBUG src/*.cpp -o point_cloud_app_debug [libraries...]

# Run with debugger
gdb ./point_cloud_app_debug
```

### Testing Without Android App
You can test the system by creating synthetic point cloud data or using sample files.

## Support and Documentation

- Ensure all dependencies are correctly installed with specified versions
- Check that your system meets the minimum requirements
- For build issues, verify library paths and versions
- For runtime issues, check network connectivity and permissions

This application represents a complete pipeline for real-time 3D object detection and analysis, suitable for research, development, and practical applications in robotics, augmented reality, and spatial computing.