# Simple RealSense Camera Node
This package provides a simple implementation of a ROS node for Intel RealSense cameras. The code has been developed and tested on Windows, using a Mamba-managed Conda environment.


## Installation

Follow these steps to set up and activate the environment for the `simple_realsense_camera_node`.

1. **Clone the Repository:**
   Ensure that you have the package in your ROS workspace (e.g., `catkin_ws/src`). If not, clone it using:
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/VModugno/simple_realsense_camera_node.git
   ```

2. **Install the Intel RealSense SDK:**
- **Windows:**
  You can download and install the Intel RealSense SDK using the vcpkg dependency manager with the following commands:
  ```
  git clone https://github.com/Microsoft/vcpkg.git
  cd vcpkg
  ./bootstrap-vcpkg.bat
  ./vcpkg integrate install
  ./vcpkg install realsense2
  ```
- **Ubuntu:**
  Follow the instructions on the [Intel RealSense SDK installation page for Linux](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) to set up the SDK on Ubuntu.

3. **Create Mamba Environment:**
   Navigate to the root of the package and create the Mamba environment using the provided `win_environment.yaml` file:
   ```
   cd simple_realsense_camera_node
   mamba env create -f win_environment.yaml
   ```


4. **Activate the Environment:**
   Activate the newly created environment to use the package:
   ```
   conda activate win_ros_2
   ```

5. **Build the Package:**
   Navigate back to your ROS workspace root and build the package with `catkin_make`:
   ```
   cd ~/catkin_ws
   catkin_make
   ```

6. **Source the Setup File:**
   Source the ROS environment setup file to ensure all package paths are included:
   for ubuntu
   ```
   source devel/setup.bash
   ```

   for windows
   ```
   devel/setup.bat
   ```

## Usage

To run the `simple_realsense_camera_node`, use the provided ROS launch file:

```
roslaunch simple_realsense_camera_node start_camera.launch
```

This command will start the ROS node that interfaces with the RealSense camera, capturing and publishing image and depth data.
```

