# 🚀 Install vS-Graphs

## 📝 I. Prerequisites

### Install OpenCV

vS-Graphs requires **OpenCV v4.2+** for computer vision tasks, which can be installed via the [installation instructions](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html) page. For Ubuntu 24.04, install it using below commands.

```bash
sudo apt update
sudo apt install libopencv-dev -y

# Check the version
dpkg -l libopencv-dev
```

### Install Eigen3

vS-Graphs relies on **Eigen3**, a C++ template library for linear algebra (matrices, vectors, solvers, etc.). You can install it via APT:

```bash
sudo apt install libeigen3-dev
```

> 🛎️ Tip: `libeigen3-dev` is a header-only library and no linking is required, as `CMake` will automatically find it.

### Install Pangolin

[Pangolin](https://github.com/stevenlovegrove/Pangolin) is a lightweight and portable library used for visualizing 3D data, camera views, and prototyping video-based programs. vS-Graphs has been tested with **Pangolin v0.8**.

```bash
# Clone
git clone --branch v0.8 --depth 1 https://github.com/stevenlovegrove/Pangolin.git

# Install using CMake
cd Pangolin
mkdir build && cd build
cmake ..
make -j
sudo make install
```

### Install `hector-trajectory-server` (Optional)

Using this library you can visualize the real-time trajectory of `camera/IMU`.
[`hector-trajectory-server`](http://wiki.ros.org/hector_trajectory_server) is a ROS package that enables real-time trajectory visualization of a camera or IMU. It is useful for monitoring SLAM progress during runtime. You can install it via the below command:

```bash
# Check the ROS-DISTRO
sudo apt install ros-[DISTRO]-hector-trajectory-server
```

### Install RealSense Library (Optional)

To use an Intel RealSense camera for **live mode** or **data collection**, you will need to install the necessary drivers and libraries. Please follow the detailed setup guide available in [RealSense Setup Instructions](/doc/RealSense/README.md) page. This includes steps for:

- Installing `librealsense`
- Verifying camera connection
- Enabling live streaming or data recording

## ⚙️ Installation <a id="installation"></a>

After installing the prerequisites, you can clone the repository and follow the commands below:

### II. Cloning the `aruco_ros` Repository (optional) <a id="aruco"></a>

This package (available [here](https://github.com/pal-robotics/aruco_ros)) enables you to detect ArUco Markers in cameras' field of view. Accordingly, install it using the commands below in **the same folder (i.e., [workspace]/src)**:

```
cd ~/catkin_ws/src/

# Cloning the latest code
git clone -b noetic-devel git@github.com:pal-robotics/aruco_ros.git
```

Instead of the original launch file, you can use the sample modified `marker_publisher.launch` file for this library available [here](doc/aruco_ros_marker_publisher.launch), which works fine with the live feed for RealSense cameras (`imageRaw` and `cameraInfo` should be changed based on the use case). Do not forget to set proper `ref_frame`, `markerSize`, `imageRaw`, and `cameraInfo` values in the launch file.

### III. Cloning the Developed `scene_segment_ros` Repository <a id="segmenter"></a>

This package (in the open sourcing process) enables you to **segment the scene** and **detect semantic entities** in the scene. Accordingly, install it using the commands below in **the same folder (i.e., [workspace]/src)**:

<!-- available [here](https://github.com/snt-arg/scene_segment_ros) -->

```
cd ~/catkin_ws/src/

# Cloning the latest code
git clone git@github.com:[repo]
```

You can then run the scene semantic segmentor using the commands `roslaunch segmenter_ros segmenter_pFCN.launch` for using the **PanopticFCN** model or `roslaunch segmenter_ros segmenter_yoso.launch` for YOSO model.

### IV. 🦊 Installing Voxblox Skeleton <a id="voxblox"></a>

This package (available [here](https://github.com/snt-arg/mav_voxblox_planning/tree/master)) enables you to use `voxblox` and `loco planning` for cluster-based room detection. Accordingly, install it using the commands below in a workspace **not necessarily the same folder (i.e., [workspace]/src)**:

```
cd ~/catkin_ws/src/

# Cloning the latest code
git clone git@github.com:snt-arg/mav_voxblox_planning.git
wstool init . ./mav_voxblox_planning/install/install_ssh.rosinstall
wstool update
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build
```

### V. Installing Other Required Libraries <a id="libraries"></a>

First, make sure that you have installed all the required dependencies, such as `ros-noetic-backward-ros` and `ros-noetic-rviz-visual-tools`, using the command `rosdep install --from-paths src --ignore-src -y`.

### VI. Build

Build the installed libraries and modules using `catkin build`. As a shortcut, you can add a new alias to the `bashrc` file to run the environment whenever needed, like below:

```
alias sourceros='source /opt/ros/noetic/setup.bash'
alias sourcevox="source ~/workspace/ros/voxblox_skeleton/devel/setup.bash"
alias sourcerealsense='source ~/workspace/realsense/rs_ros/devel/setup.bash'
alias sourcevsgraphs='source ~/workspace/ros/orbslam3_ros_ws/devel/setup.bash'
```

As a quick test, you can do as follows:

- Run a `roscore`
- Run the `aruco_ros` using `sourcevsgraphs` and then `roslaunch aruco_ros marker_publisher.launch [2>/dev/null]` for detecting multiple markers in the scene and publishing their poses.
  - The final results (scene with detected markers) produced by `aruco_ros` are published and accessible on `/aruco_marker_publisher/result` and the pose of the markers will be shown using `rostopic echo /aruco_marker_publisher/markers`.
- Run the Visual S-Graphs using `sourcevsgraphs` and then `roslaunch orb_slam3_ros vsgraphs_rgbd.launch [2>/dev/null]`
