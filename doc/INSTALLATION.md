# 🚀 Install vS-Graphs

## 📝 I. Package Installation

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

### Install Required ROS Libraries

Ensure that all necessary `ROS` dependencies are installed:

```bash
# Installs `ros-noetic-backward-ros` & `ros-noetic-rviz-visual-tools`
rosdep install --from-paths src --ignore-src -y
```

> 🛎️ Note: The current version of vS-Graphs supports **ROS Noetic** and is primarily tested on Ubuntu 20.04.

### Install `hector-trajectory-server` (Optional)

Using this library you can visualize the real-time trajectory of camera of IMU.
[`Hector-trajectory-server`](http://wiki.ros.org/hector_trajectory_server) is a ROS package that enables real-time trajectory visualization of a camera or IMU. It is useful for monitoring SLAM progress during runtime. You can install it via the below command:

```bash
# Check the ROS-DISTRO (ROS1 Noetic)
sudo apt install ros-[DISTRO]-hector-trajectory-server
```

### Install RealSense Library (Optional)

To use an Intel RealSense camera for **live mode** or **data collection**, you will need to install the necessary drivers and libraries. Please follow the detailed setup guide available in [RealSense Setup Instructions](/doc/RealSense/README.md) page. This includes steps for:

- Installing `librealsense`
- Verifying camera connection
- Enabling live streaming or data recording

### Install `aruco_ros` (Optional) <a id="aruco"></a>

vS-Graphs can operate independently of fiducial markers, but it also supports enhanced semantic labeling using detected markers (e.g., assigning room names) when a valid marker database is provided.

If you would like to use marker-driven data augmentation to the generated scene graph, install the [`aruco_ros`](https://github.com/pal-robotics/aruco_ros) library:

```bash
# Clone (ROS1 Noetic)
git clone -b noetic-devel git@github.com:pal-robotics/aruco_ros.git
```

> 🛎️ Note: Use the sample modified version of the marker detector launch file available [here](doc/template_aruco_ros.launch). This setup ensures proper marker detection and integration with vS-Graphs.

### Install the In-house Scene Segmenter <a id="segmenter"></a>

vS-Graphs relies on a ROS-based **Panoptic Scene Segmentation** module, available at [scene_segment_ros](https://github.com/snt-arg/scene_segment_ros), to semantically identify **building components** (i.e., walls and ground surfaces). This module supports multiple segmentation backends, including:

- [**YOSO**](https://github.com/hujiecpp/YOSO) _(recommended for best performance)_
- [**PanopticFCN**](https://github.com/dvlab-research/PanopticFCN)

Clone the repository and set it ready for integration:

```bash
# Clone (ROS1 Noetic)
git clone --recurse-submodules git@github.com:snt-arg/scene_segment_ros.git

# Install Python packages
pip install -r src/requirements.txt

# Follow the rest installation guide in the repository
```

### 🦊 Install Voxblox Skeleton <a id="voxblox"></a>

To detect **structural elements** (such as rooms and corridors), vS-Graphs requires `loco planning` integrated with `voxblox`, available [here](https://github.com/snt-arg/mav_voxblox_planning/tree/master). This module provides free-space information crucial for **cluster-based structural element detection**. Clone and install the module using below commands:

```bash
# Clone (ROS1 Noetic)
git clone git@github.com:snt-arg/mav_voxblox_planning.git
wstool init . ./mav_voxblox_planning/install/install_ssh.rosinstall
wstool update
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build
```

## ⚙️ II. Build

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
