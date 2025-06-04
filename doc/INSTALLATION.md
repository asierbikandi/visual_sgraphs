# 🚀 Install vS-Graphs

## 📝 Prerequisites <a id="prerequisites"></a>

### OpenCV <a id="opencv"></a>

Check the OpenCV version on your computer (required [at least 3.0](https://github.com/UZ-SLAMLab/VS_GRAPHS)):

```
python3 -c "import cv2; print(cv2.__version__)"
```

On a freshly installed Ubuntu 20.04.4 LTS with desktop image, OpenCV 4.2.0 is already included. If a newer version is required (>= 3.0), follow [installation instruction](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html) and change the corresponding OpenCV version in `CMakeLists.txt`

### Eigen3 <a id="eigen"></a>

Install `Eigen`, which is a C++ template library for linear algebra (including matrices, vectors, and numerical solvers):

```
sudo apt install libeigen3-dev
```

### Pangolin <a id="pangolin"></a>

Install `Pangolin`, which is a set of lightweight and portable utility libraries for prototyping 3D, numeric or video based programs and algorithms:

```
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
```

### 🎞️ RealSense (Live Mode - optional) <a id="realsense"></a>

Please refer to [this page](/doc/RealSense/README.md) for detailed description on how to prepare a RealSense D400 series camera for live feed or data collection.

### 🎨 Kimera-Semantics (optional) <a id="kimera"></a>

Install `Kimera-Semantics` based on the installation guide introduced [here](https://github.com/MIT-SPARK/Kimera-Semantics/tree/master). In case you have `Ros Noetic`, you may face some errors related to `pcl` library and the build fails. In this case, you should apply `catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14` to be able to build it ([issue](https://github.com/MIT-SPARK/Kimera-Semantics/issues/67)).

### `hector-trajectory-server` (optional)

Using this library you can visualize the real-time trajectory of `camera/IMU`.

```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```

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