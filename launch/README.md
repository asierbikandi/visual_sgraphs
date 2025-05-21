## Default

<!-- <arg name="offline" default="true" />
    <arg name="launch_rviz" default="true" />
    <arg name="colored_pointcloud" default="true" />
    <arg name="visualize_segmented_scene" default="true" />

    <arg name="sensor_config" default="RealSense_D435i" />
    <arg name="camera_frame" default="camera" />
    <arg name="rgb_image_topic" default="/camera/color/image_raw" />
    <arg name="rgb_camera_info_topic" default="/camera/color/camera_info" />
    <arg name="depth_image_topic" default="/camera/aligned_depth_to_color/image_raw" /> -->

## ICL

Topic: Value
sensor_config:=ICL
depth_image_topic:=/camera/depth/image_raw

<!-- roslaunch orb_slam3_ros vs_graphs.launch sensor_config:=ICL depth_image_topic:=/camera/depth/image_raw -->

## OpenLoris

Topic: Value
sensor_config:=OpenLorisScene
rgb_image_topic:=/d400/color/image_raw
depth_image_topic:=/d400/aligned_depth_to_color/image_raw
rgb_camera_info_topic:=/d400/color/camera_info
camera_frame:=d400_color

<!-- roslaunch orb_slam3_ros vs_graphs.launch sensor_config:=OpenLorisScene rgb_image_topic:=/d400/color/image_raw depth_image_topic:=/d400/aligned_depth_to_color/image_raw rgb_camera_info_topic:=/d400/color/camera_info camera_frame:=d400_color -->

## ScanNet

Topic: Value
sensor_config:=ScanNet
rgb_image_topic:=/camera/color/image_raw
depth_image_topic:=/camera/depth/image_raw
rgb_camera_info_topic:=/camera/color/camera_info
camera_frame:=camera

<!-- roslaunch orb_slam3_ros vs_graphs.launch sensor_config:=ScanNet rgb_image_topic:=/camera/color/image_raw depth_image_topic:=/camera/depth/image_raw rgb_camera_info_topic:=/camera/color/camera_info camera_frame:=camera -->

## TUM-RGBD

Topic: Value
rgb_image_topic:=/camera/rgb/image_color
depth_image_topic:=/camera/depth/image
rgb_camera_info_topic:=/camera/rgb/camera_info
sensor_config:=TUM1/TUM2/TUM3
camera_frame:=kinect

roslaunch orb_slam3_ros vs_graphs.launch rgb_image_topic:=/camera/rgb/image_color depth_image_topic:=/camera/depth/image rgb_camera_info_topic:=/camera/rgb/camera_info sensor_config:=TUM3 camera_frame:=kinect

## Live
