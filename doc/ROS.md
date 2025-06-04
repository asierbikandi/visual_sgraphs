# 🤖 ROS Topics, Params, and Services of vS-Graphs

### Subscribed Topics <a id="ros-sub"></a>

| Topic                                                            | Description                                           |
| ---------------------------------------------------------------- | ----------------------------------------------------- |
| `/imu`                                                           | for Mono/Stereo/RGBD-Inertial node                    |
| `/camera/image_raw`                                              | for Mono(-Inertial) node                              |
| `/camera/left/image_raw` and `/camera/right/image_raw`           | for Stereo(-Inertial) node                            |
| `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw` | for RGBD node                                         |
| `/aruco_marker_publisher/markers`                                | for ArUco marker library node                         |
| `/camera/color/image_segment`                                    | for Semantic segmenter library node (custom message)  |
| `/camera/color/image_segment_vis`                                | for Semantic segmenter library node (segmented image) |

### Published Topics <a id="ros-pub"></a>

| Topic                         | Description                                                          |
| ----------------------------- | -------------------------------------------------------------------- |
| `/tf`                         | with camera and imu-body poses in world frame                        |
| `/orb_slam3/camera_pose`      | left camera pose in world frame, published at camera rate            |
| `/orb_slam3/body_odom`        | imu-body odometry in world frame, published at camera rate           |
| `/orb_slam3/tracking_image`   | processed image from the left camera with key points and status text |
| `/orb_slam3/tracked_points`   | all key points contained in the sliding window                       |
| `/orb_slam3/all_points`       | all key points in the map                                            |
| `/orb_slam3/kf_markers`       | markers for all keyframes' positions                                 |
| `/orb_slam3/keyframe_image`   | keyframe poses images                                                |
| `/orb_slam3/fiducial_markers` | fiducial markers detected in the environment                         |
| `/orb_slam3/doors`            | doorways detected in the environment                                 |
| `/orb_slam3/planes`           | planes detected in the environment                                   |
| `/orb_slam3/rooms`            | corridors and rooms markers detected in the environment              |

### Params <a id="ros-param"></a>

| Param                                                     | Description                                                                                                        |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `offline`                                                 | live or reading rosbag file (offline)?                                                                             |
| `sys_params_file`                                         | path to the common system parameters (see below)                                                                   |
| `voc_file`                                                | path to ORB vocabulary file                                                                                        |
| `settings_file`                                           | path to settings file                                                                                              |
| `enable_pangolin`                                         | enable/disable Pangolin viewer and interface. (`true` by default)                                                  |
| `static_transform`                                        | enable/disable static transform between coordinate frames. (needs to be `true` for some datasets like `AutoSense`) |
| `roll`, `yaw`, and `pitch`                                | poses and dimensions of movement                                                                                   |
| `frame_map` <br /> `world_frame_id` <br /> `cam_frame_id` | different frame identifiers                                                                                        |

## 📍 Maps <a id="maps"></a>

The map file will have `.osa` extension, and is located in the `ROS_HOME` folder (`~/.ros/` by default).

### Load Map <a id="maps-load"></a>

- Set the name of the map file to be loaded with `System.LoadAtlasFromFile` param in the settings file (`.yaml`).
- If the map file is not available, `System.LoadAtlasFromFile` param should be commented out otherwise there will be error.

### Save Map <a id="maps-save"></a>

- **Option 1**: If `System.SaveAtlasToFile` is set in the settings file, the map file will be automatically saved when you kill the ros node.
- **Option 2**: You can also call the following ros service at the end of the session

```
rosservice call /orb_slam3/save_map [file_name]
```

### Services <a id="maps-services"></a>

- `rosservice call /orb_slam3/save_map [file_name]`: save the map as `[file_name].osa` in `ROS_HOME` folder.
- `rosservice call /orb_slam3/save_traj [file_name]`: save the estimated trajectory of camera and keyframes as `[file_name]_cam_traj.txt` and `[file_name]_kf_traj.txt` in `ROS_HOME` folder.
