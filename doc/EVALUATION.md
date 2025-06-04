## 📊 Evaluation <a id="eval"></a>

In order to evaluate the current method with others, such as UcoSLAM, ORB-SLAM 3.0, etc., you need to follow the below instructions:

- Prepare the `.txt` file containing robot poses, generated using a framework:
  1. Run `generate_pose_txt_files.py` in the `evaluation` folder of this repository.
     - Declare the file path to be saved, the slame method you want to obtain the poses, the dataset name, etc. before running it. For instance, if `slam_pose_semorb3_seq01.txt` is generated, it means it contain the SLAM poses of `Visual S-Graphs` on `Seq01` dataset instance.
     - It will create a new `txt` file, and the poses will be added there.
  2. Run the framework you need for the evaluation:
     - For `S-Graphs`, the file is generated while running it on a rosbag file.
     - For `ORB-SLAM 3.0 (ROS version)` and `Visual S-Graphs (current repo)`, they need to be run and a rosbag should be played to fill the `txt` file.
     - For `UcoSLAM` and `Semantic UcoSLAM`, we need to have the poses created in a new way. Accordingly, we need to put the poses created by the `S-Graphs` in `/tmp/timestamp.txt`, and then run the codes from the [ros-wrapper](https://github.com/snt-arg/ucoslam_ros_wrapper/tree/main/src):
       - UcoSLAM: `rosrun ucoslam_ros ucoslam_ros_video /[PATH]/vid.mp4 [PATH]/ucoslam_ros_wrapper/config/realsense_color_640_480_spot.yml -aruco-markerSize 0.78 -dict ARUCO -voc [PATH]/ucoslam_ros_wrapper/config/orb.fbow`
       - Semantic UcoSLAM: `rosrun ucoslam_ros ucoslam_ros_semantics_video /[PATH]/vid.mp4 [PATH]/ucoslam_ros_wrapper/config/realsense_color_640_480_spot.yml -aruco-markerSize 0.78 -dict ARUCO -voc [PATH]/ucoslam_ros_wrapper/config/orb.fbow`
  3. Finally, when the ground-truth (S-Graphs) and SLAM pose (e.g., UcoSLAM, etc.) are ready, you can use the [`evo_ape`](https://github.com/MichaelGrupp/evo) for evaluation, like `evo_ape tum s_graphs_pose_seq05.txt slam_pose_semuco_seq05.txt -va > results.txt --plot --plot_mode xy`