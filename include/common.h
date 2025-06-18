/**
 * This file is a modified version of a file from ORB-SLAM3.
 * 
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 * 
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 * 
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
// #include <tf/transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.hpp>

// #include <std_msgs/Header.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <pcl_ros/common/common.hpp>
// #include <pcl/PCLPointCloud2.hpp>
// #include <pcl/common/distances.hpp>
// #include <pcl/filters/voxel_grid.hpp>
//Tranfororm previos pcl headers for ros2
#include <pcl_ros/transforms.hpp>      // From your pcl_ros/ folder
#include <pcl_ros/point_cloud.hpp>    // From your pcl_ros/ folder (if needed)
#include <pcl_ros/pcl_node.hpp>       // From your pcl_ros/ folder (if needed)

// #include <sensor_msgs/PointCloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <visualization_msgs/Marker.hpp>
#include <visualization_msgs/msg/marker.hpp>

// #include <segmenter_ros/VSGraphDataMsg.hpp>
// #include <segmenter_ros/SegmenterDataMsg.hpp>
// #include <nav_msgs/Path.hpp>
// #include <visualization_msgs/MarkerArray.hpp>
// #include <pcl_conversions/pcl_conversions.hpp>
// #include <rviz_visual_tools/rviz_visual_tools.hpp>

// #include <message_filters/subscriber.hpp>
// #include <message_filters/time_synchronizer.hpp>
// #include <message_filters/sync_policies/approximate_time.hpp>

// // This file is created automatically, see here http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
// #include <orb_slam3_ros/SaveMap.hpp>

// // Transformation process
// #include <pcl_ros/transforms.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
// #include <tf2_ros/static_transform_broadcaster.h>

#include <segmenter_ros/msg/vs_graph_data_msg.hpp>
#include <segmenter_ros/msg/segmenter_data_msg.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

// This file is created automatically, see here http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
#include <orb_slam3_ros/srv/save_map.hpp>

// Transformation process
#include <pcl_ros/transforms.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/static_transform_broadcaster.h>


// ORB-SLAM3-specific libraries
#include "System.h"
#include "ImuTypes.h"
#include "Types/SystemParams.h"

// ArUco-ROS library
// #include <aruco_msgs/MarkerArray.h>

// Semantics
#include "Semantic/Door.h"
#include "Semantic/Room.h"
#include "Semantic/Marker.h"

// Custom Messages
// #include <orb_slam3_ros/VSGraphsAllWallsData.h>
// #include <orb_slam3_ros/VSGraphsAllDetectdetRooms.h>

#include <orb_slam3_ros/msg/vs_graphs_all_walls_data.hpp>
#include <orb_slam3_ros/msg/vs_graphs_wall_data.hpp>
#include <orb_slam3_ros/msg/vs_graphs_room_data.hpp>
#include <orb_slam3_ros/msg/vs_graphs_all_detectdet_rooms.hpp>

using json = nlohmann::json;

class ORB_SLAM3::SystemParams;
extern ORB_SLAM3::System *pSLAM;
extern ORB_SLAM3::System::eSensor sensorType;

extern bool colorPointcloud;
extern double roll, pitch, yaw;
extern bool pubStaticTransform, pubPointClouds;
extern std::string world_frame_id, cam_frame_id, imu_frame_id, frameMap, frameBC, frameSE;

// List of visited Fiducial Markers in different timestamps
extern std::vector<std::vector<ORB_SLAM3::Marker *>> markersBuffer;

// List of white space cluster points obtained from `voxblox_skeleton`
extern std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

// List of GNN-based room candidates
extern std::vector<ORB_SLAM3::Room *> gnnRoomCandidates;

extern ros::Publisher pubKFImage;
extern ros::Publisher pubAllWalls;
extern rclcpp::Time lastPlanePublishTime;
extern image_transport::Publisher pubTrackingImage;
extern ros::Publisher pubCameraPose, pubCameraPoseVis, pubOdometry, pubKeyFrameMarker;
extern ros::Publisher pubTrackedMappoints, pubAllMappoints, pubSegmentedPointcloud;

struct MapPointStruct
{
    int clusterId;
    Eigen::Vector3f coordinates;

    MapPointStruct(Eigen::Vector3f coords) : coordinates(coords), clusterId(-1) {}
};

void setupServices(ros::NodeHandle &, std::string);
void publishTopics(rclcpp::Time, Eigen::Vector3f = Eigen::Vector3f::Zero());
void setupPublishers(ros::NodeHandle &, image_transport::ImageTransport &, std::string);

void publishTrackingImage(cv::Mat, rclcpp::Time);
void publishCameraPose(Sophus::SE3f, rclcpp::Time);
void publishRooms(std::vector<ORB_SLAM3::Room *>, rclcpp::Time);
void publishDoors(std::vector<ORB_SLAM3::Door *>, rclcpp::Time);
void publishPlanes(std::vector<ORB_SLAM3::Plane *>, rclcpp::Time);
void publishTFTransform(Sophus::SE3f, string, string, rclcpp::Time);
void publishAllPoints(std::vector<ORB_SLAM3::MapPoint *>, rclcpp::Time);
void publishTrackedPoints(std::vector<ORB_SLAM3::MapPoint *>, rclcpp::Time);
void publishFiducialMarkers(std::vector<ORB_SLAM3::Marker *>, rclcpp::Time);
void publishSegmentedCloud(std::vector<ORB_SLAM3::KeyFrame *>, rclcpp::Time);
void publishKeyFrameImages(std::vector<ORB_SLAM3::KeyFrame *>, rclcpp::Time);
void publishKeyFrameMarkers(std::vector<ORB_SLAM3::KeyFrame *>, rclcpp::Time);
void publishBodyOdometry(Sophus::SE3f, Eigen::Vector3f, Eigen::Vector3f, rclcpp::Time);

/**
 * @brief Publishes all mapped walls to detect possible rooms.
 *
 * @param walls The vector of mapped walls to be published.
 * @param time The timestamp for the message.
 */
void publishAllMappedWalls(std::vector<ORB_SLAM3::Plane *>, rclcpp::Time);

void clearKFClsClouds(std::vector<ORB_SLAM3::KeyFrame *>);

bool saveMapService(orb_slam3_ros::SaveMap::Request &, orb_slam3_ros::SaveMap::Response &);
bool saveTrajectoryService(orb_slam3_ros::SaveMap::Request &, orb_slam3_ros::SaveMap::Response &);
bool saveMapPointsAsPCDService(orb_slam3_ros::SaveMap::Request &, orb_slam3_ros::SaveMap::Response &);

/**
 * @brief Converts a SE3f to a cv::Mat
 *
 * @param data The SE3f data to be converted
 */
cv::Mat SE3fToCvMat(Sophus::SE3f data);

/**
 * @brief Converts a SE3f to a tf::Transform
 *
 * @param data The SE3f data to be converted
 */
tf::Transform SE3fToTFTransform(Sophus::SE3f data);

/**
 * @brief Converts a vector of MapPoints to a PointCloud2 message
 *
 * @param mapPoints The vector of MapPoints to be converted
 * @param msgTime The timestamp for the PointCloud2 message
 */
sensor_msgs::PointCloud2 mapPointToPointcloud(std::vector<ORB_SLAM3::MapPoint *> mapPoints, rclcpp::Time msgTime);

/**
 * @brief Publishes a static transformation (TF) between two coordinate frames and define a
 * fixed spatial relationship among them.
 *
 * @param parentFrameId The parent frame ID for the static transformation
 * @param childFrameId The child frame ID for the static transformation
 * @param msgTime The timestamp for the transformation message
 */
void publishStaticTFTransform(string parentFrameId, string childFrameId, rclcpp::Time msgTime);

/**
 * @brief Publishes the free space clusters obtained from `voxblox_skeleton` as a PointCloud2 message
 *
 * @param skeletonClusterPoints The list of free space cluster points
 * @param msgTime The timestamp for the PointCloud2 message
 */
void publishFreeSpaceClusters(std::vector<std::vector<Eigen::Vector3d>>, rclcpp::Time);

/**
 * @brief Adds the markers to the buffer to be processed
 * @param markerArray The array of markers received from `aruco_ros`
 */
// void addMarkersToBuffer(const aruco_msgs::MarkerArray &markerArray);

/**
 * @brief Avoids adding duplicate markers to the buffer by checking the timestamp
 * @param frameTimestamp The timestamp of the frame that captured the marker
 */
std::pair<double, std::vector<ORB_SLAM3::Marker *>> findNearestMarker(double frameTimestamp);

/**
 * @brief Gets skeleton voxels from `voxblox_skeleton` to be processed
 * @param skeletonArray The array of skeleton voxels received
 */
void setVoxbloxSkeletonCluster(const visualization_msgs::MarkerArray &skeletonArray);

/**
 * @brief Gets the set of room candidates detected by the GNN-based room detection module
 * @param msgGNNRooms The message containing the detected room candidates
 */
void setGNNBasedRoomCandidates(const orb_slam3_ros::VSGraphsAllDetectdetRooms &msgGNNRooms);