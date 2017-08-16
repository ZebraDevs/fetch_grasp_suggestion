#ifndef GRASP_SAMPLER_H_
#define GRASP_SAMPLER_H_

// ROS
#include <actionlib/server/simple_action_server.h>
#include <rail_grasp_calculation_nodes/PoseWithHeuristic.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_grasp_calculation_msgs/RankGraspsAction.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

// C++
#include <algorithm>
#include <boost/thread/recursive_mutex.hpp>

class GraspSampler
{
public:
  GraspSampler();

private:
  void rankGraspsObject(const rail_grasp_calculation_msgs::RankGraspsGoalConstPtr &goal);

  void rankGraspsScene(const rail_grasp_calculation_msgs::RankGraspsGoalConstPtr &goal);

  void rankGraspsPOI(const rail_grasp_calculation_msgs::RankGraspsGoalConstPtr &goal);


  void preprocessPoses(geometry_msgs::PoseArray &poses, std::string commonFrame);

  void clusterPoses(std::vector<geometry_msgs::Pose> &graspPoses);


  static double heuristicNormal(geometry_msgs::Pose pose, std::vector<float> &plane);

  static double heuristicAlignment(geometry_msgs::Pose pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


  static void cropToWorkspace(rail_grasp_calculation_msgs::Workspace workspace,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut);

  static void cropAtPoint(geometry_msgs::Point point, float cropSize,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut);

  static Eigen::Quaterniond computePrincipalDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);


  static double squaredDistance(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

  static double squaredDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);

  ros::NodeHandle n, pnh;

  actionlib::SimpleActionServer<rail_grasp_calculation_msgs::RankGraspsAction> rankGraspsObjectServer;
  actionlib::SimpleActionServer<rail_grasp_calculation_msgs::RankGraspsAction> rankGraspsSceneServer;
  actionlib::SimpleActionServer<rail_grasp_calculation_msgs::RankGraspsAction> rankGraspsPOIServer;

  boost::recursive_mutex cloudMutex;

  double neighborhoodRadius;
  double orientationThreshold;
  float localSize;
  int clusterSize;
  bool removeTable;
  sensor_msgs::PointCloud2 cloud;
  tf::TransformBroadcaster tfBroadcaster;
  tf::TransformListener tfListener;
};

#endif
