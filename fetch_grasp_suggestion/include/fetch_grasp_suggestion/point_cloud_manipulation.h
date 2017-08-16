#ifndef FETCH_GRASP_SUGGESTION_POINT_CLOUD_MANIPULATION_H
#define FETCH_GRASP_SUGGESTION_POINT_CLOUD_MANIPULATION_H

// C++
#include <string>
#include <vector>

// ROS
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

class PointCloudManipulation
{

public:

  static void transformPointCloud(const sensor_msgs::PointCloud2 &cloud_in, sensor_msgs::PointCloud2 &cloud_out,
      std::string frame, tf::TransformListener &tf_listener);

  static void transformPointCloud(const sensor_msgs::PointCloud2 &cloud_in,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, std::string frame, tf::TransformListener &tf_listener);

  static void transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
      sensor_msgs::PointCloud2 &cloud_out, std::string frame, tf::TransformListener &tf_listener);

  static void transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out, std::string frame, tf::TransformListener &tf_listener);

  static void fromSensorMsgs(const sensor_msgs::PointCloud2 &cloud_in,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

  static void toSensorMsgs(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
      sensor_msgs::PointCloud2 &cloud_out);
};

#endif  // FETCH_GRASP_SUGGESTION_POINT_CLOUD_MANIPULATION_H
