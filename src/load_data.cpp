/*
 * @Copyright: 2020, Authors. All rights reserved.
 * @Description: Load data from rosbag
 *               Transform to the world frame through the given initial parameters
 * @Author: v_hezhenpeng
 * @Date: 2020-8-18
 */


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <iostream>
#include <string>
#include <vector>

#include "common.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

std::vector<PointCloud> lidar_datas;
std::vector<PointCloud> image_cloud_datas;
std::string message_title = "/livox/lidar_all";
int num_lidar_points;  // the maximum accept lidar frames
std::string input_bag_path, output_path, intrinsic_path, extrinsic_path;
Eigen::Matrix4d intrinsic, extrinsic_camera, extrinsic_lidar;

/*
 * @brief: Load ROS parameters and calculate the extrinsic
 */
void getParameters() {
  std::cout << "Get the parameters from the launch file" << std::endl;
  if (!ros::param::get("input_bag_path", input_bag_path)) {
    exit(1);
  }
  if (!ros::param::get("output_path", output_path)) {
    exit(1);
  }
  if (!ros::param::get("num_lidar_points", num_lidar_points)) {
    exit(1);
  }
  if (!ros::param::get("extrinsic_path", extrinsic_path)) {
    exit(1);
  }
  if (!ros::param::get("message_title", message_title)) {
    exit(1);
  }  
  if (!loadMatrix(extrinsic_path + "/lidar_extrinsic.txt", extrinsic_lidar)) {
    exit(1);
  }
}

/*
 * @brief: Load lidar and depth camera point cloud
 */
void loadPointcloudFromROSBag(const std::string& bag_path) {
  ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return;
  }

  std::vector<std::string> topics;
  topics.push_back(message_title);  // message title
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (const rosbag::MessageInstance& m : view) {
    if (lidar_datas.size() < num_lidar_points) {
      sensor_msgs::PointCloud2ConstPtr lidar_m = m.instantiate<sensor_msgs::PointCloud2>();
      if (lidar_m == NULL) continue;
      PointCloud cloud;
      pcl::fromROSMsg(*lidar_m, cloud);
      // pcl::transformPointCloud(cloud, cloud, extrinsic_lidar);
      lidar_datas.push_back(cloud);
    }
    if (lidar_datas.size() >= num_lidar_points) {
      ROS_INFO("Finished load data");
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "load_data");
  getParameters();
  loadPointcloudFromROSBag(input_bag_path);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_all(new pcl::PointCloud<pcl::PointXYZ>());
  for (int i=0; i < lidar_datas.size(); i++) {
    *point_all += lidar_datas[i];
  }
  pcl::io::savePCDFileBinary(output_path + "point_all.pcd", *point_all);
  ROS_INFO("Save cloud");
  return 0;
}
