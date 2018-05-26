#include "lidar_naive_l_shape_detect.h"
// #include <iostream>
// #include <ros/ros.h>
//
// #include "autoware_msgs/CloudCluster.h"
// #include "autoware_msgs/CloudClusterArray.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "lidar_naive_l_shape_detect");
  ClusterFilter app;
  ros::spin();

  return 0;
}
