#include "naive_L_shape.h"
#include <iostream>
#include <ros/ros.h>

#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "cluster_filter");
  ClusterFilter app;
  ros::spin();

  return 0;
}
