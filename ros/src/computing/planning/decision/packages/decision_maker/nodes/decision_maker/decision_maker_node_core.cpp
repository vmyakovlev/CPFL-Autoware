#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <stdio.h>

// lib
#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

#include <decision_maker_node.hpp>
//#include <vector_map/vector_map.h>

#include <autoware_msgs/lane.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace decision_maker {
void DecisionMakerNode::tryNextState(const std::string &key) {
  fprintf(stderr, "DEBUG_INFO: %s: %s\n", __func__, key.c_str());
  ctx->nextState(key);
#if 0
  std_msgs::String msg;
  msg.data = Pubs[""].publish
#endif
}

void DecisionMakerNode::update(void) {
  update_msgs();
  if (ctx)
    ctx->onUpdate();
}

void DecisionMakerNode::run(void) {
  ros::Rate loop_rate(5);

  while (ros::ok()) {
    update();
    if (enableDisplayMarker)
      displayMarker();

    loop_rate.sleep();
  }
}
}
