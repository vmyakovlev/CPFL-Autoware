
#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::publishLampCmd(const E_Lamp &status)
{
  autoware_msgs::lamp_cmd lamp_msg;

  switch (status)
  {
    case E_Lamp::LAMP_LEFT:
      lamp_msg.l = LAMP_ON;
      lamp_msg.r = LAMP_OFF;
      break;
    case E_Lamp::LAMP_RIGHT:
      lamp_msg.l = LAMP_OFF;
      lamp_msg.r = LAMP_ON;
      break;
    case E_Lamp::LAMP_HAZARD:
      lamp_msg.l = LAMP_ON;
      lamp_msg.r = LAMP_ON;
      break;
    case E_Lamp::LAMP_EMPTY:
    default:
      lamp_msg.l = LAMP_OFF;
      lamp_msg.r = LAMP_OFF;
      break;
  }
  Pubs["lamp_cmd"].publish(lamp_msg);
}

jsk_rviz_plugins::OverlayText createOverlayText(const std::string &data, const int column)
{
  jsk_rviz_plugins::OverlayText ret;

  // message setup
  ret.width = 500;
  ret.height = 500;
  ret.top = 10 + (column * 500);
  ret.left = 10;
  ret.bg_color.r = 0;
  ret.bg_color.g = 0;
  ret.bg_color.b = 0;
  ret.bg_color.a = 0.8;

  ret.line_width = 2;
  ret.text_size = 9;
  ret.font = "DejaVu Sans Mono";
  ret.fg_color.r = 1.0;
  ret.fg_color.g = 1.0;
  ret.fg_color.b = 0.5;
  ret.fg_color.a = 0.9;

  ret.text = data;

  return ret;
}

void DecisionMakerNode::publishOperatorHelpMessage(const std::string &message)
{
  static std::vector<std::string> msg_log;
  static const size_t log_size = 10;

  msg_log.push_back(message);

  if (msg_log.size() >= log_size)
  {
    msg_log.erase(msg_log.begin());
  }

  std::string joined_msg;
  for (const auto &i : msg_log)
  {
    joined_msg += "> " + i + "\n";
  }
  Pubs["operator_help_text"].publish(createOverlayText(joined_msg, 0));
}

void DecisionMakerNode::update_pubsub(void)
{
  // if state machine require to re-subscribe topic,
  // this function will re-definition subscriber.
}

int DecisionMakerNode::createCrossRoadAreaMarker(visualization_msgs::Marker &crossroad_marker, double scale)
{
  crossroad_marker.header.frame_id = "/map";
  crossroad_marker.header.stamp = ros::Time();
  crossroad_marker.id = 1;
  crossroad_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crossroad_marker.action = visualization_msgs::Marker::ADD;
  crossroad_marker.ns = "crossroad";

  crossroad_marker.scale.x = scale;
  crossroad_marker.scale.y = scale;
  crossroad_marker.scale.z = 0.5;
  crossroad_marker.color.a = 0.15;
  crossroad_marker.color.r = 1.0;
  crossroad_marker.color.g = 0.0;
  crossroad_marker.color.b = 0.0;
  crossroad_marker.frame_locked = true;
  crossroad_marker.lifetime = ros::Duration(0.3);

  return 0;
}

void DecisionMakerNode::displayMarker(void)
{
#if 0
  // vector_map init
  // parse vectormap
  jsk_recognition_msgs::BoundingBoxArray bbox_array;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker crossroad_marker;
  visualization_msgs::Marker stopline_target_marker;
  visualization_msgs::Marker inside_marker;
  visualization_msgs::Marker inside_line_marker;
  visualization_msgs::Marker detection_area_marker;

  double scale = 3.0;
  createCrossRoadAreaMarker(crossroad_marker, scale);

  detection_area_marker = crossroad_marker;
  detection_area_marker.header.frame_id = param_baselink_tf_;
  detection_area_marker.scale.x = (detectionArea_.x1 - detectionArea_.x2) * param_detection_area_rate_;
  detection_area_marker.scale.y = (detectionArea_.y1 - detectionArea_.y2) * param_detection_area_rate_;
  detection_area_marker.color.r = foundOtherVehicleForIntersectionStop_ ? 0.0 : 1.0;
  detection_area_marker.pose.position.x = detection_area_marker.scale.x / 2;
  detection_area_marker.color.g = 1;
  detection_area_marker.color.b = 0.3;
  detection_area_marker.color.a = 0.3;
  detection_area_marker.type = visualization_msgs::Marker::CUBE;
  detection_area_marker.lifetime = ros::Duration();

  stopline_target_marker = crossroad_marker;
  stopline_target_marker.type = visualization_msgs::Marker::SPHERE;
  stopline_target_marker.scale.x = 1;
  stopline_target_marker.scale.y = 1;
  stopline_target_marker.scale.z = 1;
  stopline_target_marker.color.a = 1;
  stopline_target_marker.color.r = 1.0;
  stopline_target_marker.color.g = 0.0;
  stopline_target_marker.color.b = 0.0;
  stopline_target_marker.ns = "stopline_target";
  stopline_target_marker.lifetime = ros::Duration();
  stopline_target_marker.pose.position.x = CurrentStoplineTarget_.pose.pose.position.x;
  stopline_target_marker.pose.position.y = CurrentStoplineTarget_.pose.pose.position.y;
  stopline_target_marker.pose.position.z = CurrentStoplineTarget_.pose.pose.position.z;

  inside_marker = crossroad_marker;
  inside_marker.scale.x = scale / 3;
  inside_marker.scale.y = scale / 3;
  inside_marker.scale.z = 0.4;
  inside_marker.scale.z = 0.5;
  inside_marker.color.a = 0.5;
  inside_marker.color.r = 1.0;
  inside_marker.color.g = 0.0;
  inside_marker.color.b = 0.0;
  inside_marker.ns = "inside";
  inside_marker.lifetime = ros::Duration();

  inside_marker = crossroad_marker;
  inside_marker.scale.x = scale / 3;
  inside_marker.scale.y = scale / 3;
  inside_marker.scale.z = 0.4;
  inside_marker.scale.z = 0.5;
  inside_marker.color.a = 0.5;
  inside_marker.color.r = 1.0;
  inside_marker.color.g = 0.0;
  inside_marker.color.b = 0.0;
  inside_marker.ns = "inside";
  inside_marker.lifetime = ros::Duration();

  bbox_array.header = crossroad_marker.header;
  inside_marker.points.clear();

  inside_line_marker = inside_marker;
  inside_line_marker.type = visualization_msgs::Marker::LINE_STRIP;

  for (auto &area : intersects)
  {
    area.bbox.header = crossroad_marker.header;
    bbox_array.boxes.push_back(area.bbox);
    for (const auto &p : area.insideWaypoint_points)
    {
      inside_marker.points.push_back(p);
    }

    for (const auto &lane : area.insideLanes)
    {
      inside_line_marker.points.clear();
      int id = inside_line_marker.id;
      inside_line_marker.id += 1;
      inside_marker.scale.x = scale / 3;
      inside_marker.scale.y = scale / 3;
      inside_line_marker.color.r = std::fmod(0.12345 * (id), 1.0);
      inside_line_marker.color.g = std::fmod(0.32345 * (5 - (id % 5)), 1.0);
      inside_line_marker.color.b = std::fmod(0.52345 * (10 - (id % 10)), 1.0);
      for (const auto &wp : lane.waypoints)
      {
        inside_line_marker.points.push_back(wp.pose.pose.position);
      }
      marker_array.markers.push_back(inside_line_marker);
    }
  }
  inside_line_marker.scale.x = 0.2;  // 0.3;
  inside_line_marker.scale.y = 0.2;  // 0.3;
  inside_line_marker.color.r = 0;
  inside_line_marker.color.g = 1;
  inside_line_marker.color.b = 0.3;
  inside_line_marker.color.a = 1;
  inside_line_marker.ns = "shiftline";
  for (const auto &lane : current_controlled_lane_array_.lanes)
  {
    inside_line_marker.points.clear();
    for (size_t idx = 0; idx < lane.waypoints.size(); idx++)
    {
      inside_line_marker.id += 1;

      geometry_msgs::Pose shift_p = lane.waypoints.at(idx).pose.pose;

      double current_angle = amathutils::getPoseYawAngle(shift_p);

      shift_p.position.x -= param_shift_width_ * cos(current_angle + M_PI / 2);
      shift_p.position.y -= param_shift_width_ * sin(current_angle + M_PI / 2);

      inside_line_marker.points.push_back(shift_p.position);
    }
    marker_array.markers.push_back(inside_line_marker);
  }
  Pubs["detection_area"].publish(detection_area_marker);
  Pubs["crossroad_bbox"].publish(bbox_array);
  Pubs["crossroad_marker"].publish(marker_array);
  Pubs["stopline_target"].publish(stopline_target_marker);
  bbox_array.boxes.clear();
  // Pubs["crossroad_inside_marker"].publish(inside_marker);
  Pubs["crossroad_inside_marker"].publish(inside_line_marker);
#endif
}

void DecisionMakerNode::update_msgs(void)
{
#if 1
  if (ctx)
  {
    static std_msgs::String state_msg;
    state_msg.data = ctx->getStateText();
    Pubs["state"].publish(state_msg);
    Pubs["state_overlay"].publish(createOverlayText(state_msg.data, 1));

    static std_msgs::String transition_msg;
    transition_msg.data = ctx->getAvailableTransition();

    Pubs["available_transition"].publish(transition_msg);
  }
  else
  {
    std::cerr << "ctx is not found " << std::endl;
  }
#endif
}

std::string DecisionMakerNode::createStateMessageText()
{
  // return ctx->createStateMessageText();
}

void DecisionMakerNode::publishLightColor(int status)
{
  autoware_msgs::traffic_light msg;
  msg.traffic_light = status;
  Pubs["light_color"].publish(msg);
}

void DecisionMakerNode::publishStoplineWaypointIdx(int wp_idx)
{
  std_msgs::Int32 msg;
  msg.data = wp_idx;
  Pubs["state/stopline_wpidx"].publish(msg);
}

void DecisionMakerNode::publishToVelocityArray()
{
  int count = 0;
  std_msgs::Float64MultiArray msg;

  for (const auto &i : current_status_.finalwaypoints.waypoints)
  {
    msg.data.push_back(amathutils::mps2kmph(i.twist.twist.linear.x));
    if (++count >= 10)
      break;
  }
  Pubs["target_velocity_array"].publish(msg);
}
}
