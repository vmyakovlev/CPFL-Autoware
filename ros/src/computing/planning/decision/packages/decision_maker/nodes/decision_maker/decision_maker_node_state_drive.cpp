#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::entryDriveState(const std::string &state_name, int status)
{
  publishOperatorHelpMessage("< Engaged");
}
void DecisionMakerNode::updateDriveState(const std::string &state_name, int status)
{
  if (isArrivedGoal())
  {
    tryNextState("arrived_goal");
    return;
  }

  if (isVehicleOnLaneArea())
  {
    tryNextState("on_lane_area");
  }
  else
  {
    tryNextState("on_free_area");
  }
}

uint8_t DecisionMakerNode::getSteeringStateFromWaypoint(void)
{
  static const double distance_to_target = 30.0;  // 30m in front of intersection: Follow the Japanese Law
  static const size_t ignore_idx = 0;

  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;
  uint8_t state = 0;

  if (ignore_idx > current_status_.finalwaypoints.waypoints.size())
  {
    return 0;
  }

  for (auto idx = 0; idx < current_status_.finalwaypoints.waypoints.size() - 1; idx++)
  {
    distance += amathutils::find_distance(prev_pose, current_status_.finalwaypoints.waypoints.at(idx).pose.pose);

    state = current_status_.finalwaypoints.waypoints.at(idx).wpstate.steering_state;

    if (state && (state != autoware_msgs::WaypointState::STR_STRAIGHT || distance >= distance_to_target))
    {
      break;
    }
    prev_pose = current_status_.finalwaypoints.waypoints.at(idx).pose.pose;
  }
  return state;
}
uint8_t DecisionMakerNode::getStopSignStateFromWaypoint(void)
{
  static const size_t ignore_idx = 0;
  static const double mu = 0.7;  // dry ground/ asphalt/ normal tire
  static const double g = 9.8;
  static const double reaction_time = 0.3;  // system delay(sec)
  const double velocity = amathutils::kmph2mps(current_status_.velocity);
  const double free_running_distance = reaction_time * velocity;
  const double braking_distance = velocity * velocity / (2 * g * mu);
  const double distance_to_target = (free_running_distance + braking_distance) * 2 /* safety margin*/;

  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;
  uint8_t state = 0;

  if (ignore_idx > current_status_.finalwaypoints.waypoints.size())
  {
    return 0;
  }

  for (auto idx = 0; idx < current_status_.finalwaypoints.waypoints.size() - 1; idx++)
  {
    distance += amathutils::find_distance(prev_pose, current_status_.finalwaypoints.waypoints.at(idx).pose.pose);
    state = current_status_.finalwaypoints.waypoints.at(idx).wpstate.stop_state;

    if (state && distance >= distance_to_target)
    {
      break;
    }
    prev_pose = current_status_.finalwaypoints.waypoints.at(idx).pose.pose;
  }
  return state;
}
void DecisionMakerNode::updateLaneAreaState(const std::string &state_name, int status)
{
  if (current_status_.finalwaypoints.waypoints.empty())
  {
    ROS_WARN("\"/final_waypoints.\" is not contain waypoints");
    return;
  }
  switch (getSteeringStateFromWaypoint())
  {
    case autoware_msgs::WaypointState::STR_LEFT:
      tryNextState("on_left_turn");
      break;
    case autoware_msgs::WaypointState::STR_RIGHT:
      tryNextState("on_right_turn");
      break;
    case autoware_msgs::WaypointState::STR_STRAIGHT:
      tryNextState("on_straight");
      break;
    default:
      break;
  }
}

void DecisionMakerNode::updateFreeAreaState(const std::string &state_name, int status)
{
}

void DecisionMakerNode::entryTurnState(const std::string &state_name, int status)
{
  tryNextState("clear");
}

void DecisionMakerNode::updateLeftTurnState(const std::string &state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_LEFT);
}

void DecisionMakerNode::updateRightTurnState(const std::string &state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_RIGHT);
}

void DecisionMakerNode::updateGoState(const std::string &state_name, int status)
{
  if (getStopSignStateFromWaypoint())
  {
    tryNextState("found_stopline");
  }
}

void DecisionMakerNode::updateWaitState(const std::string &state_name, int status)
{
  /* clear,*/
}

void DecisionMakerNode::updateStoplineState(const std::string &state_name, int status)
{
  /* clear found_risk*/
}
}
