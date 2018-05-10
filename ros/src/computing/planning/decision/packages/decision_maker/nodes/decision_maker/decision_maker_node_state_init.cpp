#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::entryInitState(const std::string &state_name, int status)
{
  ROS_INFO("Hello Autoware World");
  initROS();

  ROS_INFO("ROS is ready");

  ctx->nextState("init_start");
}

void DecisionMakerNode::updateInitState(const std::string &state_name, int status)
{
  static bool is_first_callback = true;

  if (!is_first_callback)
  {
    return;
  }
  ROS_INFO("Autoware is initializing now");
  is_first_callback = false;
}

void DecisionMakerNode::entrySensorInitState(const std::string &state_name, int status)
{
  Subs["filtered_points"] = nh_.subscribe("filtered_points", 1, &DecisionMakerNode::callbackFromFilteredPoints, this);
}

void DecisionMakerNode::updateSensorInitState(const std::string &state_name, int status)
{
  const double timeout = 1;

  std::vector<std::string> node_list;

  ros::master::getNodes(node_list);
  for (const auto &i : node_list)
  {
    if ("/wf_simulator" == i)
    {
      EventFlags["on_mode_sim"] = true;
    }
  }

  if (isEventFlagTrue("on_mode_sim"))
  {
    ROS_INFO("DecisionMaker is in simulation mode");
    ctx->nextState("sensor_is_ready");
  }
  else if (waitForEvent("received_pointcloud_for_NDT", true, timeout) == true)
  {
    ctx->nextState("sensor_is_ready");
  }
  ROS_INFO("DecisionMaker is waiting filtered_point for NDT");
}

void DecisionMakerNode::updateMapInitState(const std::string &state_name, int status)
{
  bool vmap_loaded = false;
  do
  {
    g_vmap.subscribe(nh_,
                     Category::POINT | Category::LINE | Category::VECTOR | Category::AREA |
                         Category::POLE |  // basic class
                         Category::DTLANE | Category::STOP_LINE | Category::ROAD_SIGN | Category::CROSS_ROAD,
                     ros::Duration(5.0));

    vmap_loaded =
        g_vmap.hasSubscribed(Category::POINT | Category::LINE | Category::VECTOR | Category::AREA | Category::POLE |
                             Category::DTLANE | Category::STOP_LINE | Category::ROAD_SIGN | Category::CROSS_ROAD);
    if (!vmap_loaded)
    {
      ROS_WARN("Necessary vectormap have not been loaded");
      ROS_WARN("DecisionMaker keeps on waiting until it loads");
    }

  } while (!vmap_loaded);
  initVectorMap();

  ctx->nextState("map_is_ready");
}

void DecisionMakerNode::entryLocalizationInitState(const std::string &state_name, int status)
{
  Subs["current_pose"] = nh_.subscribe("current_pose", 5, &DecisionMakerNode::callbackFromCurrentPose, this);
}

void DecisionMakerNode::updateLocalizationInitState(const std::string &state_name, int status)
{
  if (isLocalizationConvergence(current_status_.pose.position))
  {
    ctx->nextState("localization_is_ready");
  }
}

void DecisionMakerNode::entryPlanningInitState(const std::string &state_name, int status)
{
  Subs["closest_waypoint"] =
      nh_.subscribe("closest_waypoint", 1, &DecisionMakerNode::callbackFromClosestWaypoint, this);
  ctx->nextState("planning_is_ready");
}

void DecisionMakerNode::entryVehicleInitState(const std::string &state_name, int status)
{
  ctx->nextState("vehicle_is_ready");
}

void DecisionMakerNode::entryVehicleReadyState(const std::string &state_name, int status)
{
}

void DecisionMakerNode::updateVehicleReadyState(const std::string &state_name, int status)
{
  ctx->nextState("going_to_wait_mission_order");
}

void DecisionMakerNode::entryWaitMissionOrderState(const std::string &state_name, int status)
{
  if (!isSubscribed("lane_waypoints_array"))
  {
    Subs["lane_waypoints_array"] =
        nh_.subscribe(TPNAME_BASED_LANE_WAYPOINTS_ARRAY, 100, &DecisionMakerNode::callbackFromLaneWaypoint, this);
  }
}

void DecisionMakerNode::updateWaitMissionOrderState(const std::string &state_name, int status)
{
  if (isEventFlagTrue("received_based_lane_waypoint"))
  {
    EventFlags["received_based_lane_waypoint"] = false;
    ctx->nextState("received_mission_order");
  }
}
void DecisionMakerNode::exitWaitMissionOrderState(const std::string &state_name, int status)
{
}

void DecisionMakerNode::entryMissionCheckState(const std::string &state_name, int status)
{
  current_status_.using_lane_array = current_status_.based_lane_array;
  Pubs["lane_waypoints_array"].publish(current_status_.using_lane_array);
  if (!isSubscribed("final_waypoints"))
  {
    Subs["final_waypoints"] =
        nh_.subscribe("final_waypoints", 100, &DecisionMakerNode::callbackFromFinalWaypoint, this);
  }
}
void DecisionMakerNode::updateMissionCheckState(const std::string &state_name, int status)
{
  if (isEventFlagTrue("received_finalwaypoints"))
  {
    ctx->nextState("mission_is_compatible");
  }
}

void DecisionMakerNode::entryDriveReadyState(const std::string &state_name, int status)
{
}

void DecisionMakerNode::updateDriveReadyState(const std::string &state_name, int status)
{
  const bool start_flag = true;
  if (start_flag /*isEventFlagTrue("")*/)
  {
    ctx->nextState("engage");
  }
}

void DecisionMakerNode::updateMissionCompleteState(const std::string &state_name, int status)
{
  sleep(1);
  ctx->nextState("goto_wait_order");
}
}
