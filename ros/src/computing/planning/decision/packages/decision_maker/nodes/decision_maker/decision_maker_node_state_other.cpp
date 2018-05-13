#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::entryWaitMissionOrderState(const std::string &state_name, int status)
{
  publishOperatorHelpMessage("Please load mission order (waypoints).");
  if (!isSubscriberRegistered("lane_waypoints_array"))
  {
    Subs["lane_waypoints_array"] =
        nh_.subscribe(TPNAME_BASED_LANE_WAYPOINTS_ARRAY, 100, &DecisionMakerNode::callbackFromLaneWaypoint, this);
  }
}

void DecisionMakerNode::updateWaitMissionOrderState(const std::string &state_name, int status)
{
  if (isEventFlagTrue("received_based_lane_waypoint"))
  {
    setEventFlag("received_based_lane_waypoint", false);
    tryNextState("received_mission_order");
  }
}
void DecisionMakerNode::exitWaitMissionOrderState(const std::string &state_name, int status)
{
}

void DecisionMakerNode::entryMissionCheckState(const std::string &state_name, int status)
{
  publishOperatorHelpMessage("Received mission, DM is checking this mission, please wait.");
  current_status_.using_lane_array = current_status_.based_lane_array;
  Pubs["lane_waypoints_array"].publish(current_status_.using_lane_array);
  if (!isSubscriberRegistered("final_waypoints"))
  {
    Subs["final_waypoints"] =
        nh_.subscribe("final_waypoints", 100, &DecisionMakerNode::callbackFromFinalWaypoint, this);
  }
}
void DecisionMakerNode::updateMissionCheckState(const std::string &state_name, int status)
{
  if (isEventFlagTrue("received_finalwaypoints") && current_status_.closest_waypoint != -1)
  {
    tryNextState("mission_is_compatible");
  }
}

void DecisionMakerNode::updateMissionAbortedState(const std::string &state_name, int status)
{
  tryNextState("goto_wait_order");
}

void DecisionMakerNode::entryDriveReadyState(const std::string &state_name, int status)
{
  publishOperatorHelpMessage("Please order to engage");
}

void DecisionMakerNode::updateDriveReadyState(const std::string &state_name, int status)
{
  const bool start_flag = false;
  if (start_flag /*isEventFlagTrue("")*/)
  {
    tryNextState("engage");
  }
}

void DecisionMakerNode::updateMissionCompleteState(const std::string &state_name, int status)
{
  sleep(1);
  tryNextState("goto_wait_order");
}
}
