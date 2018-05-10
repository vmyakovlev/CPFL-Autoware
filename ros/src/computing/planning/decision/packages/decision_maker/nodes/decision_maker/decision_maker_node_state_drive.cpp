#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::updateDriveState(const std::string &state_name, int status)
{
  if (isArrivedGoal())
  {
    ctx->nextState("arrived_goal");
    return;
  }

  if (isVehicleOnLaneArea())
  {
    ctx->nextState("onLaneArea");
  }
  else
  {
    ctx->nextState("onFreeArea");
  }
}
void DecisionMakerNode::updateLaneAreaState(const std::string &state_name, int status)
{
  if (isArrivedGoal())
  {
    ctx->nextState("arrived_goal");
    return;
  }

  if (isVehicleOnLaneArea())
  {
    ctx->nextState("onLaneArea");
  }
  else
  {
    ctx->nextState("onFreeArea");
  }
}

void DecisionMakerNode::updateFreeAreaState(const std::string &state_name, int status)
{
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
}

void DecisionMakerNode::updateWaitState(const std::string &state_name, int status)
{
}

void DecisionMakerNode::updateStoplineState(const std::string &state_name, int status)
{
}
}
