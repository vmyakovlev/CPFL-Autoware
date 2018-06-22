/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "pure_pursuit_core.h"

namespace waypoint_follower
{
// Constructor
PurePursuitNode::PurePursuitNode()
  : private_nh_("~")
  , pp_(100)
  , LOOP_RATE_(30)
  , is_waypoint_set_(false)
  , is_pose_set_(false)
  , is_velocity_set_(false)
  , is_config_set_(false)
  , current_linear_velocity_(0)
  , command_linear_velocity_(0)
  , param_flag_(-1)
  , const_lookahead_distance_(4.0)
  , const_velocity_(5.0)
  , lookahead_distance_ratio_(2.0)
  , minimum_lookahead_distance_(6.0)
  , omega_sigma_(5.0)
  , alpha_sigma_(5.0)
  , dist_sigma_(20.0)
  , dt_(0.1)
{
  initForROS();

  // initialize for PurePursuit
  for(int i = 0; i < pp_.size(); i++)
    pp_[i].setLinearInterpolationParameter(is_linear_interpolation_);
}

// Destructor
PurePursuitNode::~PurePursuitNode()
{
}

void PurePursuitNode::initForROS()
{
  // ros parameter settings
  private_nh_.param("is_linear_interpolation", is_linear_interpolation_, bool(true));
  // ROS_INFO_STREAM("is_linear_interpolation : " << is_linear_interpolation_);
  private_nh_.param("publishes_for_steering_robot", publishes_for_steering_robot_, bool(false));
  private_nh_.param("vehicle_info/wheel_base", wheel_base_, double(2.7));
  private_nh_.param("eval_omega", omega_sigma_, double(5.0));
  private_nh_.param("eval_alpha", alpha_sigma_, double(5.0));
  private_nh_.param("eval_dist", dist_sigma_, double(20.0));

  // setup subscriber
  sub1_ = nh_.subscribe("final_waypoints", 10, &PurePursuitNode::callbackFromWayPoints, this);
  sub2_ = nh_.subscribe("current_pose", 10, &PurePursuitNode::callbackFromCurrentPose, this);
  sub3_ = nh_.subscribe("config/waypoint_follower", 10, &PurePursuitNode::callbackFromConfig, this);
  sub4_ = nh_.subscribe("current_velocity", 10, &PurePursuitNode::callbackFromCurrentVelocity, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  pub2_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("ctrl_cmd", 10);
  pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);  // debug tool
  pub15_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
  pub16_ = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
  // pub7_ = nh.advertise<std_msgs::Bool>("wf_stat", 0);
}

void PurePursuitNode::run()
{
  ROS_INFO_STREAM("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!is_pose_set_ || !is_waypoint_set_ || !is_velocity_set_ || !is_config_set_)
    {
      ROS_WARN("Necessary topics are not subscribed yet ... ");
      loop_rate.sleep();
      continue;
    }

    double ld = computeLookaheadDistance();
    for (int i = 0; i < pp_.size(); i++)
    {
      double ratio = (i + 1) / (double)pp_.size();
      pp_[i].setLookaheadDistance(minimum_lookahead_distance_ + (ld - minimum_lookahead_distance_) * ratio);
      pp_[i].setMinimumLookaheadDistance(minimum_lookahead_distance_);
    }

    double kappa = 1e-9;
    int id = 0;
    bool can_get_curvature = computeBestCurvature(&kappa, &id);

    publishTwistStamped(can_get_curvature, kappa);
    publishControlCommandStamped(can_get_curvature, kappa);

    // for visualization with Rviz
    pub11_.publish(displayNextWaypoint(pp_[id].getPoseOfNextWaypoint()));
    pub13_.publish(displaySearchRadius(pp_[id].getCurrentPose().position, pp_[id].getLookaheadDistance()));
    pub12_.publish(displayNextTarget(pp_[id].getPoseOfNextTarget()));
    pub15_.publish(displayTrajectoryCircle(
        waypoint_follower::generateTrajectoryCircle(pp_[id].getPoseOfNextTarget(), pp_[id].getCurrentPose())));
    std_msgs::Float32 angular_gravity_msg;
    angular_gravity_msg.data = computeAngularGravity(computeCommandVelocity(), kappa);
    pub16_.publish(angular_gravity_msg);

    is_pose_set_ = false;
    is_velocity_set_ = false;
    is_waypoint_set_ = false;

    loop_rate.sleep();
  }
}

void PurePursuitNode::publishTwistStamped(const bool &can_get_curvature, const double &kappa) const
{
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
  ts.twist.angular.z = can_get_curvature ? kappa * ts.twist.linear.x : 0;
  pub1_.publish(ts);
}

void PurePursuitNode::publishControlCommandStamped(const bool &can_get_curvature, const double &kappa) const
{
  if (!publishes_for_steering_robot_)
    return;

  autoware_msgs::ControlCommandStamped ccs;
  ccs.header.stamp = ros::Time::now();
  ccs.cmd.linear_velocity = can_get_curvature ? computeCommandVelocity() : 0;
  ccs.cmd.linear_acceleration = can_get_curvature ? computeCommandAccel() : 0;
  ccs.cmd.steering_angle = can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa) : 0;

  pub2_.publish(ccs);
}

double PurePursuitNode::computeLookaheadDistance() const
{
  if (param_flag_ == enumToInteger(Mode::dialog))
    return const_lookahead_distance_;

  double maximum_lookahead_distance = current_linear_velocity_ * 10;
  double ld = current_linear_velocity_ * lookahead_distance_ratio_;

  return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_
        : ld > maximum_lookahead_distance ? maximum_lookahead_distance
        : ld;
}

double PurePursuitNode::computeCommandVelocity() const
{
  if (param_flag_ == enumToInteger(Mode::dialog))
    return kmph2mps(const_velocity_);

  return command_linear_velocity_;
}

double PurePursuitNode::computeCommandAccel() const
{
  const geometry_msgs::Pose current_pose = pp_.back().getCurrentPose();
  const geometry_msgs::Pose target_pose = pp_.back().getCurrentWaypoints().at(1).pose.pose;

  // v^2 - v0^2 = 2ax
  const double x =  std::hypot(current_pose.position.x-target_pose.position.x, current_pose.position.y-target_pose.position.y);
  const double v0 = current_linear_velocity_;
  const double v = computeCommandVelocity();
  const double a = (v*v - v0*v0) / (2*x);
  return a;
}

double PurePursuitNode::computeAngularGravity(double velocity, double kappa) const
{
  const double gravity = 9.80665;
  return (velocity*velocity) / (1.0/kappa*gravity);
}

void PurePursuitNode::callbackFromConfig(const autoware_msgs::ConfigWaypointFollowerConstPtr &config)
{
  param_flag_ = config->param_flag;
  const_lookahead_distance_ = config->lookahead_distance;
  const_velocity_ = config->velocity;
  lookahead_distance_ratio_ = config->lookahead_ratio;
  minimum_lookahead_distance_ = config->minimum_lookahead_distance;
  is_config_set_ = true;
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  for (int i = 0; i < pp_.size(); i++)
    pp_[i].setCurrentPose(msg);
  is_pose_set_ = true;
}

void PurePursuitNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_linear_velocity_ = msg->twist.linear.x;
  if (current_linear_velocity_ > 1e-9)
  {
    current_kappa_ = msg->twist.angular.z / msg->twist.linear.x;
  }
  else
  {
    int sgn = (msg->twist.angular.z > 0.0) ? 1 : -1;
    current_kappa_ = 1e-9 * sgn;
  }
  for (int i = 0; i < pp_.size(); i++)
    pp_[i].setCurrentVelocity(current_linear_velocity_);
  is_velocity_set_ = true;
}

void PurePursuitNode::callbackFromWayPoints(const autoware_msgs::laneConstPtr &msg)
{
  if (!msg->waypoints.empty())
    command_linear_velocity_ = msg->waypoints.at(0).twist.twist.linear.x;
  else
    command_linear_velocity_ = 0;
  current_waypoints_ = msg->waypoints;
  for (int i = 0; i < pp_.size(); i++)
    pp_[i].setCurrentWaypoints(current_waypoints_);
  is_waypoint_set_ = true;
}

double PurePursuitNode::computeEvaluation(double omega, double alpha, double dist) const
{
  const double omega_eval = exp(-omega * omega / (2 * omega_sigma_ * omega_sigma_));
  const double alpha_eval = exp(-alpha * alpha / (2 * alpha_sigma_ * alpha_sigma_));
  const double dist_eval = exp(-dist * dist / (2 * dist_sigma_ * dist_sigma_));
  return omega_eval * alpha_eval * dist_eval;
}

bool PurePursuitNode::computeBestCurvature(double *kappa, int *id)
{
  const double current_theta = convertCurvatureToSteeringAngle(wheel_base_, current_kappa_);
  static double prev_theta = current_theta;
  const double prev_omega = (current_theta - prev_theta) / dt_;
  prev_theta = current_theta;
  double max_eval = 0.0;
  for (int i = 0; i < pp_.size(); i++)
  {
    double kappa_tmp = 1e-9;
    const bool can_get_curvature = pp_[i].canGetCurvature(&kappa_tmp);
    if (!can_get_curvature)
    {
      return false;
    }
    const double angle = convertCurvatureToSteeringAngle(wheel_base_, kappa_tmp);
    const double omega = (angle - current_theta) / dt_;
    const double alpha = (omega - prev_omega) / dt_;
    const double eval = computeEvaluation(omega, alpha, pp_[i].getLookaheadDistance());
    if (max_eval > eval)
    {
      continue;
    }
    max_eval = eval;
    *id = i;
    *kappa = kappa_tmp;
  }
  return true;
}

double convertCurvatureToSteeringAngle(const double &wheel_base, const double &kappa)
{
  return atan(wheel_base * kappa);
}

}  // waypoint_follower
