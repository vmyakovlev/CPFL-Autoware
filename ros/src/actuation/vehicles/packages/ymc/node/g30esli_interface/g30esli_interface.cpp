/*
 *  Copyright (c) 2017, Nagoya University
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

#include <cmath>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include "autoware_msgs/VehicleCmd.h"
#include "g30esli_interface_util.h"
#include "can_utils/cansend.h"
#include "can_utils/cansend_util.h"
#include "can_utils/ymc_can.h"

namespace
{
// ros param
double g_wheel_base;
int g_mode;
std::string g_device;
int g_loop_rate;
int g_stop_time_sec;

ros::NodeHandle *g_nh;

// ros publisher
ros::Publisher g_current_twist_pub;
ros::Publisher g_nextremer_command_pub;
ros::Publisher g_state_command_pub;

// variables
uint16_t g_target_velocity_ui16;
uint16_t g_manual_target_velocity_ui16;
int16_t g_steering_angle_deg_i16;
int16_t g_manual_steering_angle_deg_i16;
double g_current_vel_kmph = 0.0;
double g_target_vel_kmph = 0.0;
bool g_terminate_thread = false;
bool g_automode = false;
unsigned char g_shift = 0;
unsigned char g_brake = 0;
bool g_keep_state = true;
ros::Time g_joy_time;

// cansend tool
mycansend::CanSender g_cansender;

void twist_cmd_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  double target_velocity = msg->twist.linear.x * 3.6; // [m/s] -> [km/h]
  g_target_vel_kmph = target_velocity;
  double target_steering_angle_deg = ymc::computeTargetSteeringAngleDegree(msg->twist.angular.z, msg->twist.linear.x, g_wheel_base);
  target_steering_angle_deg += 8.0;
  // factor
  target_velocity           *= 10.0;
  target_steering_angle_deg *= 10.0;

  g_target_velocity_ui16    = target_velocity;
  g_steering_angle_deg_i16  = target_steering_angle_deg * -1.0;
}

void vehicle_cmd_callback(const autoware_msgs::VehicleCmdConstPtr &msg)
{
  double target_velocity = msg->twist_cmd.twist.linear.x * 3.6; // [m/s] -> [km/h]
  g_target_vel_kmph = target_velocity;
  double target_steering_angle_deg = ymc::computeTargetSteeringAngleDegree(msg->twist_cmd.twist.angular.z, msg->twist_cmd.twist.linear.x, g_wheel_base);
  target_steering_angle_deg += 8.0;
  // factor
  target_velocity           *= 10.0;
  target_steering_angle_deg *= 10.0;

  g_target_velocity_ui16    = target_velocity;
  g_steering_angle_deg_i16  = target_steering_angle_deg * -1.0;

  g_brake = (msg->emergency == 1) ? 2 : 0;
}

void current_vel_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  g_current_vel_kmph = msg->twist.linear.x * 3.6;
}

void current_joy_callback(const sensor_msgs::JoyConstPtr &msg)
{
  g_joy_time = ros::Time::now();

  if (msg->buttons[0] == 1 || msg->axes[1] != 0.0 || msg->axes[2] != 0.0)
  {
    std::cout << "manual mode" << std::endl;
    g_automode = false;
  }

  double target_velocity = 0.0;
  if (msg->buttons[1] == 1)
  {
    double r2 = (-msg->axes[4]+1.0)/2.0;  // R2
    target_velocity = 16.0 * r2 + 3.0;
  }

  double l2 = (-msg->axes[3]+1.0)/2.0;  // L2
  double steering_angle_ratio_deg = 20.0 + 17.0 * l2;
  double target_steering_angle_deg = steering_angle_ratio_deg * msg->axes[0]; // L horizontal
  target_steering_angle_deg += 8.0;

  g_brake = 0;
  if (msg->buttons[0] == 1) // square
  {
    g_brake = 1;
    target_velocity = 0.0;
  }

  if (msg->buttons[3] == 1) // square
  {
    g_brake = 2;
    target_velocity = 0.0;
  }

  g_shift = msg->buttons[5];  // R1

  if (msg->buttons[8] == 1) // share
  {
    g_mode = 3;
  }

  if (msg->buttons[9] == 1) // option
  {
    g_mode = 8;
  }

  if (msg->axes[10] == 1)
  {
  	std_msgs::Int32 msg;
  	msg.data = 13;	// keep
  	g_state_command_pub.publish(msg);
  	g_keep_state = true;
  }
  if (msg->axes[10] == -1)
  {
  	std_msgs::Int32 msg;
  	msg.data = 14;	// stop
  	g_state_command_pub.publish(msg);
  	g_keep_state = false;
  }

  // factor
  target_velocity           *= 10.0;
  target_steering_angle_deg *= 10.0;

  g_manual_target_velocity_ui16    = target_velocity;
  g_manual_steering_angle_deg_i16  = target_steering_angle_deg * -1.0;

  if (msg->buttons[12] == 1)
  {
    std::cout << "automode" << std::endl;
    g_automode = true;
    // g_shift = 0;
  }
}

void joy_check_callback(const ros::TimerEvent& e)
{
  if (std::abs((ros::Time::now() - g_joy_time).toSec()) > 2.0)
  {
    std::cout << "error joystick !!" << std::endl;
    g_automode = false;
    g_brake = 1;
    g_manual_target_velocity_ui16 = 0;
  }
}

// receive input from keyboard
// change the mode to manual mode or auto drive mode
void changeMode()
{
  while (!g_terminate_thread)
  {
    if (ymc::kbhit())
    {
      char c = getchar();

      if (c == ' ')
        g_mode = 3;

      if (c == 's')
        g_mode = 8;
    }
    usleep(20000); // sleep 20msec
  }
}

// read can data from the vehicle
void readCanData(FILE* fp)
{
  char buf[64];
  while (fgets(buf, sizeof(buf), fp) != NULL && !g_terminate_thread)
  {
    std::string raw_data = std::string(buf);
    // std::cout << "received data: " << raw_data << std::endl;

    // check if the data is valid
    if (raw_data.size() > 0)
    {
      // split data to strings
      // data format is like this
      // can0  200   [8]  08 00 00 00 01 00 01 29
      std::vector<std::string> parsed_data = ymc::splitString(raw_data);

      // delete first 3 elements
      std::vector<std::string> data = parsed_data;
      data.erase(data.begin(), data.begin() + 3);

      int id = std::stoi(parsed_data.at(1), nullptr, 10);
      double _current_vel_mps = ymc::translateCanData(id, data, &g_mode);
      if(_current_vel_mps != RET_NO_PUBLISH )
      {
	      geometry_msgs::TwistStamped ts;
        ts.header.frame_id = "base_link";
        ts.header.stamp = ros::Time::now();
	      ts.twist.linear.x = _current_vel_mps;
	      g_current_twist_pub.publish(ts);
      }

    }

  }
}

} // namespace

int main(int argc, char *argv[])
{
  // ROS initialization
  ros::init(argc, argv, "g30esli_interface");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  g_nh = new ros::NodeHandle();

  private_nh.param<double>("wheel_base", g_wheel_base, 2.4);
  private_nh.param<int>("mode", g_mode, 8);
  private_nh.param<std::string>("device", g_device, "can0");
  private_nh.param<int>("loop_rate", g_loop_rate, 100);
  private_nh.param<int>("stop_time_sec", g_stop_time_sec, 1);

  // init cansend tool
  g_cansender.init(g_device);

  // subscriber
  ros::Subscriber twist_cmd_sub = n.subscribe<geometry_msgs::TwistStamped>("twist_cmd", 1, twist_cmd_callback);
  ros::Subscriber vehicle_cmd_sub = n.subscribe<autoware_msgs::VehicleCmd>("vehicle_cmd", 1, vehicle_cmd_callback);
  ros::Subscriber current_vel_sub = n.subscribe<geometry_msgs::TwistStamped>("current_velocity", 1, current_vel_callback);
  ros::Subscriber current_joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1, current_joy_callback);

  // publisher
  g_current_twist_pub = n.advertise<geometry_msgs::TwistStamped>("ymc_current_twist", 10);
  g_state_command_pub = n.advertise<std_msgs::Int32>("/state_cmd", 10);

  ros::Timer timer = n.createTimer(ros::Duration(0.1), joy_check_callback);

  // read can data from candump
  FILE *fp = popen("candump can0", "r");

  // create threads
  std::thread t1(changeMode);
  std::thread t2(readCanData, fp);

  ros::Rate loop_rate = g_loop_rate;
  bool stopping_flag = true;
  while (ros::ok())
  {
    // data
    unsigned char mode              = static_cast<unsigned char>(g_mode);
    unsigned char shift             = g_shift;

    uint16_t target_velocity_ui16   = (g_automode) ? g_target_velocity_ui16 : g_manual_target_velocity_ui16;
    int16_t steering_angle_deg_i16  = (g_automode) ? g_steering_angle_deg_i16 : g_manual_steering_angle_deg_i16;
    unsigned char brake      = g_brake;
    unsigned char heart_beat = 0;

    if (g_automode)
    {
      double velocity_diff = g_current_vel_kmph - g_target_vel_kmph;
      if (velocity_diff > 10.0)
      {
        ROS_ERROR("Brake 2nd !!");
        brake = 2;
      }
      else if (velocity_diff > 3.0)
      {
        ROS_ERROR("Brake 1st !! ");
        brake = 1;
      }
    }

    // Insert data to 8 byte array
    unsigned char data[8] = {};
    ymc::setCanData(data, mode, shift, target_velocity_ui16, steering_angle_deg_i16, brake, heart_beat);

    // send can data
    std::string send_id("200");
    size_t data_size = sizeof(data) / sizeof(data[0]);
    char can_cmd[256];
    std::strcpy(can_cmd, mycansend::makeCmdArgument(data, data_size, send_id).c_str());
    g_cansender.send(can_cmd);

    unsigned char data_emergency[1] = {};
    unsigned char emergency = brake;
    std::memcpy(data_emergency, &emergency, sizeof(emergency));
    size_t data_size_emergency = sizeof(data_emergency) / sizeof(data_emergency[0]);
    char can_cmd_emergency[256];
    std::strcpy(can_cmd_emergency, mycansend::makeCmdArgument(data_emergency, data_size_emergency, std::string("2FF")).c_str());
    g_cansender.send(can_cmd_emergency);

    // unsigned char data_blinker[2] = {};
    // unsigned char blinker01 = 0;
    // unsigned char blinker02 = 4;
    // std::memcpy(data_blinker, &blinker01, sizeof(blinker01));
    // std::memcpy(data_blinker+1, &blinker02, sizeof(blinker02));
    // size_t data_size_blinker = sizeof(data_blinker) / sizeof(data_blinker[0]);
    // char can_cmd_blinker[256];
    // std::strcpy(can_cmd_blinker, mycansend::makeCmdArgument(data_blinker, data_size_blinker, std::string("201")).c_str());
    // g_cansender.send(can_cmd_blinker);

    // wait for callbacks
    ros::spinOnce();
    loop_rate.sleep();

    heart_beat += 1;
  }

  g_terminate_thread = true;
  t1.join();
  t2.join();

  pclose(fp);

  return 0;
}
