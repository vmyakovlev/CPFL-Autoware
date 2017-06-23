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

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <ndt_localizer/ndt_stat.h>
#include <std_msgs/Bool.h>

namespace
{
class TwistStatPublisher
{
private:
  ros::NodeHandle private_nh_;
  ros::NodeHandle nh_;

  float exe_time_threshold_;
  int iteration_threshold_;
  float score_threshold_;
  double twist_cmd_hz_threshold_;
  double ndt_stat_hz_threshold_;
  double twist_stat_hz_;

  ros::Subscriber twist_cmd_sub_;
  ros::Subscriber ndt_stat_sub_;
  ros::Publisher twist_stat_pub_;

  uint32_t twist_cmd_cnt_;
  uint32_t ndt_stat_cnt_;
  bool exists_twist_cmd_;
  bool exists_ndt_stat_;
  float exe_time_;
  int iteration_;
  float score_;

  void catchTwistCmd(const geometry_msgs::TwistStamped &msg)
  {
    ++twist_cmd_cnt_;
  }

  void catchNDTStat(const ndt_localizer::ndt_stat &msg)
  {
    ++ndt_stat_cnt_;
    exe_time_ = msg.exe_time;
    iteration_ = msg.iteration;
    score_ = msg.score;
  }

  void publishTwistStat(bool data)
  {
    std_msgs::Bool msg;
    msg.data = data;
    twist_stat_pub_.publish(msg);
  }

public:
  TwistStatPublisher() : private_nh_("~")
  {
    private_nh_.param<float>("exe_time_threshold", exe_time_threshold_, 80.0);
    private_nh_.param<int>("iteration_threshold", iteration_threshold_, 10);
    private_nh_.param<float>("score_threshold", score_threshold_, 100.0);
    private_nh_.param<double>("twist_cmd_hz_threshold", twist_cmd_hz_threshold_, 1.0);
    private_nh_.param<double>("ndt_stat_hz_threshold", ndt_stat_hz_threshold_, 1.0);
    private_nh_.param<double>("twist_stat_hz", twist_stat_hz_, 10.0);

    twist_cmd_sub_ = nh_.subscribe("twist_cmd", 100, &TwistStatPublisher::catchTwistCmd, this);
    ndt_stat_sub_ = nh_.subscribe("ndt_stat", 100, &TwistStatPublisher::catchNDTStat, this);
    twist_stat_pub_ = nh_.advertise<std_msgs::Bool>("twist_stat", 1);

    twist_cmd_cnt_ = 0;
    ndt_stat_cnt_ = 0;
    exists_twist_cmd_ = false;
    exists_ndt_stat_ = false;
    exe_time_ = std::numeric_limits<float>::max();
    iteration_ = std::numeric_limits<int>::max();
    score_ = std::numeric_limits<float>::max();
  }

  void run()
  {
    ros::Rate loop_rate(twist_stat_hz_);
    ros::Time prev = ros::Time::now();
    while (ros::ok())
    {
      ros::Time now = ros::Time::now();
      if (now - prev >= ros::Duration(1))
      {
        prev = now;
        exists_twist_cmd_ = twist_cmd_cnt_ >= twist_cmd_hz_threshold_;
        twist_cmd_cnt_ = 0;
        exists_ndt_stat_ = ndt_stat_cnt_ >= ndt_stat_hz_threshold_;
        ndt_stat_cnt_ = 0;
      }
      if (exists_twist_cmd_ && exists_ndt_stat_ &&
          exe_time_ <= exe_time_threshold_ && iteration_ <= iteration_threshold_ && score_ <= score_threshold_)
        publishTwistStat(true);
      else
        publishTwistStat(false);
      loop_rate.sleep();
      ros::spinOnce();
    }
  }
};
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_stat_publisher");

  TwistStatPublisher tsp;
  tsp.run();

  return EXIT_SUCCESS;
}
