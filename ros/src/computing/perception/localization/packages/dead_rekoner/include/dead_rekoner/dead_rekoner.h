/*
 *  Copyright (c) 2017, Tier IV, Inc.
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

#ifndef DEAD_REKONER_H
#define DEAD_REKONER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>

#include "libdead_rekoner.h"
#include "libdata_structs.h"
#include "libconvert_ros_msgs.h"

class DeadRekoner
{
    public:
        DeadRekoner(ros::NodeHandle nh, ros::NodeHandle private_nh);

    private:
        template <class T> void update(const boost::shared_ptr<T const> msg_ptr);
        void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity_msg_ptr);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_ptr);

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher delta_pose_pub_;
        ros::Subscriber velocity_sub_;
        ros::Subscriber imu_sub_;

        LibDeadRekoner dead_rekoner_;
};

DeadRekoner::DeadRekoner(ros::NodeHandle nh, ros::NodeHandle private_nh)
    :nh_(nh)
    ,private_nh_(private_nh)
{
    bool use_imu = false;
    private_nh_.getParam("use_imu", use_imu);
    dead_rekoner_.setUseImuFlag(use_imu);

    delta_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("delta_pose", 10);
    velocity_sub_ = nh.subscribe("/current_velocity", 10, &DeadRekoner::velocityCallback, this);
    imu_sub_ = nh.subscribe("/imu_raw", 10, &DeadRekoner::imuCallback, this);
}

template <class T>
void DeadRekoner::update(const boost::shared_ptr<T const> msg_ptr)
{
    auto data = convertFromROSMsg(*msg_ptr);
    double current_time_sec = msg_ptr->header.stamp.toSec();
    auto delta_pose = dead_rekoner_.calcDeltaPose(current_time_sec, data);
    auto delta_pose_msg = convertToROSMsg(msg_ptr->header, delta_pose);
    delta_pose_msg.header.frame_id = "";  //TODO: base_link?
    delta_pose_pub_.publish(delta_pose_msg);

    std::cout << delta_pose << std::endl;
}

void DeadRekoner::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity_msg_ptr)
{
    update(velocity_msg_ptr);
}

void DeadRekoner::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_ptr)
{
    update(imu_msg_ptr);
}

#endif
