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

#ifndef NDT_SLMA_PCL_H
#define NDT_SLMA_PCL_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "libndt_slam_pcl.h"
#include "libdata_structs.h"
#include "libconvert_ros_msgs.h"


class Localizer
{
    using PointT = pcl::PointXYZ;

    public:
        Localizer(ros::NodeHandle nh, ros::NodeHandle private_nh);

    private:
        void pointsMapUpdatedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr);
        void manualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg_ptr);
        void staticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr);
        void gnssPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr);
        void deltaPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr);
        void pointsRawCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr);
        void pointsFusedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr);
        void pointsFilteredCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr);

        void publishTopics(std_msgs::Header header);

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher points_map_region_pub_;
        ros::Publisher localizer_pose_pub_;
        ros::Publisher localizer_velocity_pub_;

        ros::Subscriber points_map_updated_sub_;
        ros::Subscriber manual_pose_sub_;
        ros::Subscriber static_pose_sub_;
        ros::Subscriber gnss_pose_sub_;
        ros::Subscriber delta_pose_sub_;
        ros::Subscriber points_raw_sub_;
        ros::Subscriber points_fused_sub_;
        ros::Subscriber points_filtered_sub_;

        tf::TransformBroadcaster tf_broadcaster_;
        tf::TransformListener tf_listener_;
        LibNdtSlamPCL<PointT> localizer_;

};

Localizer::Localizer(ros::NodeHandle nh, ros::NodeHandle private_nh)
    :nh_(nh)
    ,private_nh_(private_nh)
{
//    private_nh_.getParam("use_imu", use_imu);

    points_map_region_pub_  = nh_.advertise<sensor_msgs::PointCloud2>   ("points_map_region",  10);
    localizer_pose_pub_     = nh_.advertise<geometry_msgs::PoseStamped> ("localizer_pose",     10);
    localizer_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("localizer_velocity", 10);

    //points_map_updated_sub_ = nh_.subscribe("/points_map_updated", 10, &Localizer::pointsMapUpdatedCallback, this);
    points_map_updated_sub_ = nh_.subscribe("/points_map", 10, &Localizer::pointsMapUpdatedCallback, this);
    //manual_pose_sub_        = nh_.subscribe("/manual_pose",        10, &Localizer::manualPoseCallback, this);
    manual_pose_sub_        = nh_.subscribe("/initialpose",        10, &Localizer::manualPoseCallback, this);
    static_pose_sub_        = nh_.subscribe("/static_pose",        10, &Localizer::staticPoseCallback, this);
    gnss_pose_sub_          = nh_.subscribe("/gnss_pose",          10, &Localizer::gnssPoseCallback, this);
    delta_pose_sub_         = nh_.subscribe("/delta_pose",         10, &Localizer::deltaPoseCallback, this);
    points_raw_sub_         = nh_.subscribe("/points_raw",         10, &Localizer::pointsRawCallback, this);
    points_fused_sub_       = nh_.subscribe("/points_fused",       10, &Localizer::pointsFusedCallback, this);
    //points_filtered_sub_    = nh_.subscribe("/points_filtered",    10, &Localizer::pointsFilteredCallback, this);
    points_filtered_sub_    = nh_.subscribe("/filtered_points",    10, &Localizer::pointsFilteredCallback, this);

    localizer_.setResolution(1.0);
    localizer_.setTransformationEpsilon(0.01);
    localizer_.setStepSize(0.1);
    localizer_.setMaximumIterations(30);

}

void Localizer::pointsMapUpdatedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    const auto pointcloud = convertFromROSMsg<PointT>(*pointcloud2_msg_ptr);
    localizer_.updatePointsMap(pointcloud);
}

void Localizer::manualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg_ptr)
{
    const auto pose = convertFromROSMsg(*pose_msg_ptr);
    localizer_.updateManualPose(pose);
}

void Localizer::staticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr)
{
    const auto pose = convertFromROSMsg(*pose_msg_ptr);
    localizer_.updateStaticPose(pose);
}

void Localizer::gnssPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr)
{
    const auto pose = convertFromROSMsg(*pose_msg_ptr);
    localizer_.updateStaticPose(pose);
}

void Localizer::deltaPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr)
{
    const auto pose = convertFromROSMsg(*pose_msg_ptr);
    localizer_.updateDeltaPose(pose_msg_ptr->header.stamp.toSec(), pose);
}

void Localizer::pointsRawCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    const auto pointcloud = convertFromROSMsg<PointT>(*pointcloud2_msg_ptr);
    localizer_.updatePointsRaw(pointcloud);
}

void Localizer::pointsFusedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    const auto pointcloud = convertFromROSMsg<PointT>(*pointcloud2_msg_ptr);
    localizer_.updatePointsFused(pointcloud);
}

void Localizer::pointsFilteredCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    const auto pointcloud = convertFromROSMsg<PointT>(*pointcloud2_msg_ptr);
    localizer_.updatePointsFiltered(pointcloud);   // == setInputSource(pointcloud);
    localizer_.updateLocalizer(pointcloud2_msg_ptr->header.stamp.toSec());
    publishTopics(pointcloud2_msg_ptr->header);
}

void Localizer::publishTopics(std_msgs::Header header)
{
    std_msgs::Header common_header;
    common_header.frame_id = "map";
    common_header.seq = header.seq;
    common_header.stamp = header.stamp;

    const auto localizer_pose = localizer_.getLocalizerPose();
    const auto localizer_pose_msg = convertToROSMsg(common_header ,localizer_pose);
    localizer_pose_pub_.publish(localizer_pose_msg);

    const auto localizer_velocity = localizer_.getLocalizerVelocity();
    const auto localizer_velocity_msg = convertToROSMsg(common_header ,localizer_velocity);
    localizer_velocity_pub_.publish(localizer_velocity_msg);

    tf::Quaternion localizer_q;
    localizer_q.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(localizer_pose.x, localizer_pose.y, localizer_pose.z));
    transform.setRotation(localizer_q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, common_header.stamp, "/map", "/base_link"));

}
#endif
