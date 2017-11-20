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

#ifndef NDT_SLAM_PCL_H
#define NDT_SLAM_PCL_H

#include <memory>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include "libndt_slam_pcl.h"
#include "libndt_slam_pcl_omp.h"
#include "libndt_slam_pcl_gpu.h"
#include "libdata_structs.h"
#include "libconvert_ros_msgs.h"
#include "libslam_observer.h"

class NdtSlam
{
    using PointSource = pcl::PointXYZI;
    using PointTarget = pcl::PointXYZI;

    public:
        NdtSlam(ros::NodeHandle nh, ros::NodeHandle private_nh);

    private:
        void pointsMapUpdatedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr);
        void manualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_conv_msg_ptr);
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
        ros::Publisher transformation_probability_pub_;

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
        std::unique_ptr< LibNdtSlamBase<PointSource, PointTarget> > localizer_ptr_;
        LibSlamObserver observer_;  //this instance is not observer pattern

        Eigen::Matrix4f tf_ltob_;
        bool is_with_mapping_;

};

NdtSlam::NdtSlam(ros::NodeHandle nh, ros::NodeHandle private_nh)
    :nh_(nh)
    ,private_nh_(private_nh)
    ,is_with_mapping_(false)
{

    bool use_omp = false;
    private_nh_.getParam("use_omp", use_omp);
    bool use_gpu = false;
    private_nh_.getParam("use_gpu", use_gpu);

    if(use_omp && use_gpu)
    {
        ROS_ERROR("cannot set both OMP and GPU. please check rosparam 'use_omp' and 'use_gpu'.");
        exit(1);
    }
    else if(use_omp && !use_gpu)
    {
        ROS_INFO("use NDT SLAM PCL OMP version");
        localizer_ptr_.reset(new LibNdtSlamPCLOMP<PointSource, PointTarget>);
    }
    else if(!use_omp && use_gpu)
    {
        ROS_INFO("use NDT SLAM PCL GPU version");
        localizer_ptr_.reset(new LibNdtSlamPCLGPU<PointSource, PointTarget>);
    }
    else
    {
        ROS_INFO("use NDT SLAM PCL version");
        localizer_ptr_.reset(new LibNdtSlamPCL<PointSource, PointTarget>);
    }

    localizer_ptr_->setTransformationEpsilon(0.01);
    localizer_ptr_->setStepSize(0.1);
    localizer_ptr_->setResolution(1.0);
    localizer_ptr_->setMaximumIterations(30);

    private_nh_.getParam("is_with_mapping", is_with_mapping_);
    ROS_INFO("is_with_mapping_: %d", is_with_mapping_);

    Pose tf_pose(1.2, 0, 2.0, 0, 0, 0);
    nh.getParam("tf_x", tf_pose.x);
    nh.getParam("tf_y", tf_pose.y);
    nh.getParam("tf_z", tf_pose.z);
    nh.getParam("tf_roll", tf_pose.roll);
    nh.getParam("tf_pitch", tf_pose.pitch);
    nh.getParam("tf_yaw", tf_pose.yaw);
    ROS_INFO("tf(x, y, z, roll, pitch, yaw): %lf, %lf, %lf, %lf, %lf, %lf", tf_pose.x, tf_pose.y, tf_pose.z, tf_pose.roll, tf_pose.pitch, tf_pose.yaw);

    auto tf_btol = convertToEigenMatrix4f(tf_pose);
    tf_ltob_ = tf_btol.inverse();

    const int points_buffer = is_with_mapping_ ? 1000 : 10;

    points_map_region_pub_  = nh_.advertise<sensor_msgs::PointCloud2>   ("points_map_region",  10);
    localizer_pose_pub_     = nh_.advertise<geometry_msgs::PoseStamped> ("localizer_pose",     10);
    localizer_velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("localizer_velocity", 10);
    transformation_probability_pub_ = nh_.advertise<std_msgs::Float32>  ("transformation_probability", 10);

    //points_map_updated_sub_ = nh_.subscribe("/points_map_updated", 10, &NdtSlam::pointsMapUpdatedCallback, this);
    points_map_updated_sub_ = nh_.subscribe("/points_map", 10, &NdtSlam::pointsMapUpdatedCallback, this);
    //manual_pose_sub_        = nh_.subscribe("/manual_pose",        10, &NdtSlam::manualPoseCallback, this);
    manual_pose_sub_        = nh_.subscribe("/initialpose",        10, &NdtSlam::manualPoseCallback, this);
    static_pose_sub_        = nh_.subscribe("/static_pose",        10, &NdtSlam::staticPoseCallback, this);
    gnss_pose_sub_          = nh_.subscribe("/gnss_pose",          10, &NdtSlam::gnssPoseCallback, this);
    delta_pose_sub_         = nh_.subscribe("/delta_pose",         points_buffer*10, &NdtSlam::deltaPoseCallback, this);
    points_raw_sub_         = nh_.subscribe("/points_raw",         points_buffer, &NdtSlam::pointsRawCallback, this);
    points_fused_sub_       = nh_.subscribe("/points_fused",       points_buffer, &NdtSlam::pointsFusedCallback, this);
    //points_filtered_sub_    = nh_.subscribe("/points_filtered",    points_buffer, &NdtSlam::pointsFilteredCallback, this);
    points_filtered_sub_    = nh_.subscribe("/filtered_points",    points_buffer, &NdtSlam::pointsFilteredCallback, this);

}

void NdtSlam::pointsMapUpdatedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    const auto pointcloud = convertFromROSMsg<PointTarget>(*pointcloud2_msg_ptr);
    localizer_ptr_->updatePointsMap(pointcloud);
}

void NdtSlam::manualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_conv_msg_ptr)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = pose_conv_msg_ptr->header;
    pose_msg.pose = pose_conv_msg_ptr->pose.pose;

    //TODO: if use_gpu==ture, bug occurs ← fixed?
    geometry_msgs::PoseStamped transformed_pose_msg;
    try
    {
        ros::Time now = ros::Time(0);
        tf_listener_.waitForTransform("/map", pose_msg.header.frame_id, now, ros::Duration(10.0));
        tf_listener_.transformPose("/map", now, pose_msg, pose_msg.header.frame_id, transformed_pose_msg);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    const auto pose = convertFromROSMsg(transformed_pose_msg);
//    const auto pose = convertFromROSMsg(pose_msg);

    localizer_ptr_->updateManualPose(pose);
}

void NdtSlam::staticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr)
{
    const auto pose = convertFromROSMsg(*pose_msg_ptr);
    localizer_ptr_->updateStaticPose(pose);
}

void NdtSlam::gnssPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr)
{
    const auto pose = convertFromROSMsg(*pose_msg_ptr);
    localizer_ptr_->updateGnssPose(pose);
}

void NdtSlam::deltaPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr)
{
    const auto pose = convertFromROSMsg(*pose_msg_ptr);
    localizer_ptr_->updateDeltaPose(pose_msg_ptr->header.stamp.toSec(), pose);
}

void NdtSlam::pointsRawCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    const auto pointcloud = convertFromROSMsg<PointSource>(*pointcloud2_msg_ptr);
    localizer_ptr_->updatePointsRaw(pointcloud);
}

void NdtSlam::pointsFusedCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    const auto pointcloud = convertFromROSMsg<PointSource>(*pointcloud2_msg_ptr);
    localizer_ptr_->updatePointsFused(pointcloud);
}

void NdtSlam::pointsFilteredCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    const auto pointcloud = convertFromROSMsg<PointSource>(*pointcloud2_msg_ptr);
    localizer_ptr_->updatePointsFiltered(pointcloud);   // == setInputSource(pointcloud);
    localizer_ptr_->updateLocalizer(pointcloud2_msg_ptr->header.stamp.toSec());
    if(is_with_mapping_)
        localizer_ptr_->updateMap();  //TODO: this code should be called after pointsRawCallback
    publishTopics(pointcloud2_msg_ptr->header);
    localizer_ptr_->writeLogFile();

    EvaluationValue ev;
    ev.pose = localizer_ptr_->getLocalizerPose();
    ev.align_time = localizer_ptr_->getAlignTime();
    ev.transformation_probability = localizer_ptr_->getTransformationProbability();
    observer_.update(ev);

}

void NdtSlam::publishTopics(std_msgs::Header header)
{
    std_msgs::Header common_header;
    common_header.frame_id = "map";
    common_header.seq = header.seq;
    common_header.stamp = header.stamp;

    const auto localizer_pose = localizer_ptr_->getLocalizerPose();
    const auto base_link_pose = transformToPose(localizer_pose, tf_ltob_);
    const auto base_link_pose_msg = convertToROSMsg(common_header, base_link_pose);
    localizer_pose_pub_.publish(base_link_pose_msg);  //TODO: rename publisher?

    auto localizer_velocity = localizer_ptr_->getLocalizerVelocity();
    localizer_velocity.linear.x = std::sqrt(std::pow(localizer_velocity.linear.x, 2.0) + std::pow(localizer_velocity.linear.y, 2.0) + std::pow(localizer_velocity.linear.z, 2.0));
    localizer_velocity.linear.y = 0;
    localizer_velocity.linear.z = 0;
    localizer_velocity.angular.x = 0;
    localizer_velocity.angular.y = 0;
    localizer_velocity.angular.z = localizer_velocity.angular.z;
    std_msgs::Header velocity_header = common_header;
    velocity_header.frame_id = "base_link";
    const auto localizer_velocity_msg = convertToROSMsg(velocity_header, localizer_velocity);
    localizer_velocity_pub_.publish(localizer_velocity_msg);

    if(is_with_mapping_)
    {
        const auto map = localizer_ptr_->getMap();
        const auto map_msg = convertToROSMsg(common_header, map);
        points_map_region_pub_.publish(map_msg);
    }

    tf::Quaternion base_link_q;
    base_link_q.setRPY(base_link_pose.roll, base_link_pose.pitch, base_link_pose.yaw);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(base_link_pose.x, base_link_pose.y, base_link_pose.z));
    transform.setRotation(base_link_q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, common_header.stamp, "/map", "/base_link"));


    const double trans_probability_score = observer_.getTransformationProbabilityScore();
    std_msgs::Float32 trans_probability_score_msg;
    trans_probability_score_msg.data = trans_probability_score;
    transformation_probability_pub_.publish(trans_probability_score_msg);
}

#endif
