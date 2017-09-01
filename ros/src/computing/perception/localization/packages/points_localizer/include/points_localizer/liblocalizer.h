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

#ifndef LIBLOCALIZER_H
#define LIBLOCALIZER_H

#include <iostream>
#include <string>
#include <chrono>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "libdata_structs.h"
#include "libcalc_predict_pose.h"

Eigen::Matrix4f convertToEigenMatrix4f(const Pose& pose)
{
    const Eigen::Translation3f translation(pose.x, pose.y, pose.z);
    const Eigen::AngleAxisf rotation_x(pose.roll, Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf rotation_y(pose.pitch, Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf rotation_z(pose.yaw, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix4f m = (translation * rotation_z * rotation_y * rotation_x).matrix();
    return m;
}


Pose convertToPose(const Eigen::Matrix4f& m)
{
  Pose pose;
  pose.x = m(0, 3);
  pose.y = m(1, 3);
  pose.z = m(2, 3);

  //this code is written reference to tf::getEulerYPR()
  if (std::fabs(m(2,0)) >= 1)
  {
    pose.yaw = 0;
    if (m(2,0) < 0)
    {
      pose.pitch = M_PI / 2.0;
      pose.roll = std::atan2(m(0,1),m(0,2));
    }
    else
    {
      pose.pitch = -M_PI / 2.0;
      pose.roll = std::atan2(-m(0,1),-m(0,2));
    }
  }
  else
  {
    pose.pitch = -std::asin(m(2,0));
    pose.roll  = std::atan2(m(2,1)/std::cos(pose.pitch),
                            m(2,2)/std::cos(pose.pitch));
    pose.yaw   = std::atan2(m(1,0)/std::cos(pose.pitch),
                            m(0,0)/std::cos(pose.pitch));
  }

  return pose;
}


Pose transformToPose(const Pose& pose, const Eigen::Matrix4f& m)
{
  Eigen::Translation3f eigen_xyz(pose.x, pose.y, pose.z);
  Eigen::AngleAxisf eigen_roll(pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf eigen_pitch(pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf eigen_yaw(pose.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f trans_m = (eigen_xyz * eigen_yaw * eigen_pitch * eigen_roll) * m;
  Pose trans_pose = convertToPose(trans_m);

  return trans_pose;
}

template <class PointT>
class LibLocalizer
{
    public:
        LibLocalizer();
        virtual ~LibLocalizer(){};
        void updatePointsMap(const pcl::PointCloud<PointT>& pointcloud);
        void updateManualPose(const Pose& pose);
        void updateStaticPose(const Pose& pose);
        void updateGnssPose(const Pose& pose);
        void updateDeltaPose(double current_data_time_sec, const Pose& pose);
        void updatePointsRaw(const pcl::PointCloud<PointT>& pointcloud);
        void updatePointsFused(const pcl::PointCloud<PointT>& pointcloud);
        void updatePointsFiltered(const pcl::PointCloud<PointT>& pointcloud);
        void updateLocalizer(double current_scan_time_sec);
        Pose getLocalizerPose() const;
        Velocity getLocalizerVelocity() const;

    protected:
        virtual void align(const Pose& predict_pose) = 0;
        virtual void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointT> const>& map_ptr) = 0;
        virtual void setInputSource(const boost::shared_ptr< pcl::PointCloud<PointT> const>& scan_ptr) = 0;
        virtual double getFitnessScore() = 0;
        virtual Pose getFinalPose() = 0;

    private:
        Pose localizer_pose_;
        Velocity localizer_velocity_;

        LibCalcPredictPose calc_predict_pose_;

        boost::shared_ptr< pcl::PointCloud<PointT> > scan_ptr_;
        boost::shared_ptr< pcl::PointCloud<PointT> > map_ptr_;
        Eigen::Matrix4f tf_btol_;
        Eigen::Matrix4f tf_ltob_;

        bool is_initial_pose_set_;
};

template <class PointT>
LibLocalizer<PointT>::LibLocalizer()
    :is_initial_pose_set_(false)
{
    Pose tf_pose(1.2, 0, 2.0, 0, 0, 0);
    tf_btol_ = convertToEigenMatrix4f(tf_pose);
    tf_ltob_ = convertToEigenMatrix4f(tf_pose*(-1.0));
}

template <class PointT>
void LibLocalizer<PointT>::updatePointsMap(const pcl::PointCloud<PointT>& pointcloud)
{
    std::cout << __func__ << std::endl;
    map_ptr_ = boost::make_shared< pcl::PointCloud<PointT> >(pointcloud);
    setInputTarget(map_ptr_);
}

template <class PointT>
void LibLocalizer<PointT>::updateManualPose(const Pose& pose)
{
    std::cout << __func__ << std::endl;
    localizer_pose_ = pose;

    if(map_ptr_ != nullptr)
    {
      double min_distance = DBL_MAX;
      double nearest_z = localizer_pose_.z;
      for (const auto& p : map_ptr_->points)
      {
        double distance = hypot(localizer_pose_.x - p.x, localizer_pose_.y - p.y);
        if (distance < min_distance)
        {
          min_distance = distance;
          nearest_z = p.z;
        }
      }
      localizer_pose_.z = nearest_z;
    }

    is_initial_pose_set_ = true;
}

template <class PointT>
void LibLocalizer<PointT>::updateStaticPose(const Pose& pose)
{
    std::cout << __func__ << std::endl;
    localizer_pose_ = pose;
    is_initial_pose_set_ = true;
}

template <class PointT>
void LibLocalizer<PointT>::updateGnssPose(const Pose& pose)
{
    std::cout << __func__ << std::endl;
    if(is_initial_pose_set_ = false || getFitnessScore() >= 500.0)
    {
        localizer_pose_ = pose;
        is_initial_pose_set_ = true;
    }
}

template <class PointT>
void LibLocalizer<PointT>::updateDeltaPose(double current_data_time_sec, const Pose& pose)
{
    //std::cout << __func__ << std::endl;
    calc_predict_pose_.addData(current_data_time_sec, pose);
}

template <class PointT>
void LibLocalizer<PointT>::updatePointsRaw(const pcl::PointCloud<PointT>& pointcloud)
{
    //create map
    std::cout << __func__ << std::endl;
}

template <class PointT>
void LibLocalizer<PointT>::updatePointsFused(const pcl::PointCloud<PointT>& pointcloud)
{
    //create map
}

template <class PointT>
void LibLocalizer<PointT>::updatePointsFiltered(const pcl::PointCloud<PointT>& pointcloud)
{
    std::cout << __func__ << std::endl;
    scan_ptr_ = boost::make_shared< pcl::PointCloud<PointT> >(pointcloud);
    setInputSource(scan_ptr_);
}

template <class PointT>
void LibLocalizer<PointT>::updateLocalizer(double current_scan_time_sec)
{
    std::cout << __func__ << std::endl;
    if(map_ptr_ == nullptr)
    {
        std::cout << "received points. But map is not loaded" << std::endl;
        return;
    }
    if(is_initial_pose_set_ == false)
    {
        std::cout << "received points. But initial pose is not set" << std::endl;
        return;
    }

    static double previous_scan_time_sec = current_scan_time_sec;

    auto predict_pose = calc_predict_pose_.predictNextPose(previous_scan_time_sec, current_scan_time_sec, localizer_pose_);
    std::cout << localizer_pose_ << std::endl;
    std::cout << predict_pose << std::endl;
    predict_pose = transformToPose(predict_pose, tf_btol_);
    const auto align_start = std::chrono::system_clock::now();
    align(predict_pose);
    const auto align_end = std::chrono::system_clock::now();
    const auto align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

    const auto calc_fitness_score_start = std::chrono::system_clock::now();
    const auto fitness_score = getFitnessScore();
    const auto calc_fitness_score_end = std::chrono::system_clock::now();
    const auto calc_fitness_score_time = std::chrono::duration_cast<std::chrono::microseconds>(calc_fitness_score_end - calc_fitness_score_start).count() / 1000.0;

    localizer_pose_ = getFinalPose();
    localizer_pose_ = transformToPose(localizer_pose_, tf_ltob_);
    std::cout << localizer_pose_ << std::endl;
    static Pose previous_localizer_pose = localizer_pose_;
    localizer_velocity_ = Velocity(previous_localizer_pose, localizer_pose_, current_scan_time_sec-previous_scan_time_sec);

    previous_scan_time_sec = current_scan_time_sec;
    previous_localizer_pose = localizer_pose_;

}

template <class PointT>
Pose LibLocalizer<PointT>::getLocalizerPose() const
{
    return localizer_pose_;
}

template <class PointT>
Velocity LibLocalizer<PointT>::getLocalizerVelocity() const
{
    return localizer_velocity_;
}



#endif
