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
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include "libdata_structs.h"
#include "libcalc_predict_pose.h"

static Eigen::Matrix4f convertToEigenMatrix4f(const Pose& pose)
{
    const Eigen::Translation3f translation(pose.x, pose.y, pose.z);
    const Eigen::AngleAxisf rotation_x(pose.roll, Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf rotation_y(pose.pitch, Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf rotation_z(pose.yaw, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix4f m = (translation * rotation_z * rotation_y * rotation_x).matrix();
    return m;
}

static Pose convertToPose(const Eigen::Matrix4f& m)
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

static Pose transformToPose(const Pose& pose, const Eigen::Matrix4f& m)
{
  Eigen::Translation3f eigen_xyz(pose.x, pose.y, pose.z);
  Eigen::AngleAxisf eigen_roll(pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf eigen_pitch(pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf eigen_yaw(pose.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f trans_m = (eigen_xyz * eigen_yaw * eigen_pitch * eigen_roll) * m;
  Pose trans_pose = convertToPose(trans_m);

  return trans_pose;
}

template <class PointTarget>
static void passThroughPointCloud(const boost::shared_ptr< pcl::PointCloud<PointTarget> > input_point_cloud_ptr, boost::shared_ptr< pcl::PointCloud<PointTarget> > output_point_cloud_ptr, const Pose& pose, double offset_meter)
{
    pcl::PassThrough<PointTarget> pass;
    pass.setInputCloud(input_point_cloud_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(pose.x - offset_meter, pose.x + offset_meter);
    pass.filter(*output_point_cloud_ptr);

    pass.setInputCloud(output_point_cloud_ptr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(pose.y - offset_meter, pose.y + offset_meter);
    pass.filter(*output_point_cloud_ptr);
}

template <class PointTarget>
static void addPointCloud(const boost::shared_ptr< pcl::PointCloud<PointTarget> > input_ptr, boost::shared_ptr< pcl::PointCloud<PointTarget> > &output_ptr, const Pose& pose)
{
    boost::shared_ptr< pcl::PointCloud<PointTarget> > transformed_input_ptr(new pcl::PointCloud<PointTarget>);
    const auto eigen_pose = convertToEigenMatrix4f(pose);
    pcl::transformPointCloud(*input_ptr, *transformed_input_ptr, eigen_pose);

    pcl::PointCloud<PointTarget> new_pointcloud;
    new_pointcloud += *output_ptr;
    new_pointcloud += *transformed_input_ptr;
    output_ptr = new_pointcloud.makeShared();
}

template <class PointTarget>
static void removePointCloudAroundSensor(const boost::shared_ptr< pcl::PointCloud<PointTarget> > input_ptr, boost::shared_ptr< pcl::PointCloud<PointTarget> > output_ptr, double remove_range_meter)
{
    double r = 0;
    for (const auto& p : input_ptr->points)
    {
      r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (r > remove_range_meter)
      {
        output_ptr->push_back(p);
      }
    }
}

template <class PointSource, class PointTarget>
class LibLocalizer
{
    public:
        LibLocalizer();
//        virtual ~LibLocalizer() = default;
        virtual ~LibLocalizer();
        void updatePointsMap(const pcl::PointCloud<PointTarget>& pointcloud);
        void updateManualPose(const Pose& pose);
        void updateStaticPose(const Pose& pose);
        void updateGnssPose(const Pose& pose);
        void updateDeltaPose(double current_data_time_sec, const Pose& pose);
        void updatePointsRaw(const pcl::PointCloud<PointSource>& pointcloud);
        void updatePointsFused(const pcl::PointCloud<PointSource>& pointcloud);
        void updatePointsFiltered(const pcl::PointCloud<PointSource>& pointcloud);
        void updateLocalizer(double current_scan_time_sec);
        Pose getLocalizerPose() const;
        Velocity getLocalizerVelocity() const;
        pcl::PointCloud<PointTarget> getMap() const;
        void updateMap();
        void writeLogFile();

    protected:
        virtual void align(const Pose& predict_pose) = 0;
        virtual void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr) = 0;
        virtual void setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr) = 0;
        virtual double getFitnessScore() = 0;
        virtual Pose getFinalPose() = 0;
        virtual std::stringstream logFileContent() const;

    private:
        Pose localizer_pose_;
        Velocity localizer_velocity_;

        LibCalcPredictPose calc_predict_pose_;

        boost::shared_ptr< pcl::PointCloud<PointTarget> > map_raw_ptr_;
        boost::shared_ptr< pcl::PointCloud<PointTarget> > map_filtered_ptr_;
        boost::shared_ptr< pcl::PointCloud<PointSource> > scan_raw_ptr_;
        boost::shared_ptr< pcl::PointCloud<PointSource> > scan_filtered_ptr_;

        bool is_initial_pose_set_;
        double fitness_score_;
};

template <class PointSource, class PointTarget>
LibLocalizer<PointSource, PointTarget>::LibLocalizer()
    :is_initial_pose_set_(false)
    ,fitness_score_(0)
{
}

template <class PointSource, class PointTarget>
LibLocalizer<PointSource, PointTarget>::~LibLocalizer()
{
    std::string filename = "ndt_mapping.pcd";
    pcl::io::savePCDFileBinary(filename, *map_raw_ptr_);
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updatePointsMap(const pcl::PointCloud<PointTarget>& pointcloud)
{
    std::cout << __func__ << std::endl;
    map_raw_ptr_ = boost::make_shared< pcl::PointCloud<PointTarget> >(pointcloud);
    setInputTarget(map_raw_ptr_);
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updateManualPose(const Pose& pose)
{
    std::cout << __func__ << std::endl;
    localizer_pose_ = pose;

    if(map_raw_ptr_ != nullptr)
    {
      double min_distance = DBL_MAX;
      double nearest_z = localizer_pose_.z;
      for (const auto& p : map_raw_ptr_->points)
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
    std::cout << localizer_pose_ << std::endl;
    is_initial_pose_set_ = true;
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updateStaticPose(const Pose& pose)
{
    std::cout << __func__ << std::endl;
    localizer_pose_ = pose;
    is_initial_pose_set_ = true;
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updateGnssPose(const Pose& pose)
{
    std::cout << __func__ << "   " << fitness_score_ <<std::endl;
    if(is_initial_pose_set_ == false || fitness_score_ >= 500.0)
    {
        localizer_pose_ = pose;
        is_initial_pose_set_ = true;
    }
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updateDeltaPose(double current_data_time_sec, const Pose& pose)
{
    //std::cout << __func__ << std::endl;
    calc_predict_pose_.addData(current_data_time_sec, pose);
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updatePointsRaw(const pcl::PointCloud<PointSource>& pointcloud)
{
    std::cout << __func__ << std::endl;
    scan_raw_ptr_ = boost::make_shared< pcl::PointCloud<PointSource> >(pointcloud);
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updatePointsFused(const pcl::PointCloud<PointSource>& pointcloud)
{

}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updatePointsFiltered(const pcl::PointCloud<PointSource>& pointcloud)
{
    std::cout << __func__ << std::endl;
    scan_filtered_ptr_ = boost::make_shared< pcl::PointCloud<PointSource> >(pointcloud);
    setInputSource(scan_filtered_ptr_);
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updateLocalizer(double current_scan_time_sec)
{
    std::cout << __func__ << std::endl;
    if(map_raw_ptr_ == nullptr)
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
    //std::cout << localizer_pose_ << std::endl;
    //std::cout << predict_pose << std::endl;

    const auto align_start = std::chrono::system_clock::now();
    align(predict_pose);
    const auto align_end = std::chrono::system_clock::now();
    const auto align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;
    std::cout << "align_time: " << align_time << "ms" << std::endl;

    const auto calc_fitness_score_start = std::chrono::system_clock::now();
    fitness_score_ = getFitnessScore();
    const auto calc_fitness_score_end = std::chrono::system_clock::now();
    const auto calc_fitness_score_time = std::chrono::duration_cast<std::chrono::microseconds>(calc_fitness_score_end - calc_fitness_score_start).count() / 1000.0;

    localizer_pose_ = getFinalPose();
    std::cout << localizer_pose_ << std::endl;

    static Pose previous_localizer_pose = localizer_pose_;
    localizer_velocity_ = Velocity(previous_localizer_pose, localizer_pose_, current_scan_time_sec-previous_scan_time_sec);

    previous_scan_time_sec = current_scan_time_sec;
    previous_localizer_pose = localizer_pose_;

}

template <class PointSource, class PointTarget>
Pose LibLocalizer<PointSource, PointTarget>::getLocalizerPose() const
{
    return localizer_pose_;
}

template <class PointSource, class PointTarget>
Velocity LibLocalizer<PointSource, PointTarget>::getLocalizerVelocity() const
{
    return localizer_velocity_;
}

template <class PointSource, class PointTarget>
pcl::PointCloud<PointTarget> LibLocalizer<PointSource, PointTarget>::getMap() const
{
    return *map_filtered_ptr_;
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updateMap()
{
    //only first
    if(map_raw_ptr_  == nullptr)
    {
        map_raw_ptr_ = boost::make_shared< pcl::PointCloud<PointTarget> >();
        map_filtered_ptr_ = boost::make_shared< pcl::PointCloud<PointTarget> >();

        boost::shared_ptr< pcl::PointCloud<PointTarget> > scan_removed_around_sensor_ptr(new pcl::PointCloud<PointTarget>);
        removePointCloudAroundSensor(scan_raw_ptr_, scan_removed_around_sensor_ptr, 2.0);
        addPointCloud(scan_removed_around_sensor_ptr, map_raw_ptr_, localizer_pose_);
        addPointCloud(scan_removed_around_sensor_ptr, map_filtered_ptr_, localizer_pose_);
        setInputTarget(map_filtered_ptr_);
        is_initial_pose_set_ = true; //TODO
    }

    static Pose added_pose;
    double add_scan_shift_meter = std::sqrt(std::pow(localizer_pose_.x - added_pose.x, 2.0) + std::pow(localizer_pose_.y - added_pose.y, 2.0));
    const double min_add_scan_shift_meter = 1.0;
    if(add_scan_shift_meter >= min_add_scan_shift_meter)
    {
        boost::shared_ptr< pcl::PointCloud<PointTarget> > scan_removed_around_sensor_ptr(new pcl::PointCloud<PointTarget>);
        removePointCloudAroundSensor(scan_raw_ptr_, scan_removed_around_sensor_ptr, 2.0);
        addPointCloud(scan_removed_around_sensor_ptr, map_raw_ptr_, localizer_pose_);
        addPointCloud(scan_removed_around_sensor_ptr, map_filtered_ptr_, localizer_pose_);
        setInputTarget(map_filtered_ptr_);
        added_pose = localizer_pose_;
    }

    //for acceleration of matching
    static Pose filtered_pose;
    double filtering_scan_shift_meter = std::sqrt(std::pow(localizer_pose_.x - filtered_pose.x, 2.0) + std::pow(localizer_pose_.y - filtered_pose.y, 2.0));
    const double min_filtering_scan_shift_meter = 10.0;
    if(filtering_scan_shift_meter >= min_filtering_scan_shift_meter)
    {
        const double filtered_meter = 80.0;
        passThroughPointCloud(map_raw_ptr_, map_filtered_ptr_, localizer_pose_, filtered_meter);

        const double voxel_leaf_size_meter = 0.2;
        pcl::VoxelGrid<PointTarget> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size_meter, voxel_leaf_size_meter, voxel_leaf_size_meter);
        voxel_grid_filter.setInputCloud(map_filtered_ptr_);
        voxel_grid_filter.filter(*map_filtered_ptr_);

        setInputTarget(map_filtered_ptr_);

        filtered_pose = localizer_pose_;
    }
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::writeLogFile()
{
    static std::ofstream log_file_stream;
    static bool is_first_call = true;
    if(is_first_call)
    {
        is_first_call = false;
        char buffer[80];
        const std::time_t now = std::time(NULL);
        const std::tm* pnow = std::localtime(&now);
        std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
        const std::string filename = "ndt_slam_" + std::string(buffer) + ".csv";
        log_file_stream.open(filename.c_str(), std::ios::app);
    }
    if (!log_file_stream)
    {
      std::cerr << "Could not open log file." << std::endl;
      //exit(1);
    }

    log_file_stream << logFileContent() << std::endl;
}

template <class PointSource, class PointTarget>
std::stringstream LibLocalizer<PointSource, PointTarget>::logFileContent() const
{
    std::stringstream content;
    content << localizer_pose_.x << ","
            << localizer_pose_.y << ","
            << localizer_pose_.z << ","
            << localizer_pose_.roll << ","
            << localizer_pose_.pitch << ","
            << localizer_pose_.yaw << ",";
}

#endif
