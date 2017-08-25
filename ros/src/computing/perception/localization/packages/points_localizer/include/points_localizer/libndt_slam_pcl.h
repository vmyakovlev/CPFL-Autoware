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

#ifndef LIBNDT_SLMA_PCL_H
#define LIBNDT_SLMA_PCL_H

#include <iostream>
#include <string>
#include <chrono>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#ifdef USE_FAST_PCL
#include <fast_pcl/registration/ndt.h>
#else
#include <pcl/registration/ndt.h>
#endif

#include "libdata_structs.h"
#include "libconvert_ros_msgs.h"
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
        void updateCurrentVelocity(const double current_data_time_sec, const Velocity& velocity);
        void updatePointsRaw(const pcl::PointCloud<PointT>& pointcloud);
        void updatePointsFused(const pcl::PointCloud<PointT>& pointcloud);
        void updatePointsFiltered(const pcl::PointCloud<PointT>& pointcloud);
        void updateLocalizer(double current_scan_time_sec);

        Pose getLocalizerPose() const;
        Velocity getLocalizerVelocity() const;

    protected:
        virtual void align(const Pose& predict_pose) = 0;
        virtual void setInputTarget(const pcl::PointCloud<PointT>::ConstPtr& map_ptr) = 0;
        virtual void setInputSource(const pcl::PointCloud<PointT>::ConstPtr& scan_ptr) = 0;
        virtual double getFitnessScore() = 0;
        virtual Pose getFinalPose() const = 0;

    private:
        Pose localizer_pose_;
        Velocity localizer_velocity_;

        LibCalcPredictPose calc_predict_pose_;
};

template <class PointT>
LibLocalizer<PointT>::LibLocalizer()
{

}

template <class PointT>
void LibLocalizer<PointT>::LibLocalizer(const pcl::PointCloud<PointT>& pointcloud)
{
    const pcl::PointCloud<PointT>::ConstPtr map_ptr(new pcl::PointCloud<PointT>(pointcloud));
    setInputTarget(pointcloud_ptr);
}

template <class PointT>
void LibLocalizer<PointT>::updateManualPose(const Pose& pose)
{
  localizer_pose_ = pose;
}

template <class PointT>
void LibLocalizer<PointT>::updateStaticPose(const Pose& pose)
{
  localizer_pose_ = pose;
}

template <class PointT>
void LibLocalizer<PointT>::updateGnssPose(const Pose& pose)
{
  localizer_pose_ = pose;
}

template <class PointT>
void LibLocalizer<PointT>::updateCurrentVelocity(const double current_data_time_sec, const Velocity& velocity)
{
    calc_predict_pose_.add(current_data_time_sec, velocity);
}

template <class PointT>
void LibLocalizer<PointT>::updatePointsRaw(const pcl::PointCloud<PointT>& pointcloud)
{
  //create map
}

template <class PointT>
void LibLocalizer<PointT>::updatePointsFused(const pcl::PointCloud<PointT>& pointcloud)
{
    //create map
}

template <class PointT>
void LibLocalizer<PointT>::updatePointsFiltered(const pcl::PointCloud<PointT>& pointcloud)
{
    const pcl::PointCloud<PointT>::ConstPtr scan_ptr(new pcl::PointCloud<PointT>(pointcloud));
    setInputSource(scan_ptr);
}

template <class PointT>
void LibLocalizer<PointT>::updateLocalizer(double current_scan_time_sec)
{
    std::cout << __func__ << std::endl;

    static double previous_scan_time_sec = current_scan_time_sec;

    const auto predict_pose = calc_predict_pose_.predictNextPose(previous_scan_time_sec, current_scan_time_sec, localizer_pose_);

    const auto align_start = std::chrono::system_clock::now();
    align(predict_pose);
    const auto align_end = std::chrono::system_clock::now();
    const auto align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

    const auto calc_fitness_score_start = std::chrono::system_clock::now();
    const auto fitness_score = getFitnessScore();
    const auto calc_fitness_score_end = std::chrono::system_clock::now();
    const auto calc_fitness_score_time = std::chrono::duration_cast<std::chrono::microseconds>(calc_fitness_score_end - calc_fitness_score_start).count() / 1000.0;

    localizer_pose_ = getFinalPose();
    localizer_velocity_ = Velocity(previous_scan_time_sec, current_scan_time_sec, localizer_pose_);

    previous_scan_time_sec = current_scan_time_sec;
}

Pose LibLocalizer<PointT>::getLocalizerPose() const
{
    return localizer_pose_;
}

Velocity LibLocalizer<PointT>::getLocalizerVelocity() const
{
    return localizer_velocity_;
}


template <class PointT>
class LibNdtSlam : public LibLocalizer<PointT>
{
    public:
        LibNdtSlam();
        LibNdtSlam(float res, double trans_eps, double step_size, double max_iter);
        ~LibNdtSlam(){};

        void setResolution(float res);
        void setTransformationEpsilon(double trans_eps);
        void setStepSize(double step_size);
        void setMaximumIterations(double max_iter);

        float getResolution() const;
        double getTransformationEpsilon() const;
        double getStepSize() const;
        double getMaximumIterations() const;

    protected:
        void align() override;
        void setInputTarget(const pcl::PointCloud<PointT>::ConstPtr& map_ptr) override;
        void setInputSource(const pcl::PointCloud<PointT>::ConstPtr& scan_ptr) override;
        double getFitnessScore() override;
        Pose getFinalPose() const override;

    private:
        pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
};

template <class PointT>
LibNdtSlam<PointT>::LibNdtSlam()
{
}

template <class PointT>
LibNdtSlam<PointT>::LibNdtSlam(float res, double trans_eps, double step_size, double max_iter)
{
    setResolution(res);
    setTransformationEpsilon(trans_eps);
    setStepSize(step_size);
    setMaximumIterations(max_iter);
}

template <class PointT>
void LibNdtSlam<PointT>::setResolution(float res)
{
    ndt_.setResolution(res);
}

template <class PointT>
void LibNdtSlam<PointT>::setTransformationEpsilon(double trans_eps)
{
    ndt_.setTransformationEpsilon(trans_eps);
}

template <class PointT>
void LibNdtSlam<PointT>::setStepSize(double step_size)
{
    ndt_.setStepSize(step_size);
}

template <class PointT>
void LibNdtSlam<PointT>::setMaximumIterations(double max_iter)
{
    ndt_.setMaximumIterations(max_iter);
}

template <class PointT>
float LibNdtSlam<PointT>::getResolution() const
{
    return ndt_.getResolution();
}

template <class PointT>
double LibNdtSlam<PointT>::getTransformationEpsilon() const
{
    return ndt_.getTransformationEpsilon();
}

template <class PointT>
double LibNdtSlam<PointT>::getStepSize() const
{
    return ndt_.getStepSize();
}

template <class PointT>
double LibNdtSlam<PointT>::getMaximumIterations() const
{
    return ndt_.getMaximumIterations();
}

template <class PointT>
void LibNdtSlam<PointT>::align(const Pose& predict_pose)
{
    pcl::PointCloud<PointT> output_cloud;
    const auto predict_matrix = convertToEigenMatrix4f(predict_pose);
    pcl::PointCloud<PointT> output_cloud;
    #ifdef USE_FAST_PCL
      if (use_openmp_ == true)
        ndt_.omp_align(output_cloud, predict_matrix);
    #endif
      else
        ndt_.align(output_cloud, predict_matrix);
}

template <class PointT>
void LibNdtSlam<PointT>::setInputTarget(const pcl::PointCloud<PointT>::ConstPtr& map_ptr)
{
    ndt_.setInputTarget(map_ptr);
}

template <class PointT>
void LibNdtSlam<PointT>::setInputSource(const pcl::PointCloud<PointT>::ConstPtr& scan_ptr)
{
    ndt_.setInputSource(scan_ptr);
}

template <class PointT>
double LibNdtSlam<PointT>::getFitnessScore()
{
    #ifdef USE_FAST_PCL
          if (use_openmp_ == true)
            return ndt_.omp_getFitnessScore();
          else
    #endif
            return ndt_.getFitnessScore();

}

template <class PointT>
Pose LibNdtSlam<PointT>::getFinalPose() const
{
    return convertPose(ndt_.getFinalTransformation());
}

#endif
