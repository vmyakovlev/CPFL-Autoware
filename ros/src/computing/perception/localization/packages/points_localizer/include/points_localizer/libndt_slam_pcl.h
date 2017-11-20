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

#ifndef LIBNDT_SLAM_PCL_H
#define LIBNDT_SLAM_PCL_H

#include "libndt_slam_base.h"

#include <thread>
#include <future>
#include <chrono>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

template <class PointSource, class PointTarget>
class LibNdtSlamPCL
    : public LibNdtSlamBase<PointSource, PointTarget>
{
    public:
        LibNdtSlamPCL();
        ~LibNdtSlamPCL() = default;

        void setTransformationEpsilon(double trans_eps) override;
        void setStepSize(double step_size) override;
        void setResolution(float res) override;
        void setMaximumIterations(int max_iter) override;

        double getTransformationEpsilon() override;
        double getStepSize() const override;
        float getResolution() const override;
        int getMaximumIterations() override;
        double getTransformationProbability() const override;

    protected:
        void align(const Pose& predict_pose) override;
        double getFitnessScore() override;
        void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr) override;
        void setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr) override;
        Pose getFinalPose() override;
        bool swapMap() override;
        //bool isSetInputTarget() override;

    private:
        pcl::NormalDistributionsTransform<PointSource, PointTarget> ndt_;
        pcl::NormalDistributionsTransform<PointSource, PointTarget> swap_ndt_;
        std::future<int> thread_ret_;
        bool is_thread_running_;
};

template <class PointSource, class PointTarget>
LibNdtSlamPCL<PointSource, PointTarget>::LibNdtSlamPCL()
    :is_thread_running_(false)
{
}

template <class PointSource, class PointTarget>
void LibNdtSlamPCL<PointSource, PointTarget>::setTransformationEpsilon(double trans_eps)
{
    ndt_.setTransformationEpsilon(trans_eps);
}

template <class PointSource, class PointTarget>
void LibNdtSlamPCL<PointSource, PointTarget>::setStepSize(double step_size)
{
    ndt_.setStepSize(step_size);
}

template <class PointSource, class PointTarget>
void LibNdtSlamPCL<PointSource, PointTarget>::setResolution(float res)
{
    ndt_.setResolution(res);
}

template <class PointSource, class PointTarget>
void LibNdtSlamPCL<PointSource, PointTarget>::setMaximumIterations(int max_iter)
{
    ndt_.setMaximumIterations(max_iter);
}

template <class PointSource, class PointTarget>
double LibNdtSlamPCL<PointSource, PointTarget>::getTransformationEpsilon()
{
    return ndt_.getTransformationEpsilon();
}

template <class PointSource, class PointTarget>
double LibNdtSlamPCL<PointSource, PointTarget>::getStepSize() const
{
    return ndt_.getStepSize();
}

template <class PointSource, class PointTarget>
float LibNdtSlamPCL<PointSource, PointTarget>::getResolution() const
{
    return ndt_.getResolution();
}

template <class PointSource, class PointTarget>
int LibNdtSlamPCL<PointSource, PointTarget>::getMaximumIterations()
{
    return ndt_.getMaximumIterations();
}

template <class PointSource, class PointTarget>
double LibNdtSlamPCL<PointSource, PointTarget>::getTransformationProbability() const
{
    return ndt_.getTransformationProbability();
}

template <class PointSource, class PointTarget>
void LibNdtSlamPCL<PointSource, PointTarget>::align(const Pose& predict_pose)
{
    const auto predict_matrix = convertToEigenMatrix4f(predict_pose);
    pcl::PointCloud<PointSource> output_cloud;
    ndt_.align(output_cloud, predict_matrix);
}

template <class PointSource, class PointTarget>
void LibNdtSlamPCL<PointSource, PointTarget>::setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr)
{
    ndt_.setInputTarget(map_ptr);
}

template <class PointSource, class PointTarget>
void LibNdtSlamPCL<PointSource, PointTarget>::setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr)
{
    ndt_.setInputSource(scan_ptr);
}

template <class PointSource, class PointTarget>
double LibNdtSlamPCL<PointSource, PointTarget>::getFitnessScore()
{
    return ndt_.getFitnessScore();
}

template <class PointSource, class PointTarget>
Pose LibNdtSlamPCL<PointSource, PointTarget>::getFinalPose()
{
    return convertToPose(ndt_.getFinalTransformation());
}

// template <class PointSource, class PointTarget>
// void LibNdtSlamPCL<PointSource, PointTarget>::setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr)
// {
//     //not want to make many threads
//     if(is_thread_running_ == true)
//         return;
//
//     //TODO: wanna make more smart
//     thread_ret_ =  std::async(std::launch::async, [this](boost::shared_ptr< pcl::PointCloud<PointTarget> const> map_ptr){
//         const auto trans_estimation = getTransformationEpsilon();
//         const auto step_size = getStepSize();
//         const auto resolution = getResolution();
//         const auto max_iter = getMaximumIterations();
//
//         pcl::NormalDistributionsTransform<PointSource, PointTarget> tmp_ndt;
//         tmp_ndt.setTransformationEpsilon(trans_estimation);
//         tmp_ndt.setStepSize(step_size);
//         tmp_ndt.setResolution(resolution);
//         tmp_ndt.setMaximumIterations(max_iter);
//         tmp_ndt.setInputTarget(map_ptr);
//
//         const auto identity_matrix = Eigen::Matrix4f::Identity();
//         pcl::PointCloud<PointSource> output_cloud;
//         tmp_ndt.align(output_cloud, identity_matrix);
//
//         swap_ndt_ = tmp_ndt;
//         return 0;
//     }, map_ptr);
//
//     is_thread_running_ = true;
// }

template <class PointSource, class PointTarget>
bool LibNdtSlamPCL<PointSource, PointTarget>::swapMap()
{
    try{
        auto thread_status = thread_ret_.wait_for(std::chrono::milliseconds(0));
        if(is_thread_running_ == true && thread_status == std::future_status::ready)
        {
            ndt_ = swap_ndt_;
            is_thread_running_ = false;
            return true;
        }
    }
    catch(...)
    {
        return false;
    }
    return false;
}

#endif
