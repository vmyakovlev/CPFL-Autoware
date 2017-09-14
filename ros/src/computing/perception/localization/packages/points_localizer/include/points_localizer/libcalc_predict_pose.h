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

#ifndef LIBCALC_PREDICT_POSE_H
#define LIBCALC_PREDICT_POSE_H

#include <iostream>
#include <deque>
#include <utility>

#include "libdata_structs.h"

class LibCalcPredictPose
{
  public:
    LibCalcPredictPose();
    ~LibCalcPredictPose() {};
    void clear();
    void addData(const double time_stamp, const Pose& data);
    Pose predictNextPose(const double previous_time_sec, const double next_time_sec, const Pose& previous_pose);
  private:
    //first:  time_stamp_sec
    //second: data
    std::deque< std::pair<double, Pose> > queue_;
    const double DataKeepTimeSec_;
};

LibCalcPredictPose::LibCalcPredictPose()
  :DataKeepTimeSec_(5.0)
{
}

void LibCalcPredictPose::clear()
{
  queue_.clear();
}


void LibCalcPredictPose::addData(const double time_stamp_sec, const Pose& data)
{
  //This Code is not support when restarting with changing start time
  if(!queue_.empty() && queue_.front().first >= time_stamp_sec)
  {
    std::cout << "Probably bag file is restarted. Cleared all buffer" << std::endl;
    queue_.clear();
  }

  queue_.push_back( std::pair<double, Pose>(time_stamp_sec, data) );

  while(queue_.back().first - queue_.front().first > DataKeepTimeSec_)
  {
    //std::cout << "pop " << std::fmod(queue_.front(), 100.0) << std::endl;
    queue_.pop_front();
  }
}

Pose LibCalcPredictPose::predictNextPose(const double previous_time_sec, const double next_time_sec, const Pose& previous_pose)
{
  Pose next_pose = previous_pose;

  for(auto it = std::begin(queue_); it != std::end(queue_); ++it)
  {
    if(it != std::begin(queue_) && it->first > next_time_sec)
      break;

    const auto it2 = (it != std::begin(queue_) && it+1 != std::end(queue_)) ? it+1 : it;
    if(it+1 != std::end(queue_) && it2->first < previous_time_sec)
      continue;

    const double begin_time_sec = (it != std::begin(queue_) && it->first > previous_time_sec) ? it->first : previous_time_sec;
    const double end_time_sec   = (it+1 != std::end(queue_) && it2->first < next_time_sec) ? it2->first : next_time_sec;

    // std::cout << std::fmod(it->first, 100.0)
    //   << " " << std::fmod(begin_time_sec, 100.0)
    //   << " " << std::fmod(it2->first, 100.0)
    //   << " " << std::fmod(end_time_sec, 100.0)
    //   << std::endl;

    if(it2->first - it->first == 0)
        continue;
    const double diff_time_ratio = (end_time_sec - begin_time_sec) / (it2->first - it->first);
//    const double diff_time_ratio = 1;

    next_pose.roll  += it2->second.roll * diff_time_ratio;
    next_pose.pitch += it2->second.pitch * diff_time_ratio;
    next_pose.yaw   += it2->second.yaw * diff_time_ratio;

    const double x = it2->second.x * diff_time_ratio;
    const double y = it2->second.y * diff_time_ratio;
    const double z = it2->second.z * diff_time_ratio;
    const double dis = std::abs(x)+std::abs(y)+std::abs(z);

    next_pose.x += dis*cos(-next_pose.pitch)*cos(next_pose.yaw);
    next_pose.y += dis*cos(-next_pose.pitch)*sin(next_pose.yaw);
    next_pose.z += dis*sin(-next_pose.pitch);

    //const double r = next_pose.roll;
    //const double p = next_pose.pitch;
    //const double a = next_pose.yaw;
    //next_pose.x += x*cos(r)*cos(p) + y*cos(r)*sin(p)*sin(a) - y*sin(r)*cos(a) + z*cos(r)*sin(p)*cos(a) + z*sin(r)*sin(a);
    //next_pose.y += x*sin(r)*cos(p) + y*sin(r)*sin(p)*sin(a) + y*cos(r)*cos(a) + z*sin(r)*sin(p)*cos(a) - z*cos(r)*sin(a);
    //next_pose.z += -x*sin(r) + y*cos(p)*sin(a) + z*cos(p)*cos(a);

  }

  return next_pose;
}


#endif
