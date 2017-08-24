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

#ifndef LIBDEAD_REKONER_H
#define LIBDEAD_REKONER_H

#include "libdata_structs.h"

class LibDeadRekoner
{
    public:
        LibDeadRekoner();
        void setUseImuFlag(const bool use_imu);
        Pose calcDeltaPose(const double current_time_sec, const Velocity& arg_velocity);
        Pose calcDeltaPose(const double current_time_sec,  const Imu& imu);

    private:
        Pose calcDeltaPoseCommon(const double current_time_sec);

        Velocity veloticy_;
        bool use_imu_;
};

LibDeadRekoner::LibDeadRekoner()
    :use_imu_(false)
{
}

inline void LibDeadRekoner::setUseImuFlag(const bool use_imu)
{
    use_imu_ = use_imu;
}

Pose LibDeadRekoner::calcDeltaPose(const double current_time_sec, const Velocity& arg_velocity)
{
    if(use_imu_)
        veloticy_.linear = arg_velocity.linear;
    else
        veloticy_ = arg_velocity;

    Pose delta_pose = calcDeltaPoseCommon(current_time_sec);
    return delta_pose;
}

Pose LibDeadRekoner::calcDeltaPose(const double current_time_sec, const Imu& imu)
{
    if(use_imu_)
      veloticy_.angular = imu.angular_velocity;

    Pose delta_pose = calcDeltaPoseCommon(current_time_sec);
    return delta_pose;

}

Pose LibDeadRekoner::calcDeltaPoseCommon(const double current_time_sec)
{
    static double previous_time_sec = current_time_sec;
    const double time_diff_sec = current_time_sec - previous_time_sec;
    previous_time_sec = current_time_sec;

    Pose delta_pose;
    delta_pose.x = veloticy_.linear.x * time_diff_sec;
    delta_pose.y = veloticy_.linear.y * time_diff_sec;
    delta_pose.z = veloticy_.linear.z * time_diff_sec;
    delta_pose.roll  = veloticy_.angular.x * time_diff_sec;
    delta_pose.pitch = veloticy_.angular.y * time_diff_sec;
    delta_pose.yaw   = veloticy_.angular.z * time_diff_sec;

    return delta_pose;
}

#endif
