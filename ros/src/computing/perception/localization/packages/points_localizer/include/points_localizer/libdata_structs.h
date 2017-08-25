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

#ifndef LIBDATA_STRUCTS_H
#define LIBDATA_STRUCTS_H

#include <iostream>
//#include <Eigen/Geometry>

struct Pose
{
  Pose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {};
  Pose(double x, double y, double z, double roll, double pitch, double yaw)
    : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {};
  void clear(){x = y = z = roll = pitch = yaw = 0;};

  Pose operator+(const Pose& rhs_pose) const
  {
    Pose tmp_pose;
    tmp_pose.x = x + rhs_pose.x;
    tmp_pose.y = y + rhs_pose.y;
    tmp_pose.z = z + rhs_pose.z;
    tmp_pose.roll = roll + rhs_pose.roll;
    tmp_pose.pitch = pitch + rhs_pose.pitch;
    tmp_pose.yaw = yaw + rhs_pose.yaw;
    return tmp_pose;
  };

  Pose operator-(const Pose& rhs_pose) const
  {
    Pose tmp_pose;
    tmp_pose.x = x - rhs_pose.x;
    tmp_pose.y = y - rhs_pose.y;
    tmp_pose.z = z - rhs_pose.z;
    tmp_pose.roll = roll - rhs_pose.roll;
    tmp_pose.pitch = pitch - rhs_pose.pitch;
    tmp_pose.yaw = yaw - rhs_pose.yaw;
    return tmp_pose;
  };

  Pose operator*(const double val) const
  {
    Pose tmp_pose;
    tmp_pose.x = x * val;
    tmp_pose.y = y * val;
    tmp_pose.z = z * val;
    tmp_pose.roll = roll * val;
    tmp_pose.pitch = pitch * val;
    tmp_pose.yaw = yaw * val;
    return tmp_pose;
  };

  Pose operator/(const double val) const
  {
    Pose tmp_pose;
    if(val == 0)
      return tmp_pose;

    tmp_pose.x = x / val;
    tmp_pose.y = y / val;
    tmp_pose.z = z / val;
    tmp_pose.roll = roll / val;
    tmp_pose.pitch = pitch / val;
    tmp_pose.yaw = yaw / val;
    return tmp_pose;
  };

  bool operator!=(const Pose& rhs_pose) const
  {
    return !(x == rhs_pose.x && y == rhs_pose.y && z == rhs_pose.z
       && roll == rhs_pose.roll && pitch == rhs_pose.pitch && yaw == rhs_pose.yaw);
  };

  friend std::ostream& operator<<(std::ostream& os, const Pose& pose)
  {
    os << "x:" << pose.x << " y:" << pose.y << " z:" << pose.z << " roll:" << pose.roll/M_PI*180.0 << " pitch:" << pose.pitch/M_PI*180.0 << " yaw:" << pose.yaw/M_PI*180.0;
    return os;
  };

  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

};



struct Linear
{
  Linear() : x(0), y(0), z(0){};
  double x;
  double y;
  double z;

  void clear()
  {
    x = y = z = 0;
  };

  Linear operator+(const Linear& rhs_l) const
  {
    Linear tmp_l;
    tmp_l.x = x + rhs_l.x;
    tmp_l.y = y + rhs_l.y;
    tmp_l.z = z + rhs_l.z;
    return tmp_l;
  }

  Linear operator-(const Linear& rhs_l) const
  {
    Linear tmp_l;
    tmp_l.x = x - rhs_l.x;
    tmp_l.y = y - rhs_l.y;
    tmp_l.z = z - rhs_l.z;
    return tmp_l;
  }

  Linear operator*(const double val) const
  {
    Linear tmp_l;

    if(val == 0)
      return tmp_l;

    tmp_l.x = x * val;
    tmp_l.y = y * val;
    tmp_l.z = z * val;
    return tmp_l;
  }

  Linear operator/(const double val) const
  {
    Linear tmp_l;

    if(val == 0)
      return tmp_l;

    tmp_l.x = x / val;
    tmp_l.y = y / val;
    tmp_l.z = z / val;
    return tmp_l;
  }
};

struct Angular
{
  Angular() : x(0), y(0), z(0){};
  double x;
  double y;
  double z;

  void clear()
  {
    x = y = z = 0;
  };

  Angular operator+(const Angular& rhs_a) const
  {
    Angular tmp_a;
    tmp_a.x = x + rhs_a.x;
    tmp_a.y = y + rhs_a.y;
    tmp_a.z = z + rhs_a.z;
    return tmp_a;
  }

  Angular operator-(const Angular& rhs_a) const
  {
    Angular tmp_a;
    tmp_a.x = x - rhs_a.x;
    tmp_a.y = y - rhs_a.y;
    tmp_a.z = z - rhs_a.z;
    return tmp_a;
  }

  Angular operator*(const double val) const
  {
    Angular tmp_a;
    tmp_a.x = x * val;
    tmp_a.y = y * val;
    tmp_a.z = z * val;
    return tmp_a;
  }

  Angular operator/(const double val) const
  {
    Angular tmp_a;

    if(val == 0)
      return tmp_a;

    tmp_a.x = x / val;
    tmp_a.y = y / val;
    tmp_a.z = z / val;
    return tmp_a;
  }


};

struct Velocity
{
  Velocity() {};


  Velocity(const Pose& previous_pose, const Pose& current_pose, double time_diff_sec)
  {
    if(time_diff_sec == 0)
    {
      clear();
      return;
    }

    const Pose diff_pose = current_pose - previous_pose;

    linear.x = diff_pose.x / time_diff_sec;
    linear.y = diff_pose.y / time_diff_sec;
    linear.z = diff_pose.z / time_diff_sec;
    angular.x = diff_pose.roll / time_diff_sec;
    angular.y = diff_pose.pitch / time_diff_sec;
    angular.z = diff_pose.yaw / time_diff_sec;
  };

  void clear()
  {
    linear.clear();
    angular.clear();
  };

  Velocity operator+(const Velocity& rhs_v) const
  {
    Velocity tmp_v;
    tmp_v.linear = linear + rhs_v.linear;
    tmp_v.angular = angular + rhs_v.angular;
    return tmp_v;
  }

  Velocity operator-(const Velocity& rhs_v) const
  {
    Velocity tmp_v;
    tmp_v.linear = linear - rhs_v.linear;
    tmp_v.angular = angular - rhs_v.angular;
    return tmp_v;
  }

  Velocity operator*(const double val) const
  {
    Velocity tmp_v;
    tmp_v.linear = linear * val;
    tmp_v.angular = angular * val;
    return tmp_v;
  }

  Velocity operator/(const double val) const
  {
    Velocity tmp_v;
    tmp_v.linear = linear / val;
    tmp_v.angular = angular / val;
    return tmp_v;
  }

  Linear linear;
  Angular angular;
};

struct Accel
{
  Accel() {};

  Accel(const Velocity& previous_velocity, const Velocity& current_velocity, double time_diff_sec)
  {
    if(time_diff_sec == 0)
    {
      clear();
      return;
    }

    const Velocity diff_velocity = current_velocity - previous_velocity;
    linear = diff_velocity.linear / time_diff_sec;
    angular = diff_velocity.angular / time_diff_sec;
  };



  void clear()
  {
    linear.clear();
    angular.clear();
  };



  Accel operator+(const Accel& rhs_a) const
  {
    Accel tmp_a;
    tmp_a.linear = linear + rhs_a.linear;
    tmp_a.angular = angular + rhs_a.angular;
    return tmp_a;
  }

  Accel operator-(const Accel& rhs_a) const
  {
    Accel tmp_a;
    tmp_a.linear = linear - rhs_a.linear;
    tmp_a.angular = angular - rhs_a.angular;
    return tmp_a;
  }

  Accel operator*(const double val) const
  {
    Accel tmp_a;
    tmp_a.linear = linear * val;
    tmp_a.angular = angular * val;
    return tmp_a;
  }

  Accel operator/(const double val) const
  {
    Accel tmp_a;
    tmp_a.linear = linear / val;
    tmp_a.angular = angular / val;
    return tmp_a;
  }

  Linear linear;
  Angular angular;

};

struct Imu
{
  Imu() : roll(0), pitch(0), yaw(0){};
  double roll, pitch, yaw;
  Angular angular_velocity;
  Linear linear_accel;
};

// Pose convertPose(const Eigen::Matrix4f& m)
// {
//   Pose pose;
//   pose.x = m(0, 3);
//   pose.y = m(1, 3);
//   pose.z = m(2, 3);
//
//   //this code is written reference to tf::getEulerYPR()
//   if (std::fabs(m(2,0)) >= 1)
//   {
//     pose.yaw = 0;
//     if (m(2,0) < 0)
//     {
//       pose.pitch = M_PI / 2.0;
//       pose.roll = std::atan2(m(0,1),m(0,2));
//     }
//     else
//     {
//       pose.pitch = -M_PI / 2.0;
//       pose.roll = std::atan2(-m(0,1),-m(0,2));
//     }
//   }
//   else
//   {
//     pose.pitch = -std::asin(m(2,0));
//     pose.roll  = std::atan2(m(2,1)/std::cos(pose.pitch),
//                             m(2,2)/std::cos(pose.pitch));
//     pose.yaw   = std::atan2(m(1,0)/std::cos(pose.pitch),
//                             m(0,0)/std::cos(pose.pitch));
//   }
//
//   return pose;
// }
//
//
// Pose transformPose(const Pose& pose, const Eigen::Matrix4f& m)
// {
//   Eigen::Translation3f eigen_xyz(pose.x, pose.y, pose.z);
//   Eigen::AngleAxisf eigen_roll(pose.roll, Eigen::Vector3f::UnitX());
//   Eigen::AngleAxisf eigen_pitch(pose.pitch, Eigen::Vector3f::UnitY());
//   Eigen::AngleAxisf eigen_yaw(pose.yaw, Eigen::Vector3f::UnitZ());
//   Eigen::Matrix4f trans_m = (eigen_xyz * eigen_yaw * eigen_pitch * eigen_roll) * m;
//   Pose trans_pose = convertPose(trans_m);
//
//   return trans_pose;
// }

#endif
