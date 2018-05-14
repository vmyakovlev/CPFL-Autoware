/*
 *  Copyright (c) 2018, TierIV, inc.
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

#include "accel.h"

#include <iostream>

static double limit(double val, double max_limit, double min_limit)
{
    double ret = val;
    if(val > max_limit)
      ret = max_limit;
    else if(val < min_limit)
      ret = min_limit;
    return ret;
}

Accel::Accel()
    : target_velocity(0)
    , current_velocity(0)
{

}

void Accel::setTargetVelocity(const double target_velocity)
{
  this->target_velocity = target_velocity;
}

void Accel::setCurrentVelocity(const double current_velocity)
{
  this->current_velocity = current_velocity;
}

double Accel::computeAccel() const
{
  const double jerk_limit = 0.2;
  const double accel_limit = 2.0;
  const double dt = 0.3;
  static double accel = 0;

  double v = current_velocity;
  double a = accel;
  if(target_velocity - current_velocity > 0)
  {
      bool is_acceleration = false;

      while(1)
      {
          v += a;
          if(v > target_velocity)
          {
            is_acceleration = false;
            break;
          }
          a -= jerk_limit;
          if(a <= 0)
          {
            is_acceleration = true;
            break;
          }
      }
      if(is_acceleration)
         accel += jerk_limit*dt;
      else
        accel -= jerk_limit*dt;
  }
  else
  {
      bool is_deceleration = false;
      while(1)
      {
          v += a;
          if(v < target_velocity)
          {
            is_deceleration = false;
            break;
          }
          a += jerk_limit;
          if(a >= 0)
          {
            is_deceleration = true;
            break;
          }
      }
      if(is_deceleration)
         accel -= jerk_limit*dt;
      else
        accel += jerk_limit*dt;
  }
  accel = limit(accel, accel_limit, -accel_limit);
  std::cout << current_velocity << " " << target_velocity << " " << accel << std::endl;
  return accel;
}
