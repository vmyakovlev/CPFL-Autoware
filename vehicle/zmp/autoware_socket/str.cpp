/*
 *  Copyright (c) 2015, Nagoya University
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

#include "mainwindow.h"
#include "autoware_socket.h"

static double steering_diff_sum = 0;
queue<double> steering_diff_buffer;

static int ignore_str_flag = 1;

static void clear_diff()
{
  int i;

  steering_diff_sum = 0;

  for (i = 0; i < (int) steering_diff_buffer.size(); i++) {
    steering_diff_buffer.pop();
  }
}

void MainWindow::SetStrMode(int mode)
{
  switch (mode) {
  case CMD_MODE_MANUAL:
    cout << "Switching to MANUAL (Steering)" << endl;
    ZMP_SET_STR_MANUAL();
    break;
  case CMD_MODE_PROGRAM:
    cout << "Switching to PROGRAM (Steering)" << endl;
    ignore_str_flag = 1;
    ZMP_SET_STR_PROGRAM();
    clear_diff();
    break;
  default:
    cout << "Unknown mode: " << mode << endl;
  }
}

// positive torque: left/right
// nagative torque: left/right
double _str_torque_pid_control(double current_steering_angle, double cmd_steering_angle)
{
  double e;
  static double e_prev = 0xffffffff;
  double e_i;
  double e_d;
  double ret;

  // adjust the steering angle error (default offset).
  current_steering_angle -= _STEERING_ANGLE_ERROR;

  double steering_diff = cmd_steering_angle - current_steering_angle; 
  steering_diff_sum += steering_diff;

  if (steering_diff_sum > _STEERING_MAX_SUM) {
    steering_diff_sum = _STEERING_MAX_SUM;
  }
  if (steering_diff_sum < -_STEERING_MAX_SUM) {
    steering_diff_sum = -_STEERING_MAX_SUM;
  }

  // e: proportional error
  // e_i: integral error
  // e_d: differential error
  e = steering_diff;
  e_i = steering_diff_sum;

  if (e_prev == 0xffffffff) {
    e_prev = e;
  }

  e_d = (e - e_prev) / (STEERING_INTERNAL_PERIOD/1000.0);

  double k_p = _K_STEERING_P;
  double k_i = _K_STEERING_I;
  double k_d = _K_STEERING_D;

  double steering_max_torque = _STEERING_MAX_TORQUE;

  // torque control
  static double target_steering_torque = 0;

#if 0 // DEBUG configuration
  printf("accel_p = %lf, accel_i = %lf, accel_d = %lf\n",
         shm_ptr->steer.P, shm_ptr->steer.I, shm_ptr->steer.D);
  k_p = shm_ptr->steer.P;
  k_i = shm_ptr->steer.I;
  k_d = shm_ptr->steer.D;
#endif

  // if e is small, we have to move faster (more sensitive).
  if (fabs(e) < _STEERING_DIFF_SMALL) {
    k_p = _K_STEERING_P_STRICT;
    k_i = _K_STEERING_I_STRICT;
    k_d = _K_STEERING_D_STRICT;
  }

  // if we are slow on the straight line, do lazy control.
  if (fabs(e) < 0.3 && fabs(cmd_steering_angle) < 3 && vstate.velocity < 10) {
    // don't accumulate error.
    steering_diff_sum = 0;
    // skip PID control...
    if (e > 0) {
      target_steering_torque = 10;
    }
    else {
      target_steering_torque = -10;
    }
  }
  else {
    target_steering_torque = e * k_p + e_i * k_i + e_d * k_d;
  }

  // clip
  if (target_steering_torque > steering_max_torque) {
    target_steering_torque = steering_max_torque;
  }
  if (target_steering_torque < -steering_max_torque) {
    target_steering_torque = -steering_max_torque;
  }

  ret = target_steering_torque;

#if 1 /* log */ 
  ofstream ofs("/tmp/steering.log", ios::app);
  ofs << cmd_steering_angle << " " 
      << current_steering_angle << " " 
      << e << " "  
      << e_i << " "  
      << steering_diff_sum << " "//shm_ptr->steer.D * e_d << " "
      << (e - e_prev) / (STEERING_INTERNAL_PERIOD/1000.0) << " "
      << target_steering_torque << endl;
#endif

  e_prev = e;

  return ret;
}

void MainWindow::SteeringControl(double current_steering_angle, double cmd_steering_angle)
{
  double torque;

  // positive {angle,torque} == left
  // negative {angle,torque} == right

  // don't control if not in program mode.
  if (!ZMP_STR_CONTROLLED()) {
    clear_diff();
    return;
  }

#if 1
  //------------ ANGLE CONTROL --------------
  // set PID factors for ZMP control box
  int kp, ki, kd;
#if 1
  if (vstate.velocity < _K_ANGLE_THRESHOLD_SLOW) {
    kp = _K_ANGLE_CTRL_P_SLOW;
    ki = _K_ANGLE_CTRL_I_SLOW;
    kd = _K_ANGLE_CTRL_D_SLOW;
  } else if (vstate.velocity < _K_ANGLE_THRESHOLD) {
    kp = _K_ANGLE_CTRL_P_SLOW +
      (_K_ANGLE_CTRL_P - _K_ANGLE_CTRL_P_SLOW) *
      (vstate.velocity - _K_ANGLE_THRESHOLD_SLOW) /
      (_K_ANGLE_THRESHOLD - _K_ANGLE_THRESHOLD_SLOW);
    ki = _K_ANGLE_CTRL_I_SLOW +
      (_K_ANGLE_CTRL_I - _K_ANGLE_CTRL_I_SLOW) *
      (vstate.velocity - _K_ANGLE_THRESHOLD_SLOW) /
      (_K_ANGLE_THRESHOLD - _K_ANGLE_THRESHOLD_SLOW);
    kd = _K_ANGLE_CTRL_D_SLOW +
      (_K_ANGLE_CTRL_D - _K_ANGLE_CTRL_D_SLOW) *
      (vstate.velocity - _K_ANGLE_THRESHOLD_SLOW) /
      (_K_ANGLE_THRESHOLD - _K_ANGLE_THRESHOLD_SLOW);
  } else {
    kp = _K_ANGLE_CTRL_P;
    ki = _K_ANGLE_CTRL_I;
    kd = _K_ANGLE_CTRL_D;
  }
#else
  if (vstate.velocity > _K_ANGLE_THRESHOLD_SLOW) {
    kp = _K_ANGLE_CTRL_P;
    ki = _K_ANGLE_CTRL_I;
    kd = _K_ANGLE_CTRL_D;
  } else {
    kp = _K_ANGLE_CTRL_P_SLOW;
    ki = _K_ANGLE_CTRL_I_SLOW;
    kd = _K_ANGLE_CTRL_D_SLOW;
  }
#endif
    hev->SetControlGain(CONFIG_ANGLE_CTRL_KP, kp);
    hev->SetControlGain(CONFIG_ANGLE_CTRL_KI, ki);
    hev->SetControlGain(CONFIG_ANGLE_CTRL_KD, kd);
#ifdef USE_ANGLE_DIFF_THRESHOLD
  static double old_angle = cmd_steering_angle;
  static double old_angle_diff = 0.0;
  double angle_diff = cmd_steering_angle - old_angle;
  if (angle_diff - old_angle_diff > _ANGLE_DIFF_THRESHOLD) {
    cmd_steering_angle = _ANGLE_DIFF_THRESHOLD + old_angle + old_angle_diff;
    angle_diff = cmd_steering_angle - old_angle;
  } else if (angle_diff - old_angle_diff < -_ANGLE_DIFF_THRESHOLD) {
    cmd_steering_angle = -_ANGLE_DIFF_THRESHOLD + old_angle + old_angle_diff;
    angle_diff = cmd_steering_angle - old_angle;
  }
#endif /* USE_ANGLE_DIFF_THRESHOLD */
  static int str_limit_cnt = 0;
  static double old_angle = cmd_steering_angle;
  if (vstate.velocity >= _K_ANGLE_THRESHOLD_VERY_SLOW || !ignore_str_flag) {
    if (ignore_str_flag) {
      str_limit_cnt = _STEERING_LIMIT_SEC * 1000/STEERING_INTERNAL_PERIOD;
      old_angle = cmd_steering_angle;
    }
    ignore_str_flag = 0;
  }
  if (str_limit_cnt > 0) {
    str_limit_cnt--;
    if (cmd_steering_angle - old_angle > _STEERING_LIMIT_ANGLE) {
      cmd_steering_angle = old_angle + _STEERING_LIMIT_ANGLE;
    } else if (cmd_steering_angle - old_angle < -_STEERING_LIMIT_ANGLE) {
      cmd_steering_angle = old_angle - _STEERING_LIMIT_ANGLE;
    }
    old_angle = cmd_steering_angle;
  }
  if (cmd_steering_angle < -STEERING_ANGLE_MAX) {
    cmd_steering_angle = -STEERING_ANGLE_MAX;
  } else if (cmd_steering_angle > STEERING_ANGLE_MAX) {
    cmd_steering_angle = STEERING_ANGLE_MAX;
  }
  ZMP_SET_STR_ANGLE((cmd_steering_angle - _STEERING_ANGLE_ERROR)*10);
#if 1 /* log */
  ofstream ofs("/tmp/angle.log", ios::app);
  ofs << cmd_steering_angle << " "
      << current_steering_angle << " "
      << kp << " "
      << ki << " "
      << kd << endl;
#endif
#ifdef USE_ANGLE_DIFF_THRESHOLD
#if 1 /* log */
  ofstream dofs("/tmp/anglediff.log", ios::app);
  dofs << (angle_diff - old_angle_diff) << " "
      << angle_diff << " "
      << cmd_steering_angle << " "
      << current_steering_angle << endl;
#endif
  old_angle = cmd_steering_angle;
  old_angle_diff = angle_diff;
#endif /* USE_ANGLE_DIFF_THRESHOLD */
  return;
  //-----------------------------------------
#endif

  //----------- TORQUE CONTROL -------------
  if (vstate.velocity < 3) { // if nearly at stop, don't control steering.
    torque = 0;
  }
  else {
    torque = _str_torque_pid_control(current_steering_angle, cmd_steering_angle);
  }
  cout << "ZMP_SET_STR_TORQUE(" << torque << ")" << endl;
  ZMP_SET_STR_TORQUE(torque);
  //--------------------------------------
}
