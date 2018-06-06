/*
 * SimulatedController.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: hatem
 */

#include "SimulatedController.h"
#include <iostream>
#include <cmath>

using namespace std;

SimulatedController::SimulatedController()
{
	m_estimate_accel = 0.0;
	m_cycle_time = 0.01;
	m_vstate_brake_stroke = 0;
	m_vstate_accel_stroke = 0;
	m_SimulatedSpeed = 0;
}

SimulatedController::~SimulatedController()
{
}

void SimulatedController::StrokeControl(double current_velocity, double cmd_velocity)
{

  static unsigned int vel_buffer_size = 10;
  double old_velocity = 0.0;

  // estimate current acceleration.
  m_vel_buffer.push(current_velocity);
  if (m_vel_buffer.size() > vel_buffer_size)
  {
    old_velocity = m_vel_buffer.front();
    m_vel_buffer.pop(); // remove old_velocity from the queue.
    m_estimate_accel = (current_velocity-old_velocity)/(m_cycle_time*vel_buffer_size);
  }

  //cout << "estimate_accel: " << m_estimate_accel << endl;
  double speed_diff = current_velocity - cmd_velocity;

  if (fabs(cmd_velocity) > current_velocity && fabs(cmd_velocity) > 0.0 && current_velocity < SPEED_LIMIT)
  {
    //cout << "accelerate: current_velocity=" << current_velocity << ", cmd_velocity=" << cmd_velocity << endl;

    double accel_stroke = Accel_stroke_pid_control(current_velocity, cmd_velocity);
    if (accel_stroke > 0)
    {
    //  cout << "ZMP_SET_DRV_STROKE(" << accel_stroke << ")" << endl;
      //ZMP_SET_DRV_STROKE(accel_stroke);
      if(m_SimulatedSpeed < SPEED_LIMIT)
    	  m_SimulatedSpeed++;

    }
    else
    {
      cout << "ZMP_SET_DRV_STROKE(0)" << endl;
      //ZMP_SET_DRV_STROKE(0);
      if(m_SimulatedSpeed > 0)
		  m_SimulatedSpeed--;
      //cout << "ZMP_SET_BRAKE_STROKE(" << -accel_stroke << ")" << endl;
      //ZMP_SET_BRAKE_STROKE(-accel_stroke);
    }
  }
  else if (fabs(cmd_velocity) < current_velocity && fabs(cmd_velocity) > 0.0)
  {
    double brake_stroke;
    //cout << "decelerate: current_velocity=" << current_velocity
         //<< ", cmd_velocity=" << cmd_velocity << endl;
    brake_stroke = Brake_stroke_pid_control(current_velocity, cmd_velocity);
    if (brake_stroke > 0)
    {
      //cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
    	if(m_SimulatedSpeed > 0)
    		m_SimulatedSpeed--;
      //ZMP_SET_BRAKE_STROKE(brake_stroke);
    }
    else
    {
    	if(m_SimulatedSpeed > 0 )
    		m_SimulatedSpeed--;
      //cout << "ZMP_SET_BRAKE_STROKE(0)" << endl;
      //ZMP_SET_BRAKE_STROKE(0);
      //cout << "ZMP_SET_DRV_STROKE(" << -brake_stroke << ")" << endl;
      //ZMP_SET_DRV_STROKE(-brake_stroke);
    }
  }
  else if (cmd_velocity == 0.0 && current_velocity != 0.0)
  {
    double brake_stroke;
//    cout << "stopping: current_velocity=" << current_velocity
//         << ", cmd_velocity=" << cmd_velocity << endl;
    if (current_velocity < 3.0)
    { // nearly stopping
      //ZMP_SET_DRV_STROKE(0);
      brake_stroke = Stopping_control(current_velocity);
      //cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
      if(m_SimulatedSpeed > 0)
    	  m_SimulatedSpeed--;
      //ZMP_SET_BRAKE_STROKE(brake_stroke);
    }
    else
    {
      brake_stroke = Brake_stroke_pid_control(current_velocity, 0);
      if(m_SimulatedSpeed > 0)
    	  m_SimulatedSpeed--;
      if (brake_stroke > 0)
      {
     //   cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
        //ZMP_SET_BRAKE_STROKE(brake_stroke);
      }
      else
      {
       // cout << "ZMP_SET_DRV_STROKE(0)" << endl;
        //ZMP_SET_DRV_STROKE(0);
       // cout << "ZMP_SET_DRV_STROKE(" << -brake_stroke << ")" << endl;
        //ZMP_SET_DRV_STROKE(-brake_stroke);
      }
    }
  }
  else
  {
    cout << "unknown: current_velocity=" << current_velocity
         << ", cmd_velocity=" << cmd_velocity << endl;
  }
}

double SimulatedController::Accel_stroke_pid_control(double current_velocity, double cmd_velocity)
{
  double e;
  static double e_prev = 0;
  double e_i;
  double e_d;
  double ret;

  // acclerate by releasing the brake pedal if pressed.
  if (m_vstate_brake_stroke > _BRAKE_PEDAL_OFFSET)
  {
    ret = 0;
    /* reset PID variables. */
    e_prev = 0;
    Clear_diff();
  }
  else
  { // PID control
    double target_accel_stroke;

    e = cmd_velocity - current_velocity;

    e_d = e - e_prev;

    accel_diff_sum += e;
    accel_diff_buffer.push(e);
    if (accel_diff_buffer.size() > _K_ACCEL_I_CYCLES) {
      double e_old = accel_diff_buffer.front();
      accel_diff_sum -= e_old;
      if (accel_diff_sum < 0) {
        accel_diff_sum = 0;
      }
      accel_diff_buffer.pop();
    }

    if (accel_diff_sum > _ACCEL_MAX_I) {
      e_i = _ACCEL_MAX_I;
    }
    else {
      e_i = accel_diff_sum;
    }

    target_accel_stroke = _K_ACCEL_P * e + _K_ACCEL_I * e_i + _K_ACCEL_D * e_d;
    if (target_accel_stroke > _ACCEL_PEDAL_MAX) {
      target_accel_stroke = _ACCEL_PEDAL_MAX;
    }
    else if (target_accel_stroke < 0) {
      target_accel_stroke = 0;
    }

    cout << "e = " << e << endl;
    cout << "e_i = " << e_i << endl;
    cout << "e_d = " << e_d << endl;

    ret = target_accel_stroke;

    e_prev = e;
  }

  return ret;
}

double SimulatedController::Brake_stroke_pid_control(double current_velocity, double cmd_velocity)
{
  double e;
  static double e_prev = 0;
  double e_i;
  double e_d;
  double ret;

  // decelerate by releasing the accel pedal if pressed.
  if (m_vstate_accel_stroke > _ACCEL_PEDAL_OFFSET)
  {
    ret = 0;

    /* reset PID variables. */
    e_prev = 0;
    Clear_diff();
  }
  else { // PID control
    double target_brake_stroke;

    // since this is braking, multiply -1.
    e = -1 * (cmd_velocity - current_velocity);

    e_d = e - e_prev;

    brake_diff_sum += e;
    brake_diff_buffer.push(e);
    if (brake_diff_buffer.size() > _K_BRAKE_I_CYCLES) {
      double e_old = brake_diff_buffer.front();
      brake_diff_sum -= e_old;
      if (brake_diff_sum < 0) {
        brake_diff_sum = 0;
      }
      brake_diff_buffer.pop();
    }

    if (brake_diff_sum > _BRAKE_MAX_I) {
      e_i = _BRAKE_MAX_I;
    }
    else {
      e_i = brake_diff_sum;
    }

    target_brake_stroke = _K_BRAKE_P * e + _K_BRAKE_I * e_i + _K_BRAKE_D * e_d;
    if (target_brake_stroke > _BRAKE_PEDAL_MAX) {
      target_brake_stroke = _BRAKE_PEDAL_MAX;
    }
    else if (target_brake_stroke < 0) {
      target_brake_stroke = 0;
    }

    if (target_brake_stroke - m_vstate_brake_stroke > _BRAKE_STROKE_DELTA_MAX) {
      target_brake_stroke = m_vstate_brake_stroke + _BRAKE_STROKE_DELTA_MAX;
    }

    cout << "e = " << e << endl;
    cout << "e_i = " << e_i << endl;
    cout << "e_d = " << e_d << endl;

    ret = target_brake_stroke;

    e_prev = e;

  }

  return ret;
}

double SimulatedController::Stopping_control(double current_velocity)
{
  double ret;
  static double old_brake_stroke = _BRAKE_PEDAL_MED;

  // decelerate by using brake
  if (current_velocity < 0.1) {
    // nearly at stop -> apply full brake. brake_stroke should reach BRAKE_PEDAL_MAX in one second.
    int gain = (int)(((double)_BRAKE_PEDAL_MAX)*m_cycle_time);
    ret = old_brake_stroke + gain;
    if ((int)ret > _BRAKE_PEDAL_MAX)
      ret = _BRAKE_PEDAL_MAX;
    old_brake_stroke = ret;
  }
  else {
    /*
    // one second is approximately how fast full brakes applied in sharp stop
    int gain = (int)(((double)_BRAKE_PEDAL_MED)*cycle_time);
    ret = vstate.brake_stroke + gain;
    if ((int)ret > _BRAKE_PEDAL_MED)
      ret = _BRAKE_PEDAL_MED;
    */

    // vstate has some delay until applying the current state.
    // perhaps we can just set BRAKE_PEDAL_MED to avoid deceleration delay.
    ret = _BRAKE_PEDAL_MED;
    old_brake_stroke = _BRAKE_PEDAL_MED;
  }

  return ret;
}

void SimulatedController::Clear_diff()
{
  int i;

  accel_diff_sum = 0;
  brake_diff_sum = 0;

  for (i = 0; i < (int) accel_diff_buffer.size(); i++) {
    accel_diff_buffer.pop();
  }
  for (i = 0; i < (int) brake_diff_buffer.size(); i++) {
    brake_diff_buffer.pop();
  }
}
