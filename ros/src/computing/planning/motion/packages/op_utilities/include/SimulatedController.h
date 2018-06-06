/*
 * SimulatedController.h
 *
 *  Created on: Sep 14, 2017
 *      Author: hatem
 */

#ifndef SIMULATEDCONTROLLER_H_
#define SIMULATEDCONTROLLER_H_

#include <queue>

#define SPEED_LIMIT 80

// accel/brake parameters
//#define _K_ACCEL_P 30.0
#define _K_ACCEL_P 10.0
#define _K_ACCEL_I 1.0
//#define _K_ACCEL_D 2.0
#define _K_ACCEL_D 1.0
#define _K_ACCEL_I_CYCLES 100
#define _ACCEL_MAX_I 600
#define _ACCEL_STROKE_DELTA_MAX 1000
#define _ACCEL_RELEASE_STEP 400
#define _ACCEL_PEDAL_MAX 1700
#define _ACCEL_PEDAL_OFFSET 200

//#define _K_BRAKE_P 40.0
#define _K_BRAKE_P 20.0
//#define _K_BRAKE_I 10.0
#define _K_BRAKE_I 2.0
//#define _K_BRAKE_D 10.0
#define _K_BRAKE_D 1.0
#define _K_BRAKE_I_CYCLES 100
#define _BRAKE_MAX_I 200
#define _BRAKE_STROKE_DELTA_MAX 1000
#define _BRAKE_RELEASE_STEP 500
#define _BRAKE_PEDAL_MAX 4095
#define _BRAKE_PEDAL_MED 3200
#define _BRAKE_PEDAL_OFFSET 1000

class SimulatedController
{
public:
	SimulatedController();
	virtual ~SimulatedController();

	void StrokeControl(double current_velocity, double cmd_velocity);
	double Accel_stroke_pid_control(double current_velocity, double cmd_velocity);
	double Brake_stroke_pid_control(double current_velocity, double cmd_velocity);
	double Stopping_control(double current_velocity);
	void Clear_diff();

	double m_SimulatedSpeed;

protected:
	std::queue<double> m_vel_buffer;
	double m_estimate_accel;
	double m_cycle_time;
	double m_vstate_brake_stroke;
	double m_vstate_accel_stroke;

	double accel_diff_sum = 0;
	double brake_diff_sum = 0;

	std::queue<double> accel_diff_buffer;
	std::queue<double> brake_diff_buffer;

};

#endif /* SIMULATEDCONTROLLER_H_ */
