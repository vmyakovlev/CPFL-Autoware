/*
 * AlternativeVisualizer.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#include "AlternativeVisualizer.h"
#include <sstream>
#include <algorithm>
#include "UtilityH.h"
#include "DataRW.h"
#include<unistd.h>
#include<math.h>
#include<fcntl.h>
#include <sys/termios.h>
#include <plib/js.h>
#include "autoware_msgs/accel_cmd.h"
#include "autoware_msgs/brake_cmd.h"
#include "autoware_msgs/steer_cmd.h"
#include "autoware_msgs/CanInfo.h"

#define DEV_NAME "/dev/input/js0"

using namespace std;


namespace Graphics
{

AlternativeVisualizer::AlternativeVisualizer()
{
	game_ctrl_mutex = PTHREAD_MUTEX_INITIALIZER;
	game_ctrl_thread_tid = 0;
	m_bCancelThread = false;
	m_bStepForward = false;
	m_bPredStepForward = false;

	ros::NodeHandle nh;
	pub_VehicleCommand		= nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 100);
	pub_VehicleCommandAcc	= nh.advertise<autoware_msgs::accel_cmd>("accel_cmd", 100);
	pub_VehicleCommandBrk	= nh.advertise<autoware_msgs::brake_cmd>("brake_cmd", 100);
	//pub_CanInfo 			= nh.advertise<autoware_msgs::CanInfo>("can_info", 100);
	pub_SimuStepSignal 		= nh.advertise<geometry_msgs::TwistStamped>("simu_step_signal", 1);
	pub_PredStepSignal 		= nh.advertise<geometry_msgs::TwistStamped>("pred_step_signal", 1);

	twist_sub = nh.subscribe("/twist_cmd", 1, &AlternativeVisualizer::twistCMDCallback, this);
	cmd_sub = nh.subscribe("/ctrl_cmd", 1, &AlternativeVisualizer::ctrlCMDCallback, this);
	sub_can_info  = nh.subscribe("/can_info",		1,	&AlternativeVisualizer::callbackGetCanInfo, this);

	m_VehicleTargetStateSpeed = 0;
	m_VehicleTargetStateSteer = 0;
	m_VehicleCurrentStateSpeed = 0;
	m_VehicleCurrentStateSteer = 0;
	m_VehicleTargetStateAccelerator = 0;
	m_VehicleTargetStateBrake = 0;
	m_MaxVelocity = 40.0;
	m_MaxPedalValue = 256;
	m_MaxAccelStroke = 1700;
	m_MaxBrakeStroke = 4095;

	double axes_color[3] = {0.1, 0.1, 0.8};
	double graph_color[3] = {0.9, 0.2, 0.1};
	m_pCurrentVelocityGraph = new Graph2dBase(20, 200,1000, 30, 0, "Car Velocity", "T s", "V km/h", axes_color, graph_color );
	m_pTargetVelocityGraph = new Graph2dBase(20, 200,1000, 30, 0, "Target Velocity", "T s", "V km/h", axes_color, graph_color );
	m_Velocity = 0;

	pthread_create(&game_ctrl_thread_tid, NULL, &AlternativeVisualizer::GameCtrlThreadStaticEntryPoint, this);
}

void AlternativeVisualizer::twistCMDCallback(const geometry_msgs::TwistStamped& msg)
{
	cout << "Recieve Twist Data " << endl;
	m_VehicleTargetStateSpeed = msg.twist.linear.x;
	//m_VehicleTargetStateSteer = msg.twist.angular.z;
}

void AlternativeVisualizer::ctrlCMDCallback(const autoware_msgs::ControlCommandStamped& msg)
{

	//m_VehicleTargetStateSpeed = msg.cmd.linear_velocity;
	m_VehicleTargetStateSteer = msg.cmd.steering_angle;

	cout << "Recieve command Data " << endl;
}

void AlternativeVisualizer::callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg)
{
	m_VehicleCurrentStateSpeed = msg->speed/3.6;
	m_VehicleCurrentStateSteer = msg->angle * 0.45 / 600;
	cout << "Recieve Can Data " << endl;
}

void AlternativeVisualizer::LoadMaterials()
{
}

AlternativeVisualizer::~AlternativeVisualizer()
{
	if(m_pCurrentVelocityGraph)
		delete m_pCurrentVelocityGraph;
	if(m_pTargetVelocityGraph)
		delete m_pTargetVelocityGraph;

	if(m_LogData.size() > 5)
	{
	UtilityHNS::DataRW::WriteLogData(UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::ControlLogFolderName,
			"ControlLog",
			"time,Target_V, Actual_V, Target_ACC_Stroke, Target_BRK_Stroke, Target_Steer,Actual_Steer",
			m_LogData);
	}

	m_bCancelThread = true;
	AlternativeVisualizer* pRet = 0;
	if(game_ctrl_thread_tid>0)
		pthread_join(game_ctrl_thread_tid, (void**)&pRet);
}

bool AlternativeVisualizer::IsInitState()
{
	return false;
}

void AlternativeVisualizer::UpdatePlaneStartGoal(const double& x1,const double& y1, const double& a1, const double& x2,const double& y2, const double& a2)
{
}

void AlternativeVisualizer::AddSimulatedCarPos(const double& x,const double& y, const double& a)
{
}

void AlternativeVisualizer::Reset()
{
}

void AlternativeVisualizer::PrepareVectorMapForDrawing()
{

}

void AlternativeVisualizer::DrawGPSData()
{
}

void AlternativeVisualizer::DrawVectorMap()
{

}

//For HMI work
//void AlternativeVisualizer::DrawSimu()
//{
//
//	glPushMatrix();
//	std::ostringstream str_out ;
//	str_out <<  m_Velocity;
//	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
//	glPopMatrix();
////		glTranslated(centerX-left_shift-15, 70+85, 0);
////		glColor3f(0.8, 0.1, 0.7);
////		std::ostringstream str_out ;
////		str_out.precision(2);
////		str_out <<  m_VehicleCurrentState.steer*RAD2DEG;
////		DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
//
//	ros::Rate loop_rate(100);
//
//	if(ros::ok())
//	{
//		m_Controller.StrokeControl(m_CurrStatus.speed, m_TargetStatus.speed);
//		m_CurrStatus.speed = m_Controller.m_SimulatedSpeed;
//
//		m_Velocity = m_CurrStatus.speed;
//
//		geometry_msgs::Twist t;
//		geometry_msgs::TwistStamped twist;
//		t.linear.x = m_Velocity;
//		t.angular.z = 0;
//		twist.twist = t;
//		twist.header.stamp = ros::Time::now();
//
//		//pub_VehicleCommand.publish(twist);
//
//		autoware_msgs::brake_cmd brk_cmd;
//		autoware_msgs::accel_cmd acc_cmd;
//		acc_cmd.accel = m_VehicleTargetStateAccelerator;
//		brk_cmd.brake = m_VehicleTargetStateBrake;
//		//pub_VehicleCommandAcc.publish(acc_cmd);
//		//pub_VehicleCommandBrk.publish(brk_cmd);
//
//		// Log Testing Data
//		timespec time;
//		UtilityHNS::UtilityH::GetTickCount(time);
//		std::ostringstream dataLine;
//		dataLine << UtilityHNS::UtilityH::GetLongTime(time) << "," <<
//				m_VehicleTargetStateSpeed << "," <<
//				m_VehicleCurrentStateSpeed << "," <<
//				m_VehicleTargetStateAccelerator << "," <<
//				m_VehicleTargetStateBrake << "," <<
//				m_VehicleTargetStateSteer << "," <<
//				m_VehicleCurrentStateSteer << ",";
//		m_LogData.push_back(dataLine.str());
//
//		ros::spinOnce();
//		loop_rate.sleep();
//	}
//}

void AlternativeVisualizer::DrawSimu()
{

	glPushMatrix();
	std::ostringstream str_out ;
	str_out <<  m_Velocity;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
	glPopMatrix();

	ros::Rate loop_rate(15);

	if(ros::ok())
	{
		if(m_bPredStepForward)
		{
			geometry_msgs::TwistStamped s_signal;
			s_signal.header.frame_id = "velodyne";
			s_signal.header.stamp = ros::Time();
			s_signal.twist.linear.x = 1;
			pub_PredStepSignal.publish(s_signal);
			m_bPredStepForward = false;
		}

		if(m_bStepForward)
		{
			geometry_msgs::TwistStamped s_signal;
			s_signal.header.frame_id = "velodyne";
			s_signal.header.stamp = ros::Time();
			s_signal.twist.linear.x = 1;
			pub_SimuStepSignal.publish(s_signal);
			m_bStepForward = false;
			m_bPredStepForward = true;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void AlternativeVisualizer::DrawInfo(const int& centerX, const int& centerY, const int& maxX, const int& maxY)
{
	double left_shift = 150;
	double verticalTranslation = 100;
	double peda_height = 150;


	glDisable(GL_LIGHTING);
	glPushMatrix();
	glTranslated(centerX-left_shift, verticalTranslation, 0);
	glRotated(-1*m_VehicleTargetStateSteer*RAD2DEG*16, 0,0,1);
	glTranslated(-(centerX-left_shift), -verticalTranslation, 0);

	float wheel_color[3] = {0.6, 0.7, 0.8};
	DrawingHelpers::DrawWideEllipse(centerX-left_shift, verticalTranslation, 0.5, 60, 55, 54, wheel_color);

	glColor3f(0.5,0.4, 0.3);
	PlannerHNS::GPSPoint p1(centerX-left_shift, verticalTranslation, 0.52, 0), p2(centerX-left_shift+38, verticalTranslation-38, 0.52, 0);
	DrawingHelpers::DrawLinePoygonline(p1, p2, 5);

	PlannerHNS::GPSPoint p11(centerX-left_shift, verticalTranslation, 0.52, 0), p22(centerX-left_shift-38, verticalTranslation-38, 0.52, 0);
	DrawingHelpers::DrawLinePoygonline(p11, p22, 5);

	PlannerHNS::GPSPoint p111(centerX-left_shift, verticalTranslation, 0.52, 0), p222(centerX-left_shift, verticalTranslation+52, 0.52, 0);
	DrawingHelpers::DrawLinePoygonline(p111, p222, 5);
	glPopMatrix();
	glEnable(GL_LIGHTING);

	glPushMatrix();
	glTranslated(centerX-left_shift-10, verticalTranslation+85, 0);
	glColor3f(0.8, 0.1, 0.7);
	std::ostringstream str_out ;
	str_out.precision(2);
	str_out <<  m_VehicleTargetStateSteer*RAD2DEG;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out.str().c_str());
	glPopMatrix();

	double speed = m_VehicleTargetStateSpeed*3.6 * (peda_height/m_MaxVelocity);
	float pedal_color[3] = {0.5,0.4, 0.3};
	glColor3f(wheel_color[0],wheel_color[1],wheel_color[2]);
	DrawingHelpers::DrawPedal(centerX + 100 - left_shift, verticalTranslation, 0, 30.0, peda_height, speed,pedal_color );

	glPushMatrix();
	glTranslated(centerX+95 - left_shift, verticalTranslation+100, 0);
	glColor3f(0.8, 0.1, 0.7);
	std::ostringstream v_out ;
	v_out.precision(3);
	v_out << "T: ";
	v_out <<  speed;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)v_out.str().c_str());
	glPopMatrix();

	int accPedal = m_VehicleTargetStateAccelerator * (peda_height/m_MaxAccelStroke);
	float pedal_color_acc[3] = {0.1,0.9, 0.2};
	glColor3f(pedal_color_acc[0],pedal_color_acc[1],pedal_color_acc[2]);
	DrawingHelpers::DrawPedal(centerX + 100 - left_shift + 100, verticalTranslation, 0, 30.0, peda_height, accPedal,pedal_color_acc );

	glPushMatrix();
	glTranslated(centerX+95 - left_shift + 100, verticalTranslation+100, 0);
	glColor3f(0.8, 0.1, 0.7);
	std::ostringstream v_out_acc ;
	v_out_acc.precision(3);
	v_out_acc << "Acc: ";
	v_out_acc <<  accPedal;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)v_out_acc.str().c_str());
	glPopMatrix();

	int brakePedal = m_VehicleTargetStateBrake * (peda_height/m_MaxBrakeStroke);
	float pedal_color_brake[3] = {0.9,0.1, 0.2};
	glColor3f(pedal_color_brake[0],pedal_color_brake[1],pedal_color_brake[2]);
	DrawingHelpers::DrawPedal(centerX + 100 - left_shift + 210, verticalTranslation, 0, 30.0, peda_height, brakePedal,pedal_color_brake );

	glPushMatrix();
	glTranslated(centerX+95 - left_shift + 210, verticalTranslation+100, 0);
	glColor3f(0.8, 0.1, 0.7);
	std::ostringstream v_out_brake ;
	v_out_brake.precision(3);
	v_out_brake << "Brk: ";
	v_out_brake <<  brakePedal;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)v_out_brake.str().c_str());
	glPopMatrix();

	verticalTranslation +=120;
	glPushMatrix();
	glTranslated(10, verticalTranslation, 0);
	double axes_color[3] = {0.1, 0.1, 0.8};
	double graph_color[3] = {0.9, 0.2, 0.1};
	m_pTargetVelocityGraph->ReInitGraphResolution(maxX-20, 200,1000, axes_color, graph_color );
	struct timespec tStamp;
	UtilityHNS::UtilityH::GetTickCount(tStamp);
	m_pTargetVelocityGraph->InsertPointTimeStamp(tStamp, m_VehicleTargetStateSpeed*3.6);
	m_pTargetVelocityGraph->DrawGraph();
	glPopMatrix();


	verticalTranslation+=300;
	glDisable(GL_LIGHTING);
	glPushMatrix();
	glTranslated(centerX-left_shift, verticalTranslation, 0);
	glRotated(-1*m_VehicleCurrentStateSteer*RAD2DEG*16, 0,0,1);
	glTranslated(-(centerX-left_shift), -70, 0);

	//wheel_color[3] = {0.6, 0.7, 0.8};
	DrawingHelpers::DrawWideEllipse(centerX-left_shift, 70, 0.5, 60, 55, 54, wheel_color);

	glColor3f(0.5,0.4, 0.3);
	p1 = PlannerHNS::GPSPoint(centerX-left_shift, 70, 0.52, 0);
	p2 = PlannerHNS::GPSPoint(centerX-left_shift+38, 70-38, 0.52, 0);
	DrawingHelpers::DrawLinePoygonline(p1, p2, 5);

	p11 = PlannerHNS::GPSPoint(centerX-left_shift, 70, 0.52, 0);
	p22 = PlannerHNS::GPSPoint(centerX-left_shift-38, 70-38, 0.52, 0);
	DrawingHelpers::DrawLinePoygonline(p11, p22, 5);

	p111 = PlannerHNS::GPSPoint(centerX-left_shift, 70, 0.52, 0);
	p222 = PlannerHNS::GPSPoint(centerX-left_shift, 70+52, 0.52, 0);
	DrawingHelpers::DrawLinePoygonline(p111, p222, 5);
	glPopMatrix();
	glEnable(GL_LIGHTING);

	glPushMatrix();
	glTranslated(centerX-left_shift-10, verticalTranslation+85, 0);
	glColor3f(0.8, 0.1, 0.7);
	std::ostringstream str_out_curr ;
	str_out_curr.precision(3);
	str_out_curr <<  m_VehicleTargetStateSteer*RAD2DEG;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)str_out_curr.str().c_str());
	glPopMatrix();

	speed = m_VehicleCurrentStateSpeed*3.6* (peda_height/m_MaxVelocity);
	//float pedal_color[3] = {0.5,0.4, 0.3};
	glColor3f(wheel_color[0],wheel_color[1],wheel_color[2]);
	DrawingHelpers::DrawPedal(centerX + 100 - left_shift, verticalTranslation, 0, 30.0, peda_height, speed,pedal_color );

	glPushMatrix();
	glTranslated(centerX+95 - left_shift, verticalTranslation+100, 0);
	glColor3f(0.8, 0.1, 0.7);
	std::ostringstream v_out_curr ;
	v_out_curr.precision(3);
	v_out_curr << "A: ";
	v_out_curr <<  speed;
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)v_out_curr.str().c_str());
	glPopMatrix();

	verticalTranslation+=120;
	glPushMatrix();
	glTranslated(10, verticalTranslation, 0);
	//double axes_color[3] = {0.1, 0.1, 0.8};
	//double graph_color[3] = {0.9, 0.2, 0.1};
	m_pCurrentVelocityGraph->ReInitGraphResolution(maxX-20, 200,1000, axes_color, graph_color );
	m_pCurrentVelocityGraph->InsertPointTimeStamp(tStamp, m_VehicleCurrentStateSpeed*3.6);
	m_pCurrentVelocityGraph->DrawGraph();
	glPopMatrix();
}

void AlternativeVisualizer::OnLeftClick(const double& x, const double& y)
{}

void AlternativeVisualizer::OnRightClick(const double& x, const double& y)
{}

void AlternativeVisualizer::OnKeyboardPress(const SPECIAL_KEYS_TYPE& sKey, const unsigned char& key)
{
	//std::cout << "key" << std::endl;

	switch(key)
	{
	case 's':
	{
		m_bStepForward = true;
	}
	break;
	case 'v':
	{
	}
	break;
	case 'l':
	{
	}
	break;
	case 'n':
	{
	}
	break;
	case 'g':
	{
	}
	break;
	case '+':
	{
		if(m_Velocity < 10)
			m_Velocity++;
		else
			m_Velocity = 10;
	}
	break;
	case '-':
	{
		if(m_Velocity > 0)
			m_Velocity--;
		else
			m_Velocity = 0;
	}
	break;

	default:
		break;

	}
}


void* AlternativeVisualizer::GameCtrlThreadStaticEntryPoint(void* pThis)
{

	AlternativeVisualizer* pR = (AlternativeVisualizer*)pThis;
	int joy_fd = 0;
	static unsigned int nSize = 16;
	char data [nSize];
	for(unsigned int i=0; i < nSize; i++ )
		data[i] = 0;

	if(( joy_fd = open(DEV_NAME, O_RDONLY, 0)) == -1)
	{
		fprintf(stderr,"Can't open %s\n",DEV_NAME);
		return 0;
	}

	tcflush(joy_fd,TCIOFLUSH);

	while(!pR->m_bCancelThread)
	{
		//pthread_mutex_lock(&pR->game_ctrl_mutex);

		if(read(joy_fd,data,8)>0)
		{
		      data[8] = 0;
		      /*      printf("%d %d %d %d %d %d %d %d\n",
			     data[0],data[1],data[2],data[3],
			     data[4],data[5],data[6],data[7]);
		      */
		      if(data[7]==5)
		      {
		    	  printf("l %d\n",data[5]);
		    	  pR->m_VehicleTargetStateAccelerator = data[5]+128;
		    	  pR->m_VehicleTargetStateAccelerator = pR->m_VehicleTargetStateAccelerator*pR->m_MaxAccelStroke/pR->m_MaxPedalValue;
		    	  pR->SimulateAcceleration();
		      }

		      if(data[7]==2)
		      {
		    	  printf("r %d\n",data[5]);
		    	  pR->m_VehicleTargetStateBrake = data[5]+128;
		    	  pR->m_VehicleTargetStateBrake = pR->m_VehicleTargetStateBrake*pR->m_MaxBrakeStroke/pR->m_MaxPedalValue;
		    	  pR->SimulateBraking();
		      }

		}

		//pthread_mutex_unlock(&pR->game_ctrl_mutex);

		usleep(100);
	}
	cout << "Exit Game Wheel Loop." << endl;
	return pR;
}

void AlternativeVisualizer::SimulateAcceleration()
{
	m_CurrStatus.speed+=0.1;
	if(m_CurrStatus.speed > m_MaxVelocity)
		m_CurrStatus.speed = m_MaxVelocity;
}

void AlternativeVisualizer::SimulateBraking()
{
	m_CurrStatus.speed-=0.15;
	if(m_CurrStatus.speed < 0 )
		m_CurrStatus.speed = 0;
}

} /* namespace Graphics */
