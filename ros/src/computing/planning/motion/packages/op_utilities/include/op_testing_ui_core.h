/*
 * AlternativeVisualizer.h
 *
 *  Created on: Jun 17, 2016
 *      Author: hatem
 */

#ifndef OP_TESTING_CORE
#define OP_TESTING_CORE

#include <iostream>
#include "RoadNetwork.h"
#include "DrawObjBase.h"
#include "DrawingHelpers.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/CanInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include "SimulatedController.h"
#include "Graph2dBase.h"
#include <rosbag/bag.h>
#include "BagTopicPlayer.h"

namespace OP_TESTING_NS
{

#define USE_GAME_CONTROLER_

class BagReaderParams
{
public:
	std::string fileName;
	std::string lidarTopic;
	std::string poseTopic;
	std::string imageTopic;

	std::string lidarTopic_pub;
	std::string poseTopic_pub;
	std::string imageTopic_pub;
};

enum TESTING_MODE {SIMULATION_MODE, ROSBAG_MODE, LIVE_MODE};
enum ROSBAG_PLAY_MODE {PLAY_FORWARD, PLAY_BACKWARD, PLAY_PAUSE, PLAY_STEP_FORWARD, PLAY_STEP_BACKWARD };

class TestingUI : public DrawObjBase
{
public:
	TestingUI();
	virtual ~TestingUI();
	void InitNode(const BagReaderParams& params, const int& mode);

	void DrawSimu();
	void DrawInfo(const int& centerX, const int& centerY, const int& maxX, const int& maxY);
	void OnLeftClick(const double& x, const double& y);
	void OnRightClick(const double& x, const double& y);
	void OnKeyboardPress(const int& sKey, const unsigned char& key);

    void twistCMDCallback(const geometry_msgs::TwistStamped& msg);
    void ctrlCMDCallback(const autoware_msgs::ControlCommandStamped& msg);
    void callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg);

    void SimulationModeMainLoop();

#ifdef USE_GAME_CONTROLER
    bool m_bCancelThread;
	pthread_mutex_t game_ctrl_mutex;
	pthread_t game_ctrl_thread_tid;
    static void* GameCtrlThreadStaticEntryPoint(void* pThis);
#endif

    void SimulateAcceleration();
    void SimulateBraking();

    double m_VehicleTargetStateAccelerator;
    double m_VehicleTargetStateBrake;

    bool m_bStepForward;
    bool m_bPredStepForward;
    bool m_bGenerateSignal;

    double m_VehicleTargetStateSpeed;
    double m_VehicleTargetStateSteer;
    double m_VehicleCurrentStateSpeed;
    double m_VehicleCurrentStateSteer;

    ros::Publisher pub_VehicleCommand;
    ros::Publisher pub_VehicleCommandAcc;
    ros::Publisher pub_VehicleCommandBrk;
    ros::Publisher pub_CanInfo;
    ros::Publisher pub_SimuStepSignal;
    ros::Publisher pub_SimuGenSignal;
    ros::Publisher pub_PredStepSignal;

    ros::Subscriber twist_sub;
    ros::Subscriber cmd_sub ;
    ros::Subscriber sub_can_info;
    int m_Velocity;
    double m_MaxVelocity;
    double m_MaxPedalValue;
    double m_MaxAccelStroke;
    double m_MaxBrakeStroke;

    std::vector<std::string>    m_LogData;

    Graph2dBase* m_pCurrentVelocityGraph;
    Graph2dBase* m_pTargetVelocityGraph;

	PlannerHNS::VehicleState m_TargetStatus;
	PlannerHNS::VehicleState m_CurrStatus;

	SimulatedController m_Controller;

	TESTING_MODE m_TestMode;

	//Rosbag reader
private:
	BagReaderParams m_BagParams;
	rosbag::Bag m_bag;
	std::string m_ReadingInfo;

	bool m_bBagOpen;
	ROSBAG_PLAY_MODE m_PlayMode;
	bool m_bStepDone;

	geometry_msgs::PoseStampedPtr m_pLatestPose;
	sensor_msgs::PointCloud2Ptr m_pLatestCloud;
	sensor_msgs::ImagePtr m_pLatestImage;

	ros::Publisher pub_Point_Raw;
	ros::Publisher pub_Image_Raw;
	ros::Publisher pub_NDT_pose;

	UtilityHNS::BagTopicPlayer<sensor_msgs::PointCloud2> m_CloudReader;
	UtilityHNS::BagTopicPlayer<sensor_msgs::Image> m_ImageReader;
	UtilityHNS::BagTopicPlayer<geometry_msgs::PoseStamped> m_PoseReader;


	bool OpenRosBag();
	void BagReaderModeMainLoop();
	bool ReadNextFrame();
	bool ReadPrevFrame();
};

} /* namespace Graphics */

#endif /* AlternativeVisualizer_H_ */
