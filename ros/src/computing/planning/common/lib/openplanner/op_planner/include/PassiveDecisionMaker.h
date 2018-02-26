/*
 * CarState.h
 *
 *  Created on: Dec 14, 2016
 *      Author: hatem
 */

#ifndef PASSIVE_BEHAVIOR_DECISION_MAKER
#define PASSIVE_BEHAVIOR_DECISION_MAKER

#include "BehaviorStateMachine.h"
#include "PlannerCommonDef.h"
#include "RoadNetwork.h"

namespace PlannerHNS
{

class PassiveDecisionMaker
{
public:
	WayPoint state;
	CAR_BASIC_INFO m_CarInfo;
	ControllerParams m_ControlParams;
	PlannerHNS::RoadNetwork m_Map;

	double m_MaxLaneSearchDistance;

	Lane* pLane;

	BehaviorStateMachine* 		m_pCurrentBehaviorState;
	StopState* 					m_pStopState;
	WaitState* 					m_pWaitState;
	SwerveStateII*				m_pAvoidObstacleState;
	TrafficLightStopState*		m_pTrafficLightStopState;
	TrafficLightWaitState*		m_pTrafficLightWaitState;

	ForwardStateII * 			m_pGoToGoalState;;
	InitStateII* 				m_pInitState;
	MissionAccomplishedStateII*	m_pMissionCompleteState;
	GoalStateII*				m_pGoalState;
	FollowStateII*				m_pFollowState;
	StopSignStopStateII* 		m_pStopSignStopState;
	StopSignWaitStateII* 		m_pStopSignWaitState;

	void InitBehaviorStates(const BehaviorState& start_beh);

	//For Simulation
	UtilityHNS::PIDController 	m_pidVelocity;
	UtilityHNS::PIDController 	m_pidStopping;
	UtilityHNS::PIDController 	m_pidSteer;

public:

	PassiveDecisionMaker();
	PassiveDecisionMaker(const PassiveDecisionMaker& obj);
	PassiveDecisionMaker& operator=(const PassiveDecisionMaker& obj);
	virtual ~PassiveDecisionMaker();
	void Init(const ControllerParams& ctrlParams, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const BehaviorState& start_beh);
	void CalculateImportantParameterForDecisionMaking(const double& vel, const std::vector<TrafficLight>& detectedLightsy);
	BehaviorState DoOneStep(const double& dt, const PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const std::vector<TrafficLight>& trafficLight, PlannerHNS::VehicleState& u_control_status);

private:
	bool GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<TrafficLight>& trafficLights, TrafficLight& trafficL);
	void UpdateCurrentLane(const double& search_distance);
	BehaviorState GenerateBehaviorState();
	double UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const double& vel, const double& dt);
	double  GetSteerAngle();


private:
	std::vector<PlannerHNS::WayPoint> t_centerTrajectorySmoothed;
	PlannerHNS::PlanningParams m_params;
	std::vector<WayPoint> m_Path;
	std::vector<std::vector<WayPoint> > m_Paths;

};

} /* namespace PlannerHNS */

#endif /* PASSIVE_BEHAVIOR_DECISION_MAKER */
