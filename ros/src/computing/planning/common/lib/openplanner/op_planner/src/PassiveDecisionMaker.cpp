/*
 * CarState.cpp
 *
 *  Created on: Jun 20, 2016
 *      Author: hatem
 */

#include "PassiveDecisionMaker.h"
#include "UtilityH.h"
#include "PlanningHelpers.h"
#include "MappingHelpers.h"
#include "MatrixOperations.h"


namespace PlannerHNS
{

PassiveDecisionMaker::PassiveDecisionMaker()
{
	pLane = 0;
	m_MaxLaneSearchDistance = 3.0;

	m_pCurrentBehaviorState = nullptr;
	m_pStopState = nullptr;
	m_pMissionCompleteState =nullptr;
	m_pGoalState = nullptr;
	m_pGoToGoalState=nullptr;
	m_pWaitState =nullptr;
	m_pInitState =nullptr;
	m_pFollowState=nullptr;
	m_pAvoidObstacleState=nullptr;
	m_pTrafficLightStopState=nullptr;
	m_pTrafficLightWaitState=nullptr;
	m_pStopSignWaitState=nullptr;
	m_pStopSignStopState=nullptr;
}

PassiveDecisionMaker& PassiveDecisionMaker::operator=(const PassiveDecisionMaker& obj)
{
	if(m_pStopState != nullptr)
		{
			delete m_pStopState;
			m_pStopState = nullptr;
		}

		if(m_pMissionCompleteState  != nullptr)
		{
			delete m_pMissionCompleteState ;
			m_pMissionCompleteState= nullptr;
		}

		if(m_pGoalState != nullptr)
		{
			delete m_pGoalState;
			m_pGoalState= nullptr;
		}

		if(m_pGoToGoalState != nullptr)
		{
			delete m_pGoToGoalState;
			m_pGoToGoalState= nullptr;
		}

		if(m_pWaitState != nullptr)
		{
			delete m_pWaitState;
			m_pWaitState= nullptr;

		}

		if(m_pInitState != nullptr)
		{
			delete m_pInitState;
			m_pInitState= nullptr;
		}

		if(m_pFollowState != nullptr)
		{
			delete m_pFollowState;
			m_pFollowState= nullptr;
		}

		if(m_pAvoidObstacleState != nullptr)
		{
			delete m_pAvoidObstacleState;
			m_pAvoidObstacleState= nullptr;
		}

		if(m_pTrafficLightStopState != nullptr)
		{

			delete m_pTrafficLightStopState;
			m_pTrafficLightStopState= nullptr;
		}

		if(m_pTrafficLightWaitState != nullptr)
		{
			delete m_pTrafficLightWaitState;
			m_pTrafficLightWaitState= nullptr;
		}

		if(m_pStopSignWaitState != nullptr)
		{
			delete m_pStopSignWaitState;
			m_pStopSignWaitState= nullptr;
		}

		if(m_pStopSignStopState != nullptr)
		{
			delete m_pStopSignStopState;
			m_pStopSignStopState= nullptr;
		}

		pLane = 0;
		m_MaxLaneSearchDistance = obj.m_MaxLaneSearchDistance;

		m_CarInfo = obj.m_CarInfo;
		m_ControlParams = obj.m_ControlParams;
		m_params = obj.m_params;

		m_pidVelocity.Init(0.01, 0.004, 0.01);
		m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

		m_pidStopping.Init(0.05, 0.05, 0.1);
		m_pidStopping.Setlimit(m_params.horizonDistance, 0);

		if(obj.m_pCurrentBehaviorState != nullptr)
		{
			BehaviorState start_beh;
			start_beh.state = obj.m_pCurrentBehaviorState->m_Behavior;
			InitBehaviorStates(start_beh);
			*(m_pCurrentBehaviorState->GetCalcParams()) = *(obj.m_pCurrentBehaviorState->GetCalcParams());
		}

		return *this;
}

PassiveDecisionMaker::PassiveDecisionMaker(const PassiveDecisionMaker& obj)
{
	m_pCurrentBehaviorState = nullptr;
	m_pStopState = nullptr;
	m_pMissionCompleteState =nullptr;
	m_pGoalState = nullptr;
	m_pGoToGoalState=nullptr;
	m_pWaitState =nullptr;
	m_pInitState =nullptr;
	m_pFollowState=nullptr;
	m_pAvoidObstacleState=nullptr;
	m_pTrafficLightStopState=nullptr;
	m_pTrafficLightWaitState=nullptr;
	m_pStopSignWaitState=nullptr;
	m_pStopSignStopState=nullptr;

	pLane = 0;
	m_MaxLaneSearchDistance = obj.m_MaxLaneSearchDistance;

	m_CarInfo = obj.m_CarInfo;
	m_ControlParams = obj.m_ControlParams;
	m_params = obj.m_params;

	m_pidVelocity.Init(0.01, 0.004, 0.01);
	m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

	m_pidStopping.Init(0.05, 0.05, 0.1);
	m_pidStopping.Setlimit(m_params.horizonDistance, 0);


	m_pidSteer.Init(m_ControlParams.Steering_Gain.kP, m_ControlParams.Steering_Gain.kI, m_ControlParams.Steering_Gain.kD); // for 3 m/s
	m_pidSteer.Setlimit(m_CarInfo.max_steer_angle, -m_CarInfo.max_steer_angle);

	BehaviorState start_beh;
	start_beh.state = obj.m_pCurrentBehaviorState->m_Behavior;
	InitBehaviorStates(start_beh);
	*m_pCurrentBehaviorState->GetCalcParams() = *obj.m_pCurrentBehaviorState->GetCalcParams();
}

PassiveDecisionMaker::~PassiveDecisionMaker()
{
	if(m_pStopState != nullptr)
	{
		delete m_pStopState;
		m_pStopState = nullptr;
	}

	if(m_pMissionCompleteState  != nullptr)
	{
		delete m_pMissionCompleteState ;
		m_pMissionCompleteState= nullptr;
	}

	if(m_pGoalState != nullptr)
	{
		delete m_pGoalState;
		m_pGoalState= nullptr;
	}

	if(m_pGoToGoalState != nullptr)
	{
		delete m_pGoToGoalState;
		m_pGoToGoalState= nullptr;
	}

	if(m_pWaitState != nullptr)
	{
		delete m_pWaitState;
		m_pWaitState= nullptr;

	}

	if(m_pInitState != nullptr)
	{
		delete m_pInitState;
		m_pInitState= nullptr;
	}

	if(m_pFollowState != nullptr)
	{
		delete m_pFollowState;
		m_pFollowState= nullptr;
	}

	if(m_pAvoidObstacleState != nullptr)
	{
		delete m_pAvoidObstacleState;
		m_pAvoidObstacleState= nullptr;
	}

	if(m_pTrafficLightStopState != nullptr)
	{

		delete m_pTrafficLightStopState;
		m_pTrafficLightStopState= nullptr;
	}

	if(m_pTrafficLightWaitState != nullptr)
	{
		delete m_pTrafficLightWaitState;
		m_pTrafficLightWaitState= nullptr;
	}

	if(m_pStopSignWaitState != nullptr)
	{
		delete m_pStopSignWaitState;
		m_pStopSignWaitState= nullptr;
	}

	if(m_pStopSignStopState != nullptr)
	{
		delete m_pStopSignStopState;
		m_pStopSignStopState= nullptr;
	}
}

void PassiveDecisionMaker::Init(const ControllerParams& ctrlParams, const PlannerHNS::PlanningParams& params,const CAR_BASIC_INFO& carInfo, const BehaviorState& start_beh)
 	{
 		m_CarInfo = carInfo;
 		m_ControlParams = ctrlParams;
 		m_params = params;

 		m_pidVelocity.Init(0.01, 0.004, 0.01);
		m_pidVelocity.Setlimit(m_params.maxSpeed, 0);

		m_pidStopping.Init(0.05, 0.05, 0.1);
		m_pidStopping.Setlimit(m_params.horizonDistance, 0);


		m_pidSteer.Init(m_ControlParams.Steering_Gain.kP, m_ControlParams.Steering_Gain.kI, m_ControlParams.Steering_Gain.kD); // for 3 m/s
		m_pidSteer.Setlimit(m_CarInfo.max_steer_angle, -m_CarInfo.max_steer_angle);

		InitBehaviorStates(start_beh);

//		if(m_pCurrentBehaviorState != nullptr)
//		{
//			m_pCurrentBehaviorState->SetBehaviorsParams(&m_params);
//		}
 	}



void PassiveDecisionMaker::InitBehaviorStates(const BehaviorState& start_beh)
{

	if(m_pCurrentBehaviorState == nullptr)
	{
		m_pStopState 				= new StopState(&m_params, 0, 0);
		m_pMissionCompleteState 	= new MissionAccomplishedStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), 0);
		m_pGoalState				= new GoalStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pMissionCompleteState);
		m_pGoToGoalState 			= new ForwardStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoalState);
		m_pInitState 				= new InitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);

	//	m_pWaitState 				= new WaitState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
		m_pFollowState				= new FollowStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
		m_pAvoidObstacleState		= new SwerveStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	//	m_pTrafficLightStopState	= new TrafficLightStopState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
	//	m_pTrafficLightWaitState	= new TrafficLightWaitState(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
		m_pStopSignWaitState		= new StopSignWaitStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pGoToGoalState);
		m_pStopSignStopState		= new StopSignStopStateII(m_pStopState->m_pParams, m_pStopState->GetCalcParams(), m_pStopSignWaitState);

	//	m_pGoToGoalState->InsertNextState(m_pStopState);
	//	m_pGoToGoalState->InsertNextState(m_pWaitState);
	//	m_pGoToGoalState->InsertNextState(m_pTrafficLightStopState);

		m_pGoToGoalState->InsertNextState(m_pAvoidObstacleState);
		m_pGoToGoalState->InsertNextState(m_pStopSignStopState);
		m_pGoToGoalState->InsertNextState(m_pFollowState);
		m_pGoToGoalState->decisionMakingCount = m_params.nReliableCount;

		m_pGoalState->InsertNextState(m_pGoToGoalState);

		m_pStopState->InsertNextState(m_pGoToGoalState);
	//
	//	m_pTrafficLightStopState->InsertNextState(m_pTrafficLightWaitState);
	//	m_pTrafficLightWaitState->InsertNextState(m_pTrafficLightStopState);
	//
		m_pStopSignWaitState->decisionMakingTime = 5.0;
		m_pStopSignWaitState->InsertNextState(m_pStopSignStopState);
		m_pStopSignWaitState->InsertNextState(m_pGoalState);
	//
	//	m_pFollowState->InsertNextState(m_pStopState);
	//	m_pFollowState->InsertNextState(m_pTrafficLightStopState);

		m_pFollowState->InsertNextState(m_pAvoidObstacleState);
		m_pFollowState->InsertNextState(m_pStopSignStopState);
		m_pFollowState->InsertNextState(m_pGoalState);
		m_pFollowState->decisionMakingCount = m_params.nReliableCount;


		m_pInitState->decisionMakingCount = m_params.nReliableCount;

		if(start_beh.state == PlannerHNS::FORWARD_STATE)
			m_pCurrentBehaviorState = m_pGoToGoalState;
		else if(start_beh.state == PlannerHNS::STOP_SIGN_STOP_STATE)
			m_pCurrentBehaviorState = m_pStopSignStopState;
		else if(start_beh.state == PlannerHNS::STOP_SIGN_WAIT_STATE)
			m_pCurrentBehaviorState = m_pStopSignWaitState;
		else if(start_beh.state == PlannerHNS::FOLLOW_STATE)
			m_pCurrentBehaviorState = m_pFollowState;
		else if(start_beh.state == PlannerHNS::STOPPING_STATE)
			m_pCurrentBehaviorState = m_pStopState;
		else
			m_pCurrentBehaviorState = m_pInitState;
	}
}

 bool PassiveDecisionMaker::GetNextTrafficLight(const int& prevTrafficLightId, const std::vector<PlannerHNS::TrafficLight>& trafficLights, PlannerHNS::TrafficLight& trafficL)
 {
	 for(unsigned int i = 0; i < trafficLights.size(); i++)
	 {
		 double d = hypot(trafficLights.at(i).pos.y - state.pos.y, trafficLights.at(i).pos.x - state.pos.x);
		 if(d <= trafficLights.at(i).stoppingDistance)
		 {
			 double a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(UtilityHNS::UtilityH::FixNegativeAngle(trafficLights.at(i).pos.a) , UtilityHNS::UtilityH::FixNegativeAngle(state.pos.a));

			 if(a_diff < M_PI_2 && trafficLights.at(i).id != prevTrafficLightId)
			 {
				 //std::cout << "Detected Light, ID = " << trafficLights.at(i).id << ", Distance = " << d << ", Angle = " << trafficLights.at(i).pos.a*RAD2DEG << ", Car Heading = " << state.pos.a*RAD2DEG << ", Diff = " << a_diff*RAD2DEG << std::endl;
				 trafficL = trafficLights.at(i);
				 return true;
			 }
		 }
	 }

	 return false;
 }

 void PassiveDecisionMaker::CalculateImportantParameterForDecisionMaking(const double& vel,  const std::vector<TrafficLight>& detectedLights)
 {
	 if(m_Path.size() == 0) return;

 	PreCalculatedConditions* pValues = m_pCurrentBehaviorState->GetCalcParams();

 	pValues->minStoppingDistance = -pow(vel, 2)/(m_CarInfo.max_deceleration);
 	pValues->iCentralTrajectory		= m_pCurrentBehaviorState->m_pParams->rollOutNumber/2;

	if(pValues->iPrevSafeTrajectory < 0)
		pValues->iPrevSafeTrajectory = pValues->iCentralTrajectory;

 	pValues->stoppingDistances.clear();
 	pValues->currentVelocity 		= vel;
 	pValues->bTrafficIsRed 			= false;
 	pValues->currentTrafficLightID 	= -1;
 	pValues->bFullyBlock 			= false;

 	pValues->distanceToNext = m_params.horizonDistance;
 	pValues->velocityOfNext = m_params.maxSpeed;

 	pValues->iCurrSafeTrajectory = 0;
	pValues->bFullyBlock = false;
 	pValues->iCurrSafeLane = 0;

 	double critical_long_front_distance =  m_CarInfo.wheel_base/2.0 + m_CarInfo.length/2.0 + m_params.verticalSafetyDistance;

	pValues->currentGoalID = -1;

 	int stopLineID = -1;
 	int stopSignID = -1;
 	int trafficLightID = -1;
 	double distanceToClosestStopLine = 0;
 	bool bGreenTrafficLight = true;


  	distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(m_Path, state, 0, stopLineID, stopSignID, trafficLightID) - critical_long_front_distance;

 	if(distanceToClosestStopLine > 0 && distanceToClosestStopLine < pValues->minStoppingDistance)
 	{
 		if(m_pCurrentBehaviorState->m_pParams->enableTrafficLightBehavior)
 		{
 			pValues->currentTrafficLightID = trafficLightID;
 			//cout << "Detected Traffic Light: " << trafficLightID << endl;
 			for(unsigned int i=0; i< detectedLights.size(); i++)
 			{
 				if(detectedLights.at(i).id == trafficLightID)
 					bGreenTrafficLight = (detectedLights.at(i).lightState == GREEN_LIGHT);
 			}
 		}

 		if(m_pCurrentBehaviorState->m_pParams->enableStopSignBehavior)
 			pValues->currentStopSignID = stopSignID;

		pValues->stoppingDistances.push_back(distanceToClosestStopLine);
		//std::cout << "LP => D: " << pValues->distanceToStop() << ", PrevSignID: " << pValues->prevTrafficLightID << ", CurrSignID: " << pValues->currentTrafficLightID << ", Green: " << bGreenTrafficLight << endl;
 	}


 	//std::cout << "Distance To Closest: " << distanceToClosestStopLine << ", Stop LineID: " << stopLineID << ", Stop SignID: " << stopSignID << ", TFID: " << trafficLightID << std::endl;

 	pValues->bTrafficIsRed = !bGreenTrafficLight;

 	//cout << "Distances: " << pValues->stoppingDistances.size() << ", Distance To Stop : " << pValues->distanceToStop << endl;
 }

 void PassiveDecisionMaker::UpdateCurrentLane(const double& search_distance)
 {
	 PlannerHNS::Lane* pMapLane = 0;
	PlannerHNS::Lane* pPathLane = 0;
	pPathLane = MappingHelpers::GetLaneFromPath(state, m_Path);
	if(!pPathLane)
	{
		std::cout << "Performance Alert: Can't Find Lane Information in Global Path, Searching the Map :( " << std::endl;
		pMapLane  = MappingHelpers::GetClosestLaneFromMap(state, m_Map, search_distance);
	}

	if(pPathLane)
		pLane = pPathLane;
	else if(pMapLane)
		pLane = pMapLane;
	else
		pLane = 0;
 }

 PlannerHNS::BehaviorState PassiveDecisionMaker::GenerateBehaviorState()
 {
	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_pCurrentBehaviorState->GetCalcParams();

	m_pCurrentBehaviorState = m_pCurrentBehaviorState->GetNextState();
	if(m_pCurrentBehaviorState==0)
		m_pCurrentBehaviorState = m_pInitState;

	PlannerHNS::BehaviorState currentBehavior;

	currentBehavior.state = m_pCurrentBehaviorState->m_Behavior;
	currentBehavior.followDistance = preCalcPrams->distanceToNext;

	if(preCalcPrams->bUpcomingRight)
		currentBehavior.indicator = PlannerHNS::INDICATOR_RIGHT;
	else if(preCalcPrams->bUpcomingLeft)
		currentBehavior.indicator = PlannerHNS::INDICATOR_LEFT;
	else
		currentBehavior.indicator = PlannerHNS::INDICATOR_NONE;

	currentBehavior.minVelocity		= 0;
	currentBehavior.stopDistance 	= preCalcPrams->distanceToStop();
	currentBehavior.followVelocity 	= preCalcPrams->velocityOfNext;
	currentBehavior.iTrajectory		= 0;

	return currentBehavior;
 }

 double PassiveDecisionMaker::UpdateVelocityDirectlyToTrajectory(const BehaviorState& beh, const double& vel, const double& dt)
 {
	if(m_Path.size() ==0 ) return 0;

	RelativeInfo info, total_info;
	PlanningHelpers::GetRelativeInfo(m_Path, state, total_info);
	PlanningHelpers::GetRelativeInfo(m_Path, state, info);
	double average_braking_distance = -pow(vel, 2)/(m_CarInfo.max_deceleration) + m_params.additionalBrakingDistance;
	double max_velocity = PlannerHNS::PlanningHelpers::GetVelocityAhead(m_Path, total_info, total_info.iBack, average_braking_distance);

	unsigned int point_index = 0;
	double critical_long_front_distance = m_CarInfo.length/2.0;

	double desiredVelocity = 0;

	if(beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE || beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE)
	{
		PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, beh.stopDistance - critical_long_front_distance, point_index);

		double e = -beh.stopDistance;
		desiredVelocity = m_pidStopping.getPID(e);
	}
	else if(beh.state == FOLLOW_STATE)
	{
		double follow_d = beh.followDistance;
		double min_follow_distance = m_params.minFollowingDistance;

		double targe_acceleration = -pow(vel, 2)/(2.0*(follow_d));
		double target_velocity = (targe_acceleration * dt);
		double e = beh.followDistance - min_follow_distance;
		desiredVelocity = m_pidStopping.getPID(e);

		if(desiredVelocity > max_velocity)
			desiredVelocity = max_velocity;
		else if(desiredVelocity <= m_params.minSpeed)
			desiredVelocity = 0;

			//std::cout << "Acc: V: " << desiredVelocity << ", Object V: " <<  target_velocity << ", Accel: " << targe_acceleration << std::endl;
	}
	else if(beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE )
	{
		double target_velocity = max_velocity;
	//	double e = target_velocity - vel;
	//	desiredVelocity = m_pidVelocity.getPID(e);

		desiredVelocity = target_velocity;

		//std::cout << "Target Velocity: " << vel << ", e: " << e << ", Max Vel: " << max_velocity << std::endl;

		if(desiredVelocity>max_velocity)
			desiredVelocity = max_velocity;
		else if(desiredVelocity <= m_params.minSpeed)
			desiredVelocity = 0;
	}

	return desiredVelocity;
 }

 double PassiveDecisionMaker::GetSteerAngle()
 {
   RelativeInfo info;
    PlanningHelpers::GetRelativeInfo(m_Path, state, info);
    unsigned int point_index = 0;
    PlannerHNS::WayPoint pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(m_Path, info, 2, point_index);

    double current_a = UtilityHNS::UtilityH::SplitPositiveAngle(state.pos.a);
    double target_a = atan2(pursuite_point.pos.y - state.pos.y, pursuite_point.pos.x - state.pos.x);
    double e =  UtilityHNS::UtilityH::SplitPositiveAngle(target_a - current_a);
    double before_lowpass = e;//m_pidSteer.getPID(e);
    //std::cout << "CurrA: " << current_a << ", targetA: " << target_a << ", e: " << e << std::endl;
    return before_lowpass;

 }

 PlannerHNS::BehaviorState PassiveDecisionMaker::DoOneStep(const double& dt, const PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const std::vector<TrafficLight>& trafficLight, PlannerHNS::VehicleState& u_control_status)
{
	 PlannerHNS::BehaviorState beh;
	 state = currPose;
	 m_Path = path;

	 PlanningHelpers::FixPathDensity(m_Path, m_params.pathDensity);
	 PlanningHelpers::CalcAngleAndCost(m_Path);
	 //PlanningHelpers::GenerateRecommendedSpeed(m_Path, 5, 0.5);
	//UpdateCurrentLane(m_MaxLaneSearchDistance);

	CalculateImportantParameterForDecisionMaking(currPose.v, trafficLight);

	beh = GenerateBehaviorState();

	beh.maxVelocity = UpdateVelocityDirectlyToTrajectory(beh, currPose.v, dt);

	PlannerHNS::VehicleState currStatus;
	currStatus.speed = state.v;
	currStatus.shift = PlannerHNS::SHIFT_POS_DD;
	u_control_status.speed = beh.maxVelocity;
	u_control_status.steer = GetSteerAngle();
	//std::cout << "Eval_i: " << tc.index << ", Curr_i: " <<  m_pCurrentBehaviorState->GetCalcParams()->iCurrSafeTrajectory << ", Prev_i: " << m_pCurrentBehaviorState->GetCalcParams()->iPrevSafeTrajectory << std::endl;

	return beh;
 }

} /* namespace PlannerHNS */
