
/// \file  BehaviorPrediction.cpp
/// \brief Predict detected vehicles's possible trajectories, these trajectories extracted from the vector map.
/// \author Hatem Darweesh
/// \date Jul 6, 2017



#include "BehaviorPrediction.h"
#include "MappingHelpers.h"
#include "MatrixOperations.h"


namespace PlannerHNS
{

BehaviorPrediction::BehaviorPrediction()
{
	m_MaxLaneDetectionDistance = 0.5;
	m_PredictionDistance = 20.0;
	m_bGenerateBranches = false;
	m_bUseFixedPrediction = true;
	m_bStepByStep = false;
	UtilityHNS::UtilityH::GetTickCount(m_GenerationTimer);
	UtilityHNS::UtilityH::GetTickCount(m_ResamplingTimer);
}

BehaviorPrediction::~BehaviorPrediction()
{
}

void BehaviorPrediction::FilterObservations(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<DetectedObject>& filtered_list)
{
	for(unsigned int i=0; i < obj_list.size(); i++)
	{
		if(obj_list.at(i).t == SIDEWALK || obj_list.at(i).center.v < 1.0)
			continue;

		bool bFound = false;
		int found_index = 0;
		for(unsigned int ip=0; ip < filtered_list.size(); ip++)
		{
			if(filtered_list.at(ip).id == obj_list.at(i).id)
			{
				found_index = ip;
				bFound = true;
				break;
			}
		}

		if(bFound)
			filtered_list.at(found_index) = obj_list.at(i);
		else
			filtered_list.push_back(obj_list.at(i));
	}

	for(int ip=0; ip < (int)filtered_list.size(); ip++)
	{
		//check for cleaning
		bool bRevFound = false;
		for(unsigned int ic=0; ic < obj_list.size(); ic++)
		{
			if(filtered_list.at(ip).id == obj_list.at(ic).id)
			{
				bRevFound = true;
				break;
			}
		}

		if(!bRevFound)
		{
			filtered_list.erase(filtered_list.begin()+ip);
			ip--;
		}
	}
}

void BehaviorPrediction::DoOneStep(const std::vector<DetectedObject>& obj_list, const WayPoint& currPose, const double& minSpeed, const double& maxDeceleration, RoadNetwork& map)
{
	if(!m_bUseFixedPrediction && maxDeceleration !=0)
		m_PredictionDistance = -pow(currPose.v, 2)/(maxDeceleration);

	ExtractTrajectoriesFromMapII(obj_list, map, m_ParticleInfo_II);
	CalculateCollisionTimes(minSpeed);
	PredictionStepII(m_ParticleInfo_II);
	CorrectionStepII(m_ParticleInfo_II);

	//ExtractTrajectoriesFromMap(obj_list, map, m_ParticleInfo);
	//CalculateCollisionTimes(minSpeed);
	//PredictionStep(m_ParticleInfo);
	//CorrectionStep(m_ParticleInfo);
}

void BehaviorPrediction::CalculateCollisionTimes(const double& minSpeed)
{
	for(unsigned int i=0; i < m_PredictedObjects.size(); i++)
	{
		for(unsigned int j=0; j < m_PredictedObjects.at(i).predTrajectories.size(); j++)
		{
			PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(m_PredictedObjects.at(i).predTrajectories.at(j), m_PredictedObjects.at(i).center, minSpeed, m_PredictionDistance);
			PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_PredictedObjects.at(i).predTrajectories.at(j));
		}
	}
}

void BehaviorPrediction::ExtractTrajectoriesFromMapII(const std::vector<DetectedObject>& curr_obj_list,RoadNetwork& map, std::vector<ObjParticles*>& old_obj_list)
{
	PlannerH planner;
	m_temp_list_ii.clear();

	std::vector<ObjParticles*> delete_me_list = old_obj_list;

	for(unsigned int i=0; i < curr_obj_list.size(); i++)
	{
		bool bMatch = false;
		for(unsigned int ip=0; ip < old_obj_list.size(); ip++)
		{
			if(old_obj_list.at(ip)->obj.id == curr_obj_list.at(i).id)
			{
				bool bFound = false;
				for(unsigned int k=0; k < m_temp_list_ii.size(); k++)
				{
					if(m_temp_list_ii.at(k) == old_obj_list.at(ip))
					{
						bFound = true;
						break;
					}
				}

				if(!bFound)
				{
					old_obj_list.at(ip)->obj = curr_obj_list.at(i);
					m_temp_list_ii.push_back(old_obj_list.at(ip));
				}

				DeleteFromList(delete_me_list, old_obj_list.at(ip));

				old_obj_list.erase(old_obj_list.begin()+ip);
				bMatch = true;
				break;
			}
		}

		if(!bMatch)
		{
			ObjParticles* pNewObj = new  ObjParticles();
			pNewObj->obj = curr_obj_list.at(i);
			m_temp_list_ii.push_back(pNewObj);
		}
	}

	DeleteTheRest(delete_me_list);
	old_obj_list.clear();
	old_obj_list = m_temp_list_ii;

	m_PredictedObjects.clear();
	for(unsigned int ip=0; ip < old_obj_list.size(); ip++)
	{
		PredictCurrentTrajectory(map, old_obj_list.at(ip));
		m_PredictedObjects.push_back(old_obj_list.at(ip)->obj);
		old_obj_list.at(ip)->MatchTrajectories();
	}

}

void BehaviorPrediction::CalPredictionTimeForObject(ObjParticles* pCarPart)
{
	if(pCarPart->obj.center.v > 0 )
		pCarPart->m_PredictionTime = (MIN_PREDICTION_DISTANCE + 1.5*pCarPart->obj.center.v) / pCarPart->obj.center.v;
	else
		pCarPart->m_PredictionTime = MIN_PREDICTION_DISTANCE;
}

void BehaviorPrediction::PredictCurrentTrajectory(RoadNetwork& map, ObjParticles* pCarPart)
{
	pCarPart->obj.predTrajectories.clear();
	PlannerH planner;

	if(pCarPart->obj.bDirection && pCarPart->obj.bVelocity)
	{
		pCarPart->obj.pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(pCarPart->obj.center, map, m_MaxLaneDetectionDistance, pCarPart->obj.bDirection);
		if(m_bGenerateBranches)
		{
			double res_ret = planner.PredictTrajectoriesUsingDP(pCarPart->obj.center, pCarPart->obj.pClosestWaypoints, m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, pCarPart->obj.bDirection);
//			if(res_ret <= 2)
//			{
//				pCarPart->obj.predTrajectories =prev_predTrajectories;
//			}
		}
		else
		{
			planner.PredictTrajectoriesUsingDP(pCarPart->obj.center, pCarPart->obj.pClosestWaypoints, m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, pCarPart->obj.bDirection);
		}
	}
	else
	{
		bool bLocalDirectionSearch = false;
		pCarPart->obj.pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(pCarPart->obj.center, map, m_MaxLaneDetectionDistance, pCarPart->obj.bDirection);
		if(pCarPart->obj.pClosestWaypoints.size()>0)
		{
			pCarPart->obj.center.pos.a = pCarPart->obj.pClosestWaypoints.at(0)->pos.a;
			bLocalDirectionSearch = true;
		}
		if(m_bGenerateBranches)
		{
			double res_ret = planner.PredictTrajectoriesUsingDP(pCarPart->obj.center, pCarPart->obj.pClosestWaypoints, m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, bLocalDirectionSearch);
//				if(res_ret <= 2)
//				{
//					pCarPart->obj.predTrajectories =prev_predTrajectories;
//				}
		}
		else
		{
			planner.PredictTrajectoriesUsingDP(pCarPart->obj.center, pCarPart->obj.pClosestWaypoints, m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, pCarPart->obj.bDirection);
		}
	}
}

void BehaviorPrediction::ExtractTrajectoriesFromMap(const std::vector<DetectedObject>& obj_list,RoadNetwork& map, std::vector<ObjParticles>& old_list)
{
	PlannerH planner;
	m_temp_list.clear();
	m_PredictedObjects.clear();
//	for(unsigned int i=0; i <m_d_makers.size(); i++)
//		delete m_d_makers.at(i);
//	m_d_makers.clear();

	for(unsigned int i=0; i < obj_list.size(); i++)
	{
		bool bFound = false;
		int found_index = 0;
		for(unsigned int ip=0; ip < old_list.size(); ip++)
		{
			if(old_list.at(ip).obj.id == obj_list.at(i).id)
			{
				found_index = ip;
				bFound = true;
				break;
			}
		}

		if(!bFound)
		{
			old_list.push_back(ObjParticles());
			found_index = old_list.size()-1;
		}

		//unsigned int prev_trajectories_num = old_list.at(found_index).obj.predTrajectories.size();

		std::vector<std::vector<WayPoint> > prev_predTrajectories = old_list.at(found_index).obj.predTrajectories;
		old_list.at(found_index).obj = obj_list.at(i);
		old_list.at(found_index).obj.predTrajectories.clear();

		if(old_list.at(found_index).obj.bDirection && old_list.at(found_index).obj.bVelocity)
		{
			old_list.at(found_index).obj.pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(old_list.at(found_index).obj.center, map, m_MaxLaneDetectionDistance, old_list.at(found_index).obj.bDirection);
			if(m_bGenerateBranches)
			{
				double res_ret = planner.PredictTrajectoriesUsingDP(old_list.at(found_index).obj.center, old_list.at(found_index).obj.pClosestWaypoints, m_PredictionDistance, old_list.at(found_index).obj.predTrajectories, m_bGenerateBranches, old_list.at(found_index).obj.bDirection);
//				if(res_ret <= 2)
//				{
//					old_list.at(found_index).obj.predTrajectories =prev_predTrajectories;
//				}
			}
			else
			{
				planner.PredictTrajectoriesUsingDP(old_list.at(found_index).obj.center, old_list.at(found_index).obj.pClosestWaypoints, m_PredictionDistance, old_list.at(found_index).obj.predTrajectories, m_bGenerateBranches, old_list.at(found_index).obj.bDirection);
			}
		}
		else
		{
			bool bLocalDirectionSearch = false;
			old_list.at(found_index).obj.pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(old_list.at(found_index).obj.center, map, m_MaxLaneDetectionDistance, old_list.at(found_index).obj.bDirection);
			if(old_list.at(found_index).obj.pClosestWaypoints.size()>0)
			{
				old_list.at(found_index).obj.center.pos.a = old_list.at(found_index).obj.pClosestWaypoints.at(0)->pos.a;
				bLocalDirectionSearch = true;
			}
			if(m_bGenerateBranches)
			{
				double res_ret = planner.PredictTrajectoriesUsingDP(old_list.at(found_index).obj.center, old_list.at(found_index).obj.pClosestWaypoints, m_PredictionDistance, old_list.at(found_index).obj.predTrajectories, m_bGenerateBranches, bLocalDirectionSearch);
//				if(res_ret <= 2)
//				{
//					old_list.at(found_index).obj.predTrajectories =prev_predTrajectories;
//				}
			}
			else
			{
				planner.PredictTrajectoriesUsingDP(old_list.at(found_index).obj.center, old_list.at(found_index).obj.pClosestWaypoints, m_PredictionDistance, old_list.at(found_index).obj.predTrajectories, m_bGenerateBranches, old_list.at(found_index).obj.bDirection);
			}
		}

		//std::cout << "--------------------------" << std::endl;
		old_list.at(found_index).MatchTrajectories();
		//std::cout << "--------------------------" << std::endl;
		//std::cout << obj_list.at(i).id << "=> nTraj: " << old_list.at(found_index).obj.predTrajectories.size() << ", nInf: " << old_list.at(found_index).m_TrajectoryTracker.size() << std::endl;
		//for(unsigned int itt =0; itt < old_list.at(found_index).m_TrajectoryTracker.size(); itt++ )
		{
			//std::cout << "prev: " << old_list.at(found_index).m_TrajectoryTracker.at(itt).prev_index << ", curr: " << old_list.at(found_index).m_TrajectoryTracker.at(itt).index << ", beh: " <<  old_list.at(found_index).m_TrajectoryTracker.at(itt).beh << " [ ";
//			for(unsigned int it_ids = 0 ; it_ids < old_list.at(found_index).m_TrajectoryTracker.at(itt).ids.size(); it_ids++)
//					std::cout << old_list.at(found_index).m_TrajectoryTracker.at(itt).ids.at(it_ids) << "," ;
			//std::cout << " ] " << std::endl;;
		}
		//std::cout << "--------------------------" << std::endl << std::endl;

		m_temp_list.push_back(old_list.at(found_index));
		m_PredictedObjects.push_back(old_list.at(found_index).obj);

//		ControllerParams ctrl_params;
//		PlanningParams params;
//		CAR_BASIC_INFO car_info;
//		BehaviorState decision_beh;
//		PassiveDecisionMaker* dmaker = new PassiveDecisionMaker;
//		dmaker->Init(ctrl_params, params, car_info, decision_beh);
//		PassiveDecisionMaker* dmaker2 = new PassiveDecisionMaker(*dmaker);
//
//		m_d_makers.push_back(dmaker2);
//
//		delete dmaker;

	}

	old_list = m_temp_list;
}

void BehaviorPrediction::CorrectionStep(std::vector<ObjParticles>& part_info)
{
	for(unsigned int i=0; i < part_info.size(); i++)
	{
		CalculateWeights(part_info.at(i));
		ReSamplesParticles(part_info.at(i));
	}
}

void BehaviorPrediction::CorrectionStepII(std::vector<ObjParticles*>& part_info)
{
	for(unsigned int i=0; i < part_info.size(); i++)
	{
		CalculateWeightsII(part_info.at(i));
		ReSamplesParticlesII(part_info.at(i));
	}
}

void BehaviorPrediction::PredictionStep(std::vector<ObjParticles>& part_info)
{
	for(unsigned int i=0; i < part_info.size(); i++)
		SamplesParticles(part_info.at(i));
}

void BehaviorPrediction::PredictionStepII(std::vector<ObjParticles*>& part_info)
{
	for(unsigned int i=0; i < part_info.size(); i++)
		SamplesParticlesII(part_info.at(i));
}

void BehaviorPrediction::GenerateParticles(ObjParticles& parts)
{

	Particle p;
	p.pose = parts.obj.center;
	p.vel = 0;
	p.acc = 0;
	p.indicator = 0;

	for(unsigned int t=0; t < parts.m_TrajectoryTracker.size(); t++)
	{
		if(parts.m_TrajectoryTracker.at(t)->nAliveStop == 0)
		{
			p.beh = PlannerHNS::BEH_STOPPING_STATE;
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);
		}

		if(parts.m_TrajectoryTracker.at(t)->nAliveYield == 0)
		{
			p.beh = PlannerHNS::BEH_YIELDING_STATE;
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);
		}

		if(parts.m_TrajectoryTracker.at(t)->nAliveForward == 0 && parts.m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_FORWARD_STATE)
		{
			p.beh = PlannerHNS::BEH_FORWARD_STATE;
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);
		}

		if(parts.m_TrajectoryTracker.at(t)->nAliveLeft == 0 && parts.m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
		{
			p.beh = PlannerHNS::BEH_BRANCH_LEFT_STATE;
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);
		}

		if(parts.m_TrajectoryTracker.at(t)->nAliveRight == 0 && parts.m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
		{
			p.beh = PlannerHNS::BEH_BRANCH_RIGHT_STATE;
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);
		}
	}
}

void BehaviorPrediction::SamplesParticles(ObjParticles& parts)
{
	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	srand(t.tv_nsec);

	ENG eng(t.tv_nsec);
	NormalDIST dist_x(0, MOTION_POSE_ERROR);
	VariatGEN gen_x(eng, dist_x);
	NormalDIST vel(MOTION_VEL_ERROR, MOTION_VEL_ERROR);
	VariatGEN gen_v(eng, vel);
	NormalDIST ang(0, MOTION_ANGLE_ERROR);
	VariatGEN gen_a(eng, ang);
//	NormalDIST acl(0, MEASURE_ACL_ERROR);
//	VariatGEN gen_acl(eng, acl);

	Particle p;
	p.pose = parts.obj.center;
	p.vel = 0;
	p.acc = 0;
	p.indicator = 0;
	bool bRegenerate = true;

	if(UtilityHNS::UtilityH::GetTimeDiffNow(m_GenerationTimer) > 2)
	{
		UtilityHNS::UtilityH::GetTickCount(m_GenerationTimer);
		bRegenerate = true;
	}

	double number_of_particles = 0;

	for(unsigned int t=0; t < parts.m_TrajectoryTracker.size(); t++)
	{
		RelativeInfo info;

		if(parts.m_TrajectoryTracker.at(t)->nAliveForward == 0 && parts.m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_FORWARD_STATE)
		{
			//PlanningHelpers::GetRelativeInfo(parts.obj.predTrajectories.at(parts.m_TrajectoryTracker.at(t)->index), parts.obj.center, info);
			PlanningHelpers::GetRelativeInfo(*parts.m_TrajectoryTracker.at(t)->trajectory, parts.obj.center, info);
			unsigned int point_index = 0;
			//p.pose = PlanningHelpers::GetFollowPointOnTrajectory(parts.obj.predTrajectories.at(parts.m_TrajectoryTracker.at(t)->index), info, PREDICTION_DISTANCE, point_index);
			p.pose = PlanningHelpers::GetFollowPointOnTrajectory(*parts.m_TrajectoryTracker.at(t)->trajectory, info, PREDICTION_DISTANCE, point_index);

			p.beh = PlannerHNS::BEH_FORWARD_STATE;

			parts.m_TrajectoryTracker.at(t)->m_ForwardPart.clear();

			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			{
				parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.x  = parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.x + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.y  = parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.y + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.a  = parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.a + gen_a();
				parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).vel = parts.obj.center.v + fabs(gen_v());
				parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).indicator = 0;
				number_of_particles++;
			}

			//std::cout << "Forward Particles for Trajectory: " <<t << ", Is: " <<  parts.m_TrajectoryTracker.at(t)->m_ForwardPart.size()<< std::endl;
		}

		if((parts.m_TrajectoryTracker.at(t)->nAliveLeft == 0) && parts.m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
		{
			//PlanningHelpers::GetRelativeInfo(parts.obj.predTrajectories.at(parts.m_TrajectoryTracker.at(t)->index), parts.obj.center, info);
			PlanningHelpers::GetRelativeInfo(*parts.m_TrajectoryTracker.at(t)->trajectory, parts.obj.center, info);
			unsigned int point_index = 0;
			//p.pose = PlanningHelpers::GetFollowPointOnTrajectory(parts.obj.predTrajectories.at(parts.m_TrajectoryTracker.at(t)->index), info, PREDICTION_DISTANCE, point_index);
			p.pose = PlanningHelpers::GetFollowPointOnTrajectory(*parts.m_TrajectoryTracker.at(t)->trajectory, info, PREDICTION_DISTANCE, point_index);

			p.beh = PlannerHNS::BEH_BRANCH_LEFT_STATE;

			parts.m_TrajectoryTracker.at(t)->m_LeftPart.clear();
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
			{
				parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.x  = parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.x + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.y  = parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.y + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.a  = parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.a + gen_a();
				parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i).vel = parts.obj.center.v + gen_v()/2.0;
				parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i).indicator = 1;
				number_of_particles++;
			}

			//std::cout << "Forward Particles for Trajectory: " << t << ", Is: " <<  parts.m_TrajectoryTracker.at(t)->m_ForwardPart.size()<< std::endl;
		}

		if((parts.m_TrajectoryTracker.at(t)->nAliveRight == 0 ) && parts.m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
		{
			//std::cout << "size of info: " << parts.obj.predTrajectories.size() << std::endl;
			//PlanningHelpers::GetRelativeInfo(parts.obj.predTrajectories.at(parts.m_TrajectoryTracker.at(t)->index), parts.obj.center, info);
			PlanningHelpers::GetRelativeInfo(*parts.m_TrajectoryTracker.at(t)->trajectory, parts.obj.center, info);
			unsigned int point_index = 0;
			//p.pose = PlanningHelpers::GetFollowPointOnTrajectory(parts.obj.predTrajectories.at(parts.m_TrajectoryTracker.at(t)->index), info, PREDICTION_DISTANCE, point_index);
			p.pose = PlanningHelpers::GetFollowPointOnTrajectory(*parts.m_TrajectoryTracker.at(t)->trajectory, info, PREDICTION_DISTANCE, point_index);

			p.beh = PlannerHNS::BEH_BRANCH_RIGHT_STATE;

			parts.m_TrajectoryTracker.at(t)->m_RightPart.clear();
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
			{
				parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.x  = parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.x + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.y  = parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.y + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.a  = parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.a + gen_a();
				parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i).vel = parts.obj.center.v + gen_v()/2.0;
				parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i).indicator = -1;
				number_of_particles++;
			}
		}

		if(parts.m_TrajectoryTracker.at(t)->nAliveStop == 0 )
		{
			//std::cout << "size of info: " << parts.obj.predTrajectories.size() << std::endl;
			p.beh = PlannerHNS::BEH_STOPPING_STATE;

			parts.m_TrajectoryTracker.at(t)->m_StopPart.clear();
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
			{
				parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.x = parts.obj.center.pos.x + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.y = parts.obj.center.pos.y + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.a = parts.obj.center.pos.a + gen_a();
				parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.z = parts.obj.center.pos.z;
				parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i).vel = 0;
				parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i).indicator = 0;
				number_of_particles++;
			}
		}


		if(parts.m_TrajectoryTracker.at(t)->nAliveYield == 0 )
		{
			p.beh = PlannerHNS::BEH_YIELDING_STATE;

			parts.m_TrajectoryTracker.at(t)->m_YieldPart.clear();
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			double dx = info.perp_point.pos.x + (PREDICTION_DISTANCE * 0.5 * cos(info.perp_point.pos.a));
			double dy = info.perp_point.pos.y + (PREDICTION_DISTANCE * 0.5 * sin(info.perp_point.pos.a));
			double da = info.perp_point.pos.a;

			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
			{
				parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.x = dx + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.y = dy + gen_x();
				parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.a = da + gen_a();
				parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.z = parts.obj.center.pos.z;
				parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i).vel = parts.obj.center.v - fabs(gen_v());
				parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i).indicator = 0;
				number_of_particles++;
			}
		}
	}

	//std::cout <<"Number of Samples Particles = " << number_of_particles << std::endl;

}

void BehaviorPrediction::SamplesParticlesII(ObjParticles* parts)
{
	timespec _time;
	UtilityHNS::UtilityH::GetTickCount(_time);
	srand(_time.tv_nsec);

	ENG eng(_time.tv_nsec);
	NormalDIST dist_x(0, MOTION_POSE_ERROR);
	VariatGEN gen_x(eng, dist_x);
	NormalDIST vel(MOTION_VEL_ERROR, MOTION_VEL_ERROR);
	VariatGEN gen_v(eng, vel);
	NormalDIST ang(0, MOTION_ANGLE_ERROR);
	VariatGEN gen_a(eng, ang);
//	NormalDIST acl(0, MEASURE_ACL_ERROR);
//	VariatGEN gen_acl(eng, acl);

	Particle p;
	p.pose = parts->obj.center;
	p.vel = 0;
	p.acc = 0;
	p.indicator = 0;
	bool bRegenerate = true;

	if(UtilityHNS::UtilityH::GetTimeDiffNow(m_GenerationTimer) > 2)
	{
		UtilityHNS::UtilityH::GetTickCount(m_GenerationTimer);
		bRegenerate = true;
	}

	double number_of_particles = 0;

	for(unsigned int t=0; t < parts->m_TrajectoryTracker.size(); t++)
	{
		RelativeInfo info;

		if(parts->m_TrajectoryTracker.at(t)->nAliveForward == 0 && parts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_FORWARD_STATE)
		{
			PlanningHelpers::GetRelativeInfo(*(parts->m_TrajectoryTracker.at(t)->trajectory), parts->obj.center, info);
			unsigned int point_index = 0;
			p.pose = PlanningHelpers::GetFollowPointOnTrajectory(*(parts->m_TrajectoryTracker.at(t)->trajectory), info, PREDICTION_DISTANCE, point_index);
			p.pose.v = p.pose.v /3.6;

			p.beh = PlannerHNS::BEH_FORWARD_STATE;

			parts->m_TrajectoryTracker.at(t)->m_ForwardPart.clear();

			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			{
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.x  = parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.x + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.y  = parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.y + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.a  = parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.a + gen_a();
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).vel = parts->obj.center.v + fabs(gen_v());
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).indicator = 0;
				number_of_particles++;
			}

			//std::cout << "Forward Particles for Trajectory: " <<t << ", Is: " <<  parts->m_TrajectoryTracker.at(t)->m_ForwardPart.size()<< std::endl;
		}

		continue;

		if(parts->m_TrajectoryTracker.at(t)->nAliveLeft == 0 && parts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
		{
			PlanningHelpers::GetRelativeInfo(*(parts->m_TrajectoryTracker.at(t)->trajectory), parts->obj.center, info);

			unsigned int point_index = 0;
			p.pose = PlanningHelpers::GetFollowPointOnTrajectory(*(parts->m_TrajectoryTracker.at(t)->trajectory), info, PREDICTION_DISTANCE, point_index);

			p.beh = PlannerHNS::BEH_BRANCH_LEFT_STATE;

			parts->m_TrajectoryTracker.at(t)->m_LeftPart.clear();
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
			{
				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.x  = parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.x + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.y  = parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.y + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.a  = parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.a + gen_a();
				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).vel = parts->obj.center.v + gen_v()/2.0;
				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).indicator = 1;
				number_of_particles++;
			}

			//std::cout << "Forward Particles for Trajectory: " << t << ", Is: " <<  parts->m_TrajectoryTracker.at(t)->m_ForwardPart.size()<< std::endl;
		}

		if(parts->m_TrajectoryTracker.at(t)->nAliveRight == 0 && parts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
		{
			//std::cout << "size of info: " << parts->obj.predTrajectories.size() << std::endl;
			PlanningHelpers::GetRelativeInfo(*(parts->m_TrajectoryTracker.at(t)->trajectory), parts->obj.center, info);
			unsigned int point_index = 0;
			p.pose = PlanningHelpers::GetFollowPointOnTrajectory(*(parts->m_TrajectoryTracker.at(t)->trajectory), info, PREDICTION_DISTANCE, point_index);

			p.beh = PlannerHNS::BEH_BRANCH_RIGHT_STATE;

			parts->m_TrajectoryTracker.at(t)->m_RightPart.clear();
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
			{
				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.x  = parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.x + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.y  = parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.y + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.a  = parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.a + gen_a();
				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).vel = parts->obj.center.v + gen_v()/2.0;
				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).indicator = -1;
				number_of_particles++;
			}
		}

		if(parts->m_TrajectoryTracker.at(t)->nAliveStop == 0 )
		{
			//std::cout << "size of info: " << parts->obj.predTrajectories.size() << std::endl;
			p.beh = PlannerHNS::BEH_STOPPING_STATE;

			parts->m_TrajectoryTracker.at(t)->m_StopPart.clear();
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
			{
				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.x = parts->obj.center.pos.x + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.y = parts->obj.center.pos.y + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.a = parts->obj.center.pos.a + gen_a();
				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.z = parts->obj.center.pos.z;
				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).vel = 0;
				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).indicator = 0;
				number_of_particles++;
			}
		}


		if(parts->m_TrajectoryTracker.at(t)->nAliveYield == 0 )
		{

			p.beh = PlannerHNS::BEH_YIELDING_STATE;
			PlanningHelpers::GetRelativeInfo(*(parts->m_TrajectoryTracker.at(t)->trajectory), parts->obj.center, info);
			unsigned int point_index = 0;
			p.pose = PlanningHelpers::GetFollowPointOnTrajectory(*(parts->m_TrajectoryTracker.at(t)->trajectory), info, PREDICTION_DISTANCE/2.0, point_index);

			parts->m_TrajectoryTracker.at(t)->m_YieldPart.clear();
			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);

//			double dx = info.perp_point.pos.x + (PREDICTION_DISTANCE * 0.5 * cos(info.perp_point.pos.a));
//			double dy = info.perp_point.pos.y + (PREDICTION_DISTANCE * 0.5 * sin(info.perp_point.pos.a));
//			double da = info.perp_point.pos.a;

			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
			{
				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.x = parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.x + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.y = parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.y + gen_x();
				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.a = parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.a + gen_a();
				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.z = parts->obj.center.pos.z;
				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).vel = parts->obj.center.v - fabs(gen_v());
				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).indicator = 0;
				number_of_particles++;
			}
		}
	}

	//std::cout <<"Number of Samples Particles = " << number_of_particles << std::endl;

}

void BehaviorPrediction::CalOnePartWeight(ObjParticles& parts,Particle& p)
{
	if(p.bDeleted) return;

	p.pose_w = exp(-(0.5*pow(p.pose.pos.x - parts.obj.center.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+ pow(p.pose.pos.y - parts.obj.center.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)));
	p.dir_w  = exp(-(pow(p.pose.pos.a - parts.obj.center.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR)));
	p.vel_w  = exp(-(pow(p.vel - parts.obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR)));

	parts.pose_w_t += p.pose_w;
	parts.dir_w_t += p.dir_w;
	parts.vel_w_t += p.vel_w;

	if(p.pose_w > parts.pose_w_max)
		parts.pose_w_max = p.pose_w;
	if(p.dir_w > parts.dir_w_max)
		parts.dir_w_max = p.dir_w;
	if(p.vel_w > parts.vel_w_max)
		parts.vel_w_max = p.vel_w;
	if(p.pose_w < parts.pose_w_min)
		parts.pose_w_min = p.pose_w;
	if(p.dir_w < parts.dir_w_min)
		parts.dir_w_min = p.dir_w;
	if(p.vel_w < parts.vel_w_min)
		parts.vel_w_min = p.vel_w;
}

void BehaviorPrediction::CalOnePartWeightII(ObjParticles* pParts,Particle& p)
{
	if(p.bDeleted) return;

	p.pose_w = exp(-(0.5*pow(p.pose.pos.x - pParts->obj.center.pos.x,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+ pow(p.pose.pos.y - pParts->obj.center.pos.y,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)));
	p.dir_w  = exp(-(pow(p.pose.pos.a - pParts->obj.center.pos.a,2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR)));
	p.vel_w  = exp(-(pow(p.vel - pParts->obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR)));

	pParts->pose_w_t += p.pose_w;
	pParts->dir_w_t += p.dir_w;
	pParts->vel_w_t += p.vel_w;

	if(p.pose_w > pParts->pose_w_max)
		pParts->pose_w_max = p.pose_w;
	if(p.dir_w > pParts->dir_w_max)
		pParts->dir_w_max = p.dir_w;
	if(p.vel_w > pParts->vel_w_max)
		pParts->vel_w_max = p.vel_w;
	if(p.pose_w < pParts->pose_w_min)
		pParts->pose_w_min = p.pose_w;
	if(p.dir_w < pParts->dir_w_min)
		pParts->dir_w_min = p.dir_w;
	if(p.vel_w < pParts->vel_w_min)
		pParts->vel_w_min = p.vel_w;
}

void BehaviorPrediction::NormalizeOnePartWeightII(ObjParticles* pParts,Particle& p)
{
	if(p.bDeleted) return;

	double pose_diff  = pParts->pose_w_max-pParts->pose_w_min;
	double dir_diff = pParts->dir_w_max-pParts->dir_w_min;
	double vel_diff = pParts->vel_w_max-pParts->vel_w_min;
	double prev_val = p.dir_w;
	if(pose_diff != 0)
		p.pose_w = pose_diff/p.pose_w;
	else
		p.pose_w = 0;

	if(dir_diff != 0)
		p.dir_w = dir_diff/p.dir_w;
	else
		p.dir_w = 0;

	if(vel_diff != 0 )
		p.vel_w = vel_diff/p.vel_w;
	else
		p.vel_w = 0;

	p.w = p.pose_w*POSE_FACTOR + p.dir_w*DIRECTION_FACTOR + p.vel_w*VELOCITY_FACTOR;

	if(p.w >= pParts->max_w)
		pParts->max_w = p.w;

	if(p.w <= pParts->min_w)
		pParts->min_w = p.w;

	if(p.w > 1.1)
	  pParts->all_w += p.w;
}

void BehaviorPrediction::NormalizeOnePartWeight(ObjParticles& parts,Particle& p)
{
	if(p.bDeleted) return;

	double pose_diff  = parts.pose_w_max-parts.pose_w_min;
	double dir_diff = parts.dir_w_max-parts.dir_w_min;
	double vel_diff = parts.vel_w_max-parts.vel_w_min;
	if(pose_diff != 0)
		p.pose_w = p.pose_w/pose_diff;
	else
		p.pose_w = 0;

	if(dir_diff != 0)
		p.dir_w = p.dir_w/dir_diff;
	else
		p.dir_w = 0;

	if(vel_diff != 0 )
		p.vel_w = p.vel_w/vel_diff;
	else
		p.vel_w = 0;

	p.w = p.pose_w*POSE_FACTOR + p.dir_w*DIRECTION_FACTOR + p.vel_w*VELOCITY_FACTOR;

	if(p.w >= parts.max_w)
		parts.max_w = p.w;

	if(p.w <= parts.min_w)
		parts.min_w = p.w;

	parts.all_w += p.w;
}

void BehaviorPrediction::CalculateWeights(ObjParticles& parts)
{
	parts.all_w = 0;
	parts.pose_w_t = 0;
	parts.dir_w_t = 0;
	parts.vel_w_t = 0;

	parts.pose_w_max = -99999999;
	parts.dir_w_max = -99999999;
	parts.vel_w_max = -99999999;

	parts.pose_w_min = 99999999;
	parts.dir_w_min = 99999999;
	parts.vel_w_min = 99999999;

	for(unsigned int t=0; t < parts.m_TrajectoryTracker.size(); t++)
	{
		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			CalOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i));

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
			CalOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i));

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
			CalOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i));

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
			CalOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i));

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
			CalOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i));
	}

	//Normalize
	parts.max_w = -9999999;
	parts.min_w = 9999999;
	parts.all_w = 0;

	for(unsigned int t=0; t < parts.m_TrajectoryTracker.size(); t++)
	{
		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			NormalizeOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i));

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
			NormalizeOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i));

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
			NormalizeOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i));

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
			NormalizeOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i));

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
			NormalizeOnePartWeight(parts, parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i));
	}

	double half_percent_value = parts.min_w + (parts.max_w - parts.min_w)*KEEP_PERCENTAGE;

//	std::cout << "Behavior Prob ------------------------------------------------ " << std::endl;
//	std::cout << "Max: " <<  parts.max_w << ", Min: " << parts.min_w << ", HalfVal: "<< half_percent_value <<  std::endl;
//	std::cout << "------------------------------------------------ --------------" << std::endl;

//	parts.n_left_branch = 0;
//	parts.n_stop = 0;
//	parts.n_yield = 0;
//	parts.n_right_branch = 0;

	for(unsigned int t=0; t < parts.m_TrajectoryTracker.size(); t++)
	{
		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
		{
			if(parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).w < half_percent_value)
			{
				parts.m_TrajectoryTracker.at(t)->DeleteParticle(parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i), i);
			}
		}

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
		{
			if(parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i).w < half_percent_value)
			{
				parts.m_TrajectoryTracker.at(t)->DeleteParticle(parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i), i);
			}
		}

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
		{
			if(parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i).w < half_percent_value)
			{
				parts.m_TrajectoryTracker.at(t)->DeleteParticle(parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i), i);
			}
		}

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
		{
			if(parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i).w < half_percent_value)
			{
				parts.m_TrajectoryTracker.at(t)->DeleteParticle(parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i), i);
			}
		}

		for(unsigned int i = 0; i < parts.m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
		{
			if(parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i).w < half_percent_value)
			{
				parts.m_TrajectoryTracker.at(t)->DeleteParticle(parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i), i);
			}
		}
	}

	parts.CalculateProbabilities();

	std::cout << "Behavior Prob ------------------------------------------------ " << std::endl;
	for(unsigned int t=0; t < parts.m_TrajectoryTracker.size(); t++)
	{
		std::cout << "Traj:" << t << ", Best Beh:" << parts.m_TrajectoryTracker.at(t)->best_beh << ", Best P: " << parts.m_TrajectoryTracker.at(t)->best_p << std::endl;
		std::cout << "     F: N:" << parts.m_TrajectoryTracker.at(t)->nAliveForward << ", P:" << parts.m_TrajectoryTracker.at(t)->pForward << std::endl;
		std::cout << "     L: N:" << parts.m_TrajectoryTracker.at(t)->nAliveLeft << ", P:" << parts.m_TrajectoryTracker.at(t)->pLeft << std::endl;
		std::cout << "     R: N:" << parts.m_TrajectoryTracker.at(t)->nAliveRight << ", P:" << parts.m_TrajectoryTracker.at(t)->pRight << std::endl;
		std::cout << "     S: N:" << parts.m_TrajectoryTracker.at(t)->nAliveStop << ", P:" << parts.m_TrajectoryTracker.at(t)->pStop << std::endl;
		std::cout << "     Y: N:" << parts.m_TrajectoryTracker.at(t)->nAliveYield << ", P:" << parts.m_TrajectoryTracker.at(t)->pYield << std::endl << std::endl;
	}
	std::cout << "------------------------------------------------ --------------" << std::endl;


//	std::string best_behavior = "Unknown";
//	if(type == 1001)
//	{
//		best_behavior = "Stopping";
//		parts.m_beh.state = STOPPING_STATE;
//		parts.m_beh.stopDistance = -1;
//	}
//	else if(type < 1000)
//	{
//		best_behavior = "Forward";
//		parts.m_beh.state = FORWARD_STATE;
//		parts.m_beh.stopDistance = type;
//	}
//	else if(type == 1002)
//	{
//		best_behavior = "Yielding";
//		parts.m_beh.state = YIELDING_STATE;
//		parts.m_beh.stopDistance = -1;
//	}
//	else if(type == 1003)
//	{
//		best_behavior = "Right";
//		parts.m_beh.state = BRANCH_RIGHT_STATE;
//		parts.m_beh.stopDistance = -1;
//	}
//	else if(type == 1004)
//	{
//		best_behavior = "Left";
//		parts.m_beh.state = BRANCH_LEFT_STATE;
//		parts.m_beh.stopDistance = -1;
//	}
//
//	for(unsigned int iPath=0; iPath < parts.pred_paths.size(); iPath++)
//	{
//		if(parts.pred_paths.at(iPath).size() == 0 ) continue;
//
//		for(unsigned int iRes = 0; iRes < prop_list.size(); iRes++)
//		{
//			if(prop_list.at(iRes).first == iPath)
//			{
//				if(prop_list.at(iRes).first == parts.m_beh.stopDistance)
//					parts.pred_paths.at(iPath).at(0).collisionCost = prop_list.at(iRes).second*3.0;
//				else
//					parts.pred_paths.at(iPath).at(0).collisionCost = prop_list.at(iRes).second;
//			}
//		}
//
//		if(parts.pred_paths.at(iPath).at(0).beh_state == BEH_BRANCH_RIGHT_STATE)
//		{
//			if(parts.m_beh.state == BRANCH_RIGHT_STATE)
//				parts.pred_paths.at(iPath).at(0).collisionCost = parts.p_right_branch*3.0;
//			else
//				parts.pred_paths.at(iPath).at(0).collisionCost = parts.p_right_branch;
//		}
//		else if(parts.pred_paths.at(iPath).at(0).beh_state == BEH_BRANCH_LEFT_STATE)
//		{
//			if(parts.m_beh.state == BRANCH_LEFT_STATE)
//				parts.pred_paths.at(iPath).at(0).collisionCost = parts.p_left_branch*3.0;
//			else
//				parts.pred_paths.at(iPath).at(0).collisionCost = parts.p_left_branch;
//		}
//	}
	//std::cout << "Behavior: " <<  best_behavior << "; Confidence: " << percentage << "; Index: " << type << ", Remaining Particles: " << parts.particles.size() << std::endl << std::endl;

}

void BehaviorPrediction::CalculateWeightsII(ObjParticles* pParts)
{
	pParts->all_w = 0;
	pParts->pose_w_t = 0;
	pParts->dir_w_t = 0;
	pParts->vel_w_t = 0;

	pParts->pose_w_max = -99999999;
	pParts->dir_w_max = -99999999;
	pParts->vel_w_max = -99999999;

	pParts->pose_w_min = 99999999;
	pParts->dir_w_min = 99999999;
	pParts->vel_w_min = 99999999;

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			CalOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i));

		continue;

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
			CalOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i));

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
			CalOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_RightPart.at(i));

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
			CalOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i));

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
			CalOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i));
	}

	//Normalize
	pParts->max_w = -9999999;
	pParts->min_w = 9999999;
	pParts->all_w = 0;

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			NormalizeOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i));

		continue;

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
			NormalizeOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i));

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
			NormalizeOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_RightPart.at(i));

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
			NormalizeOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i));

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
			NormalizeOnePartWeightII(pParts, pParts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i));
	}

	double half_percent_value = pParts->min_w + (pParts->max_w - pParts->min_w)*KEEP_PERCENTAGE;

	std::cout << "Behavior Weights ------------------------------------------------ " << std::endl;
	std::cout << " Keep Percent Value: " << half_percent_value <<  std::endl;


	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		double avg_sum = 0;
		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
		{
			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).w;
		}
		if(pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size() > 0)
			std::cout << "     F: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveForward << ", W:" << avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size() << std::endl;

		continue;

		avg_sum = 0;
		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
		{
			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).w;
		}

		if(pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size() > 0)
			std::cout << "     L: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveLeft << ", W:" << avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size() << std::endl;

		avg_sum = 0;
		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
		{
			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).w;
		}
		if(pParts->m_TrajectoryTracker.at(t)->m_RightPart.size() > 0)
			std::cout << "     R: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveRight << ", W:" << avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_RightPart.size() << std::endl;

		avg_sum = 0;
		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
		{
			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).w;
		}
		if(pParts->m_TrajectoryTracker.at(t)->m_StopPart.size() > 0)
			std::cout << "     S: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveStop << ", W:" << avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_StopPart.size() << std::endl;

		avg_sum = 0;
		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
		{
			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).w;
		}
		if(pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size() > 0)
			std::cout << "     Y: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveYield << ", W:" << avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size() << std::endl;
	}

	std::cout << "------------------------------------------------ --------------" << std::endl;

//	return;

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
		{
			if(pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).w < half_percent_value)
			{
				pParts->m_TrajectoryTracker.at(t)->DeleteParticle(pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i), i);
			}
		}

		continue;

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
		{
			if(pParts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).w < half_percent_value)
			{
				pParts->m_TrajectoryTracker.at(t)->DeleteParticle(pParts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i), i);
			}
		}

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
		{
			if(pParts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).w < half_percent_value)
			{
				pParts->m_TrajectoryTracker.at(t)->DeleteParticle(pParts->m_TrajectoryTracker.at(t)->m_RightPart.at(i), i);
			}
		}

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
		{
			if(pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).w < half_percent_value)
			{
				pParts->m_TrajectoryTracker.at(t)->DeleteParticle(pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i), i);
			}
		}

		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
		{
			if(pParts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).w < half_percent_value)
			{
				pParts->m_TrajectoryTracker.at(t)->DeleteParticle(pParts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i), i);
			}
		}
	}

	pParts->CalculateProbabilities();

//	std::cout << "Behavior Prob ------------------------------------------------ " << std::endl;
//	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
//	{
//		std::cout << "Traj:" << t << ", Best Beh:" << pParts->m_TrajectoryTracker.at(t)->best_beh << ", Best P: " << pParts->m_TrajectoryTracker.at(t)->best_p << std::endl;
//		std::cout << "     F: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveForward << ", P:" << pParts->m_TrajectoryTracker.at(t)->pForward << std::endl;
//		std::cout << "     L: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveLeft << ", P:" << pParts->m_TrajectoryTracker.at(t)->pLeft << std::endl;
//		std::cout << "     R: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveRight << ", P:" << pParts->m_TrajectoryTracker.at(t)->pRight << std::endl;
//		std::cout << "     S: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveStop << ", P:" << pParts->m_TrajectoryTracker.at(t)->pStop << std::endl;
//		std::cout << "     Y: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveYield << ", P:" << pParts->m_TrajectoryTracker.at(t)->pYield << std::endl << std::endl;
//	}
//	std::cout << "------------------------------------------------ --------------" << std::endl;
}

void BehaviorPrediction::ReSamplesParticles(ObjParticles& parts)
{
	double dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_ResamplingTimer);
	UtilityHNS::UtilityH::GetTickCount(m_ResamplingTimer);

	timespec t;
	UtilityHNS::UtilityH::GetTickCount(t);
	srand(t.tv_nsec);

	ENG eng(t.tv_nsec);
	NormalDIST dist_x(0, MOTION_POSE_ERROR);
	VariatGEN gen_x(eng, dist_x);
	NormalDIST vel(MOTION_VEL_ERROR, MOTION_VEL_ERROR);
	VariatGEN gen_v(eng, vel);
	NormalDIST ang(0, MOTION_ANGLE_ERROR);
	VariatGEN gen_a(eng, ang);
//	NormalDIST acl(0, MEASURE_ACL_ERROR);
//	VariatGEN gen_acl(eng, acl);

	double dx = 0;
	double dy = 0;
	std::vector<Particle> part_list;
	for(unsigned int t=0; t < parts.m_TrajectoryTracker.size(); t++)
	{

		if(parts.m_TrajectoryTracker.at(t)->nAliveForward > 0 && parts.m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_FORWARD_STATE)
		{
			int n_per_part = ((BEH_PARTICLES_NUM - parts.m_TrajectoryTracker.at(t)->nAliveForward)/parts.m_TrajectoryTracker.at(t)->nAliveForward);
			part_list.clear();
			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			{
				Particle* p = &parts.m_TrajectoryTracker.at(t)->m_ForwardPart.at(i);
				p->pose.pos.x += parts.obj.center.v * dt * cos(p->pose.pos.a);
				p->pose.pos.y += parts.obj.center.v * dt * sin(p->pose.pos.a);

				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel += fabs(gen_v());
					part_list.push_back(np);
				}
			}

			for(unsigned int ip=0; ip < part_list.size(); ip++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}


		if(parts.m_TrajectoryTracker.at(t)->nAliveLeft > 0 && parts.m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
		{
			int n_per_part = ((BEH_PARTICLES_NUM - parts.m_TrajectoryTracker.at(t)->nAliveLeft)/parts.m_TrajectoryTracker.at(t)->nAliveLeft);
			part_list.clear();
			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
			{
				Particle* p = &parts.m_TrajectoryTracker.at(t)->m_LeftPart.at(i);
				p->pose.pos.x += parts.obj.center.v * dt * cos(p->pose.pos.a);
				p->pose.pos.y += parts.obj.center.v * dt * sin(p->pose.pos.a);

				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel += gen_v()/2.0;
					part_list.push_back(np);
				}
			}
			for(unsigned int ip=0; ip < part_list.size(); ip++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}

		if(parts.m_TrajectoryTracker.at(t)->nAliveRight > 0 && parts.m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
		{
			int n_per_part = ((BEH_PARTICLES_NUM - parts.m_TrajectoryTracker.at(t)->nAliveRight)/parts.m_TrajectoryTracker.at(t)->nAliveRight)+1;
			part_list.clear();
			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
			{
				Particle* p = &parts.m_TrajectoryTracker.at(t)->m_RightPart.at(i);
				p->pose.pos.x += parts.obj.center.v * dt * cos(p->pose.pos.a);
				p->pose.pos.y += parts.obj.center.v * dt * sin(p->pose.pos.a);

				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel += gen_v()/2.0;
					part_list.push_back(np);
				}
			}

			for(unsigned int ip=0; ip < part_list.size(); ip++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}

		if(parts.m_TrajectoryTracker.at(t)->nAliveStop > 0 )
		{
			int n_per_part = ((BEH_PARTICLES_NUM - parts.m_TrajectoryTracker.at(t)->nAliveStop)/parts.m_TrajectoryTracker.at(t)->nAliveStop)+1;
			part_list.clear();
			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
			{
				Particle* p = &parts.m_TrajectoryTracker.at(t)->m_StopPart.at(i);
				p->pose.pos.x += parts.obj.center.v * dt * cos(p->pose.pos.a);
				p->pose.pos.y += parts.obj.center.v * dt * sin(p->pose.pos.a);

				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel = 0;
					part_list.push_back(np);
				}
			}

			for(unsigned int ip=0; ip < part_list.size(); ip++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}


		if(parts.m_TrajectoryTracker.at(t)->nAliveYield == 0 )
		{
			int n_per_part = ((BEH_PARTICLES_NUM - parts.m_TrajectoryTracker.at(t)->nAliveYield)/parts.m_TrajectoryTracker.at(t)->nAliveYield)+1;
			part_list.clear();
			for(unsigned int i=0; i < parts.m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
			{
				Particle* p = &parts.m_TrajectoryTracker.at(t)->m_YieldPart.at(i);
				p->pose.pos.x += parts.obj.center.v * dt * cos(p->pose.pos.a);
				p->pose.pos.y += parts.obj.center.v * dt * sin(p->pose.pos.a);

				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel = parts.obj.center.v - fabs(gen_v());
					part_list.push_back(np);
				}
			}

			for(unsigned int ip=0; ip < part_list.size(); ip++)
				parts.m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}
	}

//		std::cout << "Behavior Prob ------------------------------------------------ " << std::endl;
//		for(unsigned int t=0; t < parts.m_TrajectoryTracker.size(); t++)
//		{
//			std::cout << "Traj:" << t << ", Best Beh:" << parts.m_TrajectoryTracker.at(t)->best_beh << ", Best P: " << parts.m_TrajectoryTracker.at(t)->best_p << std::endl;
//			std::cout << "     F: N:" << parts.m_TrajectoryTracker.at(t)->m_ForwardPart.size() << ", P:" << parts.m_TrajectoryTracker.at(t)->pForward << std::endl;
//			std::cout << "     L: N:" << parts.m_TrajectoryTracker.at(t)->m_LeftPart.size() << ", P:" << parts.m_TrajectoryTracker.at(t)->pLeft << std::endl;
//			std::cout << "     R: N:" << parts.m_TrajectoryTracker.at(t)->m_RightPart.size() << ", P:" << parts.m_TrajectoryTracker.at(t)->pRight << std::endl;
//			std::cout << "     S: N:" << parts.m_TrajectoryTracker.at(t)->m_StopPart.size()<< ", P:" << parts.m_TrajectoryTracker.at(t)->pStop << std::endl;
//			std::cout << "     Y: N:" << parts.m_TrajectoryTracker.at(t)->m_YieldPart.size() << ", P:" << parts.m_TrajectoryTracker.at(t)->pYield << std::endl << std::endl;
//		}
//		std::cout << "------------------------------------------------ --------------" << std::endl;
}

void BehaviorPrediction::ReSamplesParticlesII(ObjParticles* pParts)
{

	double dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_ResamplingTimer);
	if(m_bStepByStep)
		dt = 0.02;

	UtilityHNS::UtilityH::GetTickCount(m_ResamplingTimer);

	timespec _time;
	UtilityHNS::UtilityH::GetTickCount(_time);
	srand(_time.tv_nsec);

	ENG eng(_time.tv_nsec);
	NormalDIST dist_x(0, MOTION_POSE_ERROR);
	VariatGEN gen_x(eng, dist_x);
	NormalDIST vel(MOTION_VEL_ERROR, MOTION_VEL_ERROR);
	VariatGEN gen_v(eng, vel);
	NormalDIST ang(0, MOTION_ANGLE_ERROR);
	VariatGEN gen_a(eng, ang);
//	NormalDIST acl(0, MEASURE_ACL_ERROR);
//	VariatGEN gen_acl(eng, acl);

	double dx = 0;
	double dy = 0;
	std::vector<Particle> part_list;
	std::vector<TrafficLight> trafficLight;
	PlannerHNS::BehaviorState curr_behavior;
	PlannerHNS::VehicleState control_u;
	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{

		if(pParts->m_TrajectoryTracker.at(t)->nAliveForward > 0 && pParts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_FORWARD_STATE)
		{
			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveForward)/pParts->m_TrajectoryTracker.at(t)->nAliveForward);
			part_list.clear();
			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			{
				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i);
				//std::cout<< "P_V: " << p->pose.v << ", P_Obj: " << pParts->obj.center.v << std::endl;
				//p->pose.v = pParts->obj.center.v;
				if(USE_OPEN_PLANNER_MOVE == 0)
				  {
				    p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
				    p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
				  }
				else
				  {
				    curr_behavior = pParts->m_TrajectoryTracker.at(t)->m_SinglePathDecisionMaker.DoOneStep(dt, p->pose, *(pParts->m_TrajectoryTracker.at(t)->trajectory), trafficLight,control_u);
				    p->pose.pos.x += control_u.speed * dt * cos(p->pose.pos.a);
				    p->pose.pos.y += control_u.speed * dt * sin(p->pose.pos.a);
				    p->pose.pos.a += control_u.speed * dt * tan(control_u.steer)  / pParts->obj.l*0.75;
				   // std::cout << "Move Forward: " << control_u.speed  <<" , Steer:" << control_u.steer << ", Beh: " << curr_behavior.state << ", " << pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size() << std::endl;
				  }

				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel += fabs(gen_v());
					part_list.push_back(np);
				}
			}

			for(unsigned int ip=0; ip < part_list.size(); ip++)
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}


		continue;

		if(pParts->m_TrajectoryTracker.at(t)->nAliveLeft > 0 && pParts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
		{
			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveLeft)/pParts->m_TrajectoryTracker.at(t)->nAliveLeft);
			part_list.clear();
			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
			{
				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i);
				if(USE_OPEN_PLANNER_MOVE == 0)
				{
				p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
				p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
				}
				else
				  {
				curr_behavior = pParts->m_TrajectoryTracker.at(t)->m_SinglePathDecisionMaker.DoOneStep(dt, p->pose, *(pParts->m_TrajectoryTracker.at(t)->trajectory), trafficLight,control_u);
				p->pose.pos.x += curr_behavior.maxVelocity * dt * cos(p->pose.pos.a);
				p->pose.pos.y += curr_behavior.maxVelocity * dt * sin(p->pose.pos.a);
				p->pose.pos.a += control_u.speed * dt * tan(control_u.steer)  / pParts->obj.l*0.75;
				//std::cout << "Move Left: " << t << ", " << pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size() << std::endl;
				  }


				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel += gen_v()/2.0;
					part_list.push_back(np);
				}
			}
			for(unsigned int ip=0; ip < part_list.size(); ip++)
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}

		if(pParts->m_TrajectoryTracker.at(t)->nAliveRight > 0 && pParts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
		{
			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveRight)/pParts->m_TrajectoryTracker.at(t)->nAliveRight)+1;
			part_list.clear();
			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
			{
				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_RightPart.at(i);
				if(USE_OPEN_PLANNER_MOVE == 0)
				{
				p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
				p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
				}
				else
				  {
				curr_behavior = pParts->m_TrajectoryTracker.at(t)->m_SinglePathDecisionMaker.DoOneStep(dt, p->pose, *(pParts->m_TrajectoryTracker.at(t)->trajectory), trafficLight,control_u);
				p->pose.pos.x += curr_behavior.maxVelocity * dt * cos(p->pose.pos.a);
				p->pose.pos.y += curr_behavior.maxVelocity * dt * sin(p->pose.pos.a);
				p->pose.pos.a += control_u.speed * dt * tan(control_u.steer)  / pParts->obj.l*0.75;
				//std::cout << "Move Right: " << t << ", " << pParts->m_TrajectoryTracker.at(t)->m_RightPart.size() << std::endl;
				  }


				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel += gen_v()/2.0;
					part_list.push_back(np);
				}
			}

			for(unsigned int ip=0; ip < part_list.size(); ip++)
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}

		if(pParts->m_TrajectoryTracker.at(t)->nAliveStop > 0 )
		{
			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveStop)/pParts->m_TrajectoryTracker.at(t)->nAliveStop)+1;
			part_list.clear();
			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
			{
				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i);
				if(USE_OPEN_PLANNER_MOVE == 0)
				{
				p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
				p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
				}
				else
				  {
				curr_behavior = pParts->m_TrajectoryTracker.at(t)->m_SinglePathDecisionMaker.DoOneStep(dt, p->pose, *(pParts->m_TrajectoryTracker.at(t)->trajectory), trafficLight,control_u);
				p->pose.pos.x += curr_behavior.maxVelocity * dt * cos(p->pose.pos.a);
				p->pose.pos.y += curr_behavior.maxVelocity * dt * sin(p->pose.pos.a);
				p->pose.pos.a += control_u.speed * dt * tan(control_u.steer)  / pParts->obj.l*0.75;
				//std::cout << "Move Stop: " << curr_behavior.maxVelocity << ", " << pParts->m_TrajectoryTracker.at(t)->m_StopPart.size() << std::endl;
				  }

				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel = 0;
					part_list.push_back(np);
				}
			}

			for(unsigned int ip=0; ip < part_list.size(); ip++)
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}


		if(pParts->m_TrajectoryTracker.at(t)->nAliveYield > 0 )
		{
			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveYield)/pParts->m_TrajectoryTracker.at(t)->nAliveYield)+1;
			part_list.clear();
			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
			{
				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i);
				if(USE_OPEN_PLANNER_MOVE == 0)
				{
				p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
				p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
				}
				else
				  {
				curr_behavior = pParts->m_TrajectoryTracker.at(t)->m_SinglePathDecisionMaker.DoOneStep(dt, p->pose, *(pParts->m_TrajectoryTracker.at(t)->trajectory), trafficLight,control_u);
				p->pose.pos.x += curr_behavior.maxVelocity * dt * cos(p->pose.pos.a);
				p->pose.pos.y += curr_behavior.maxVelocity * dt * sin(p->pose.pos.a);
				p->pose.pos.a += control_u.speed * dt * tan(control_u.steer)  / pParts->obj.l*0.75;
			//std::cout << "Move Yield: " << curr_behavior.maxVelocity << ", " << pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size() << std::endl;
				  }

				for(int ic=0; ic < n_per_part; ic++)
				{
					Particle np = *p;
					np.pose.pos.x +=  gen_x();
					np.pose.pos.y +=  gen_x();
					np.pose.pos.a +=  gen_a();
					np.vel = pParts->obj.center.v - fabs(gen_v());
					part_list.push_back(np);
				}
			}

			for(unsigned int ip=0; ip < part_list.size(); ip++)
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
		}
	}

//		std::cout << "Behavior Prob ------------------------------------------------ " << std::endl;
//		for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
//		{
//			std::cout << "Traj:" << t << ", Best Beh:" << pParts->m_TrajectoryTracker.at(t)->best_beh << ", Best P: " << pParts->m_TrajectoryTracker.at(t)->best_p << std::endl;
//			std::cout << "     F: N:" << pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size() << ", P:" << pParts->m_TrajectoryTracker.at(t)->pForward << std::endl;
//			std::cout << "     L: N:" << pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size() << ", P:" << pParts->m_TrajectoryTracker.at(t)->pLeft << std::endl;
//			std::cout << "     R: N:" << pParts->m_TrajectoryTracker.at(t)->m_RightPart.size() << ", P:" << pParts->m_TrajectoryTracker.at(t)->pRight << std::endl;
//			std::cout << "     S: N:" << pParts->m_TrajectoryTracker.at(t)->m_StopPart.size()<< ", P:" << pParts->m_TrajectoryTracker.at(t)->pStop << std::endl;
//			std::cout << "     Y: N:" << pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size() << ", P:" << pParts->m_TrajectoryTracker.at(t)->pYield << std::endl << std::endl;
//		}
//		std::cout << "------------------------------------------------ --------------" << std::endl;
}

} /* namespace PlannerHNS */
