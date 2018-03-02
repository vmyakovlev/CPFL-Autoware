
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
	m_bCanDecide = true;
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

	ExtractTrajectoriesFromMap(obj_list, map, m_ParticleInfo_II);
	CalculateCollisionTimes(minSpeed);
	//PredictionStepII(m_ParticleInfo_II);
	//CorrectionStepII(m_ParticleInfo_II);
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

void BehaviorPrediction::ExtractTrajectoriesFromMap(const std::vector<DetectedObject>& curr_obj_list,RoadNetwork& map, std::vector<ObjParticles*>& old_obj_list)
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

	std::cout << "------------------------------------------------------------" << std::endl;

	if(pCarPart->obj.bDirection && pCarPart->obj.bVelocity)
	{
		PlannerHNS::WayPoint fake_pose = pCarPart->obj.center;
//		fake_pose.pos.x -= 2.0 * cos(fake_pose.pos.a);
//		fake_pose.pos.y -= 2.0 * sin(fake_pose.pos.a);

		pCarPart->obj.pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(fake_pose, map, m_MaxLaneDetectionDistance, pCarPart->obj.bDirection);
		planner.PredictTrajectoriesUsingDP(fake_pose, pCarPart->obj.pClosestWaypoints, m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, pCarPart->obj.bDirection);
		for(unsigned int j=0; i < pCarPart->obj.pClosestWaypoints.size(); j++)
		{
			std::cout << "(" << j << " , " << pCarPart->obj.pClosestWaypoints.at(j)->pos.a << ")";
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
		planner.PredictTrajectoriesUsingDP(pCarPart->obj.center, pCarPart->obj.pClosestWaypoints, m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, pCarPart->obj.bDirection);
	}


	for(unsigned int t = 0; t < pCarPart->obj.predTrajectories.size(); t ++)
	{
		if(pCarPart->obj.predTrajectories.at(t).size() > 0)
		{
			pCarPart->obj.predTrajectories.at(t).at(0).collisionCost = 1;

			std::cout << "CarA: " << pCarPart->obj.center.pos.a << ", ";

		}
	}

	std::cout << "------------------------------------------------------------" << std::endl;
}

void BehaviorPrediction::CorrectionStepII(std::vector<ObjParticles*>& part_info)
{
	for(unsigned int i=0; i < part_info.size(); i++)
	{
		CollectParticles(part_info.at(i));
		MoveParticles(part_info.at(i));
		CalculateWeightsII(part_info.at(i));
		RemoveWeakParticles(part_info.at(i));
		CalculateProbabilities(part_info.at(i));
		//ReSamplesParticlesII(part_info.at(i));
	}
}

int BehaviorPrediction::FromIndicatorToNumber(const PlannerHNS::LIGHT_INDICATOR& indi)
{
	if(indi == PlannerHNS::INDICATOR_NONE)
		return 0;
	else if(indi == PlannerHNS::INDICATOR_LEFT)
		return 1;
	else if(indi == PlannerHNS::INDICATOR_RIGHT)
	{
		return 2;

	}
	else if(indi == PlannerHNS::INDICATOR_BOTH)
		return 3;
	else
		return 0;
}

PlannerHNS::LIGHT_INDICATOR BehaviorPrediction::FromNumbertoIndicator(const int& num)
{
	if(num == 0)
		return PlannerHNS::INDICATOR_NONE;
	else if(num == 1)
		return PlannerHNS::INDICATOR_LEFT;
	else if(num == 2)
		return PlannerHNS::INDICATOR_RIGHT;
	else if(num == 3)
		return PlannerHNS::INDICATOR_BOTH;
	else
		return PlannerHNS::INDICATOR_NONE;
}

void BehaviorPrediction::PredictionStepII(std::vector<ObjParticles*>& part_info)
{
	for(unsigned int i=0; i < part_info.size(); i++)
		SamplesFreshParticles(part_info.at(i));
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
		PlanningHelpers::FixPathDensity(*(parts->m_TrajectoryTracker.at(t)->trajectory), 0.5);
		PlanningHelpers::CalcAngleAndCost(*(parts->m_TrajectoryTracker.at(t)->trajectory));
	}

	for(unsigned int t=0; t < parts->m_TrajectoryTracker.size(); t++)
	{
		RelativeInfo info;
		PlanningHelpers::GetRelativeInfo(*(parts->m_TrajectoryTracker.at(t)->trajectory), parts->obj.center, info);
		unsigned int point_index = 0;
		p.pose = PlanningHelpers::GetFollowPointOnTrajectory(*(parts->m_TrajectoryTracker.at(t)->trajectory), info, PREDICTION_DISTANCE_PERCENTAGE*m_PredictionDistance, point_index);

		if(parts->m_TrajectoryTracker.at(t)->nAliveForward == 0 && parts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_FORWARD_STATE)
		{
			p.beh = PlannerHNS::BEH_FORWARD_STATE;
			parts->m_TrajectoryTracker.at(t)->m_ForwardPart.clear();

			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);

			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			{
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.x += gen_x();
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.y += gen_x();
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.pos.a += gen_a();
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).vel = parts->obj.center.v + fabs(gen_v());
				parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).pose.v = parts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).vel;
				number_of_particles++;
			}
			//std::cout << "Forward Particles for Trajectory: " <<t << ", Is: " <<  parts->m_TrajectoryTracker.at(t)->m_ForwardPart.size()<< std::endl;
		}

//		continue;
//
//		if(parts->m_TrajectoryTracker.at(t)->nAliveLeft == 0 && parts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
//		{
//
//			p.beh = PlannerHNS::BEH_BRANCH_LEFT_STATE;
//			parts->m_TrajectoryTracker.at(t)->m_LeftPart.clear();
//
//			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
//				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);
//
//			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
//			{
//				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.x += gen_x();
//				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.y += gen_x();
//				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.pos.a += gen_a();
//				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).vel = parts->obj.center.v + gen_v()/2.0;
//				parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).pose.v = parts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).vel;
//				number_of_particles++;
//			}
//			//std::cout << "Forward Particles for Trajectory: " << t << ", Is: " <<  parts->m_TrajectoryTracker.at(t)->m_ForwardPart.size()<< std::endl;
//		}
//
//		if(parts->m_TrajectoryTracker.at(t)->nAliveRight == 0 && parts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
//		{
//			p.beh = PlannerHNS::BEH_BRANCH_RIGHT_STATE;
//			parts->m_TrajectoryTracker.at(t)->m_RightPart.clear();
//
//			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
//				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);
//
//			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
//			{
//				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.x += gen_x();
//				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.y += gen_x();
//				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.pos.a += gen_a();
//				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).vel = parts->obj.center.v + gen_v()/2.0;
//				parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).pose.v = parts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).vel;
//				number_of_particles++;
//			}
//		}
//
//		if(parts->m_TrajectoryTracker.at(t)->nAliveStop == 0 )
//		{
//			p.beh = PlannerHNS::BEH_STOPPING_STATE;
//			parts->m_TrajectoryTracker.at(t)->m_StopPart.clear();
//
//			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
//				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);
//
//			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
//			{
//				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.x += gen_x();
//				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.y += gen_x();
//				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.pos.a += gen_a();
//				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).vel = 0;
//				parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).pose.v = parts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).vel;
//				number_of_particles++;
//			}
//		}
//
//
//		if(parts->m_TrajectoryTracker.at(t)->nAliveYield == 0 )
//		{
//			p.beh = PlannerHNS::BEH_YIELDING_STATE;
//			parts->m_TrajectoryTracker.at(t)->m_YieldPart.clear();
//
//			for(unsigned int i=0; i < BEH_PARTICLES_NUM; i++)
//				parts->m_TrajectoryTracker.at(t)->InsertNewParticle(p);
//
//			for(unsigned int i=0; i < parts->m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
//			{
//				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.x += gen_x();
//				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.y += gen_x();
//				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.pos.a += gen_a();
//				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).vel = parts->obj.center.v - fabs(gen_v());
//				parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).pose.v = parts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).vel;
//				number_of_particles++;
//			}
//		}
	}
}

double BehaviorPrediction::CalcIndicatorWeight(PlannerHNS::LIGHT_INDICATOR p_ind, PlannerHNS::LIGHT_INDICATOR obj_ind)
{
	if((obj_ind == PlannerHNS::INDICATOR_LEFT || obj_ind == PlannerHNS::INDICATOR_RIGHT) && p_ind == obj_ind)
		return 0.9;
	else
		return 0.1;
}

void BehaviorPrediction::CalOnePartWeightII(ObjParticles* pParts,Particle& p)
{
	if(p.bDeleted) return;

	p.pose_w = exp(-(0.5*pow((p.pose.pos.x - pParts->obj.center.pos.x)/10.,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+ pow((p.pose.pos.y - pParts->obj.center.pos.y)/10.0,2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)));
	//p.dir_w  = exp(-(pow(UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(p.pose.pos.a,  pParts->obj.center.pos.a),2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR)));
	p.dir_w  = fabs(UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(p.pose.pos.a,  pParts->obj.center.pos.a));
	if(p.dir_w != 0)
		p.dir_w = 1./p.dir_w;
	p.vel_w  = exp(-(pow(p.vel - pParts->obj.center.v,2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR)));
	//std::cout << p.vel_w;
	p.ind_w  = CalcIndicatorWeight(FromNumbertoIndicator(p.indicator), pParts->obj.indicator_state);

	p.ind_w  -= p.ind_w*MEASURE_IND_ERROR;

	pParts->pose_w_t += p.pose_w;
	pParts->dir_w_t += p.dir_w;
	pParts->vel_w_t += p.vel_w;
	pParts->ind_w_t += p.ind_w;

	if(p.pose_w > pParts->pose_w_max)
		pParts->pose_w_max = p.pose_w;
	if(p.dir_w > pParts->dir_w_max)
		pParts->dir_w_max = p.dir_w;
	if(p.vel_w > pParts->vel_w_max)
		pParts->vel_w_max = p.vel_w;
	if(p.ind_w > pParts->ind_w_max)
		pParts->ind_w_max = p.ind_w;

	if(p.pose_w < pParts->pose_w_min)
		pParts->pose_w_min = p.pose_w;
	if(p.dir_w < pParts->dir_w_min)
		pParts->dir_w_min = p.dir_w;
	if(p.vel_w < pParts->vel_w_min)
		pParts->vel_w_min = p.vel_w;
	if(p.ind_w < pParts->ind_w_min)
		pParts->ind_w_min = p.ind_w;
}

void BehaviorPrediction::NormalizeOnePartWeightII(ObjParticles* pParts,Particle& p)
{
	if(p.bDeleted) return;

	double pose_diff  = pParts->pose_w_max-pParts->pose_w_min;
	double dir_diff = pParts->dir_w_max-pParts->dir_w_min;
	double vel_diff = pParts->vel_w_max-pParts->vel_w_min;
	double ind_diff = pParts->ind_w_max-pParts->ind_w_min;

	double epsilon = 0.01;

	if(fabs(pose_diff) > epsilon)
		p.pose_w = p.pose_w/pose_diff;
	else
		p.pose_w = 0;

	if(p.pose_w > 1.0 ) p.pose_w = 1.0;

	if(fabs(dir_diff) > epsilon)
		p.dir_w = p.dir_w/dir_diff;
	else
		p.dir_w = 0;

	if(p.dir_w > 1.0 ) p.dir_w = 1.0;

	if(fabs(vel_diff) > epsilon)
		p.vel_w = p.vel_w/vel_diff;
	else
		p.vel_w = 0;

	if(p.vel_w > 1.0) p.vel_w = 1.0;

	if(fabs(ind_diff) > epsilon)
		p.ind_w = p.ind_w/ind_diff;
	else
		p.ind_w = 0;

	if(p.ind_w > 1.0) p.ind_w = 1.0;

	p.w = p.pose_w*POSE_FACTOR + p.dir_w*DIRECTION_FACTOR + p.vel_w*VELOCITY_FACTOR + p.ind_w*INDICATOR_FACTOR;
	//p.w = p.pose_w*0.1 + p.vel_w*0.2 + p.dir_w * 0.2 + p.ind_w + 0.5;

	if(p.w >= pParts->max_w)
		pParts->max_w = p.w;

	if(p.w <= pParts->min_w)
		pParts->min_w = p.w;

	  pParts->all_w += p.w;
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

//	std::cout << "Velocity Diff: ";
	for(unsigned int i = 0 ; i < pParts->m_AllParticles.size(); i++)
	{
		CalOnePartWeightII(pParts, *pParts->m_AllParticles.at(i));
	//	std::cout << ", ";
	}
	//std::cout << std::endl;

	//Normalize
	pParts->max_w = -9999999;
	pParts->min_w = 9999999;
	pParts->all_w = 0;

	for(unsigned int i = 0 ; i < pParts->m_AllParticles.size(); i++)
	{
		NormalizeOnePartWeightII(pParts, *pParts->m_AllParticles.at(i));
	}

//	std::cout << "Behavior Weights ------------------------------------------------ " << std::endl;
//	std::cout << "Max: " << pParts->max_w << ", Min: " << pParts->min_w << std::endl;
//	std::cout << "Keep Percent Value: " << half_percent_value <<  std::endl;
//
//	std::vector<std::pair<int, double> > traj_list;
//
//	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
//	{
//		double avg_sum = 0;
//		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
//		{
//			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).w;
//		}
//
//		if(pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size() > 0)
//		{
//			double w = avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size();
//			traj_list.push_back(std::make_pair(t, w));
//			//std::cout << "     F: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveForward << ", W:" << w << std::endl;
////			if(pParts->m_TrajectoryTracker.at(t)->trajectory->size() > 0)
////				pParts->m_TrajectoryTracker.at(t)->trajectory->at(0).collisionCost = w;
//		}
//
//		continue;
//
//		avg_sum = 0;
//		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
//		{
//			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i).w;
//		}
//
//		if(pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size() > 0)
//			std::cout << "     L: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveLeft << ", W:" << avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size() << std::endl;
//
//		avg_sum = 0;
//		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
//		{
//			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_RightPart.at(i).w;
//		}
//		if(pParts->m_TrajectoryTracker.at(t)->m_RightPart.size() > 0)
//			std::cout << "     R: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveRight << ", W:" << avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_RightPart.size() << std::endl;
//
//		avg_sum = 0;
//		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
//		{
//			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).w;
//		}
//		if(pParts->m_TrajectoryTracker.at(t)->m_StopPart.size() > 0)
//			std::cout << "     S: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveStop << ", W:" << avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_StopPart.size() << std::endl;
//
//		avg_sum = 0;
//		for(unsigned int i = 0; i < pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
//		{
//			avg_sum += pParts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i).w;
//		}
//		if(pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size() > 0)
//			std::cout << "     Y: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveYield << ", W:" << avg_sum/(double)pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size() << std::endl;
//	}
//
//	std::sort(traj_list.begin(), traj_list.end(),sort_trajectories);
//
//	if(traj_list.size() > 0)
//	{
//		std::cout << "First Weight: " << traj_list.at(0).second << "Last Weight: " << traj_list.at(traj_list.size()-1).second << std::endl;
//		if(pParts->m_TrajectoryTracker.size() > traj_list.at(0).first)
//			pParts->m_TrajectoryTracker.at(traj_list.at(0).first)->trajectory->at(0).collisionCost = 1;
//	}
//
//	std::cout << "------------------------------------------------ --------------" << std::endl;
}

void BehaviorPrediction::RemoveWeakParticles(ObjParticles* pParts)
{

	if(pParts->max_w == 0 || fabs(pParts->max_w - pParts->min_w) < 0.1 )
	{
		m_bCanDecide = false;
		return;
	}
	m_bCanDecide = true;


	double critical_val = pParts->min_w + (pParts->max_w - pParts->min_w)*KEEP_PERCENTAGE;


	for(int i =0 ; i < pParts->m_AllParticles.size(); i++)
	{
		if(pParts->m_AllParticles.at(i)->w < critical_val)
		{
			pParts->m_AllParticles.at(i)->pTraj->DeleteParticle(*(pParts->m_AllParticles.at(i)), pParts->m_AllParticles.at(i)->original_index);
			pParts->m_AllParticles.erase(pParts->m_AllParticles.begin()+i);
			i--;
		}
	}
//	std::sort(pParts->m_AllParticles.begin(),pParts->m_AllParticles.end(), sort_weights);

//	int remove_number = pParts->m_AllParticles.size() * KEEP_PERCENTAGE;
//
//	while(remove_number > 0)
//	{
//		int i = pParts->m_AllParticles.size()-1;
//		if(i >= 0)
//		{
//			pParts->m_AllParticles.at(i)->pTraj->DeleteParticle(*(pParts->m_AllParticles.at(i)), pParts->m_AllParticles.at(i)->original_index);
//			pParts->m_AllParticles.erase(pParts->m_AllParticles.begin()+i);
//		}
//
//		remove_number--;
//	}
}

void BehaviorPrediction::CalculateProbabilities(ObjParticles* pParts)
{
	pParts->best_beh_track = nullptr;
	if(m_bCanDecide)
		pParts->CalculateProbabilities();

//	std::vector<std::pair<int, double> > traj_list;
//
//	std::cout << "Behavior Prob ------------------------------------------------ : " << pParts->m_TrajectoryTracker.size() << std::endl;
//	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
//	{
//		std::cout << "F: N: Trajectory (" << t << ") , Probability: " << pParts->m_TrajectoryTracker.at(t)->pForward << std::endl;
//		traj_list.push_back(std::make_pair(t, pParts->m_TrajectoryTracker.at(t)->pForward));
////		if(pParts->m_TrajectoryTracker.at(t)->trajectory->size() > 0)
////			pParts->m_TrajectoryTracker.at(t)->trajectory->at(0).collisionCost = pParts->m_TrajectoryTracker.at(t)->pForward;
//
//		//std::cout << "     L: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveLeft << ", P:" << pParts->m_TrajectoryTracker.at(t)->pLeft << std::endl;
//		//std::cout << "     R: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveRight << ", P:" << pParts->m_TrajectoryTracker.at(t)->pRight << std::endl;
//		//std::cout << "     S: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveStop << ", P:" << pParts->m_TrajectoryTracker.at(t)->pStop << std::endl;
//		//std::cout << "     Y: N:" << pParts->m_TrajectoryTracker.at(t)->nAliveYield << ", P:" << pParts->m_TrajectoryTracker.at(t)->pYield << std::endl << std::endl;
//	}
//
//
//	std::sort(traj_list.begin(), traj_list.end(),sort_trajectories);
//
//	std::cout << "------------------------------------------------ --------------" << std::endl;
//
//	if(traj_list.size() > 0)
//	{
//		std::cout << "Trajectory (" << traj_list.at(0).first << "), Max_P: " << traj_list.at(0).second << " , Min_P: " << traj_list.at(traj_list.size()-1).second << std::endl;
//
//		for(unsigned int j =1; j < traj_list.size() ; j++)
//			pParts->m_TrajectoryTracker.at(traj_list.at(j).first)->trajectory->at(0).collisionCost = 0.25;
//
//		if(traj_list.at(0).second == traj_list.at(traj_list.size()-1).second)
//		{
//			for(unsigned int j =0; j < traj_list.size() ; j++)
//				pParts->m_TrajectoryTracker.at(traj_list.at(j).first)->trajectory->at(0).collisionCost = 1;
//		}
//		else
//		{
//			if(pParts->m_TrajectoryTracker.size() > traj_list.at(0).first)
//				pParts->m_TrajectoryTracker.at(traj_list.at(0).first)->trajectory->at(0).collisionCost = 1;
//		}
//	}
//	std::cout << "------------------------------------------------ --------------" << std::endl;


	std::cout << "Behavior Prob ------------------------------------------------ : " << pParts->m_TrajectoryTracker.size() << std::endl;
	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size() ; t++)
			pParts->m_TrajectoryTracker.at(t)->trajectory->at(0).collisionCost = 1;

	if(pParts->best_beh_track != nullptr)
	{
		std::string str_beh = "Unknown";
		if(pParts->best_beh_track->best_beh == BEH_STOPPING_STATE)
			str_beh = "Stopping";
		else if(pParts->best_beh_track->best_beh == BEH_FORWARD_STATE)
			str_beh = "Forward";

		std::cout << "Trajectory (" << pParts->i_best_track << "), P: " << pParts->best_beh_track->best_p << " , Beh (" << pParts->best_beh_track->best_beh << ", " << str_beh << ")" << std::endl;
		pParts->best_beh_track->trajectory->at(0).collisionCost = 1;

		for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size() ; t++)
		{
			if(t != pParts->i_best_track)
				pParts->m_TrajectoryTracker.at(t)->trajectory->at(0).collisionCost = 0.25;
		}
	}

	std::cout << "------------------------------------------------ --------------" << std::endl;
}

void BehaviorPrediction::SamplesFreshParticles(ObjParticles* pParts)
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
	p.pose = pParts->obj.center;
	p.vel = 0;
	p.acc = 0;
	p.indicator = 0;
	bool bRegenerate = true;

	if(UtilityHNS::UtilityH::GetTimeDiffNow(m_GenerationTimer) > 2)
	{
		UtilityHNS::UtilityH::GetTickCount(m_GenerationTimer);
		bRegenerate = true;
	}

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		PlanningHelpers::FixPathDensity(*(pParts->m_TrajectoryTracker.at(t)->trajectory), 0.5);
		PlanningHelpers::CalcAngleAndCost(*(pParts->m_TrajectoryTracker.at(t)->trajectory));
	}

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		RelativeInfo info;
		PlanningHelpers::GetRelativeInfo(*(pParts->m_TrajectoryTracker.at(t)->trajectory), pParts->obj.center, info);
		unsigned int point_index = 0;
		p.pose = PlanningHelpers::GetFollowPointOnTrajectory(*(pParts->m_TrajectoryTracker.at(t)->trajectory), info, PREDICTION_DISTANCE_PERCENTAGE*m_PredictionDistance, point_index);

		if(pParts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_FORWARD_STATE && pParts->m_TrajectoryTracker.at(t)->nAliveForward < BEH_PARTICLES_NUM)
		{
			p.beh = PlannerHNS::BEH_FORWARD_STATE;
			int nPs = BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveForward;

			for(unsigned int i=0; i < nPs; i++)
			{
				Particle p_new = p;
				p_new.pose.pos.x += gen_x();
				p_new.pose.pos.y += gen_x();
				p_new.pose.pos.a += gen_a();
				p_new.vel = pParts->obj.center.v + fabs(gen_v());
				p_new.pose.v = p_new.vel;
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
			}

			//std::cout << "For Path(" << t << ") New Particles: " << nPs << ", Total Particles( " << pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size() << ", " << pParts->m_TrajectoryTracker.at(t)->nAliveForward << ")" << std::endl;
		}

//		if(pParts->m_TrajectoryTracker.at(t)->nAliveStop < BEH_PARTICLES_NUM)
//		{
//			p.beh = PlannerHNS::BEH_STOPPING_STATE;
//			int nPs = BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveStop;
//
//			for(unsigned int i=0; i < nPs; i++)
//			{
//				Particle p_new = p;
//				p_new.pose.pos.x += gen_x();
//				p_new.pose.pos.y += gen_x();
//				p_new.pose.pos.a += gen_a();
//				p_new.vel = 0;
//				p_new.pose.v = pParts->obj.center.v + fabs(gen_v());
//				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
//			}
//
//			//std::cout << "For Path(" << t << ") New Particles: " << nPs << ", Total Particles( " << pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size() << ", " << pParts->m_TrajectoryTracker.at(t)->nAliveForward << ")" << std::endl;
//		}
	}
}

void BehaviorPrediction::ReSamplesParticlesII(ObjParticles* pParts)
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

	double dx = 0;
	double dy = 0;
	std::vector<Particle> part_list;
	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		if(pParts->m_TrajectoryTracker.at(t)->nAliveForward > 0 && pParts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_FORWARD_STATE)
		{
			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveForward)/pParts->m_TrajectoryTracker.at(t)->nAliveForward);
			part_list.clear();
			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
			{
				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i);

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
			{
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
			}
		}
		//std::cout << "For Path(" << t << ") New Particles: " << part_list.size() << ", Total Particles: " << pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size() << std::endl;

//		if(pParts->m_TrajectoryTracker.at(t)->nAliveLeft > 0 && pParts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
//		{
//			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveLeft)/pParts->m_TrajectoryTracker.at(t)->nAliveLeft);
//			part_list.clear();
//			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size(); i++)
//			{
//				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_LeftPart.at(i);
//				if(USE_OPEN_PLANNER_MOVE == 0)
//				{
//				p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
//				p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
//				}
//				else
//				  {
//				curr_behavior = pParts->m_TrajectoryTracker.at(t)->m_SinglePathDecisionMaker.DoOneStep(dt, p->pose, *(pParts->m_TrajectoryTracker.at(t)->trajectory), trafficLight,control_u);
//				p->pose.pos.x += curr_behavior.maxVelocity * dt * cos(p->pose.pos.a);
//				p->pose.pos.y += curr_behavior.maxVelocity * dt * sin(p->pose.pos.a);
//				p->pose.pos.a += control_u.speed * dt * tan(control_u.steer)  / pParts->obj.l*0.75;
//				//std::cout << "Move Left: " << t << ", " << pParts->m_TrajectoryTracker.at(t)->m_LeftPart.size() << std::endl;
//				  }
//
//
//				for(int ic=0; ic < n_per_part; ic++)
//				{
//					Particle np = *p;
//					np.pose.pos.x +=  gen_x();
//					np.pose.pos.y +=  gen_x();
//					np.pose.pos.a +=  gen_a();
//					np.vel += gen_v()/2.0;
//					part_list.push_back(np);
//				}
//			}
//			for(unsigned int ip=0; ip < part_list.size(); ip++)
//				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
//		}
//
//		if(pParts->m_TrajectoryTracker.at(t)->nAliveRight > 0 && pParts->m_TrajectoryTracker.at(t)->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
//		{
//			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveRight)/pParts->m_TrajectoryTracker.at(t)->nAliveRight)+1;
//			part_list.clear();
//			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_RightPart.size(); i++)
//			{
//				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_RightPart.at(i);
//				if(USE_OPEN_PLANNER_MOVE == 0)
//				{
//				p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
//				p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
//				}
//				else
//				  {
//				curr_behavior = pParts->m_TrajectoryTracker.at(t)->m_SinglePathDecisionMaker.DoOneStep(dt, p->pose, *(pParts->m_TrajectoryTracker.at(t)->trajectory), trafficLight,control_u);
//				p->pose.pos.x += curr_behavior.maxVelocity * dt * cos(p->pose.pos.a);
//				p->pose.pos.y += curr_behavior.maxVelocity * dt * sin(p->pose.pos.a);
//				p->pose.pos.a += control_u.speed * dt * tan(control_u.steer)  / pParts->obj.l*0.75;
//				//std::cout << "Move Right: " << t << ", " << pParts->m_TrajectoryTracker.at(t)->m_RightPart.size() << std::endl;
//				  }
//
//
//				for(int ic=0; ic < n_per_part; ic++)
//				{
//					Particle np = *p;
//					np.pose.pos.x +=  gen_x();
//					np.pose.pos.y +=  gen_x();
//					np.pose.pos.a +=  gen_a();
//					np.vel += gen_v()/2.0;
//					part_list.push_back(np);
//				}
//			}
//
//			for(unsigned int ip=0; ip < part_list.size(); ip++)
//				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
//		}
//
//		if(pParts->m_TrajectoryTracker.at(t)->nAliveStop > 0 )
//		{
//			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveStop)/pParts->m_TrajectoryTracker.at(t)->nAliveStop)+1;
//			part_list.clear();
//			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
//			{
//				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i);
//				if(USE_OPEN_PLANNER_MOVE == 0)
//				{
//				p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
//				p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
//				}
//				else
//				  {
//				curr_behavior = pParts->m_TrajectoryTracker.at(t)->m_SinglePathDecisionMaker.DoOneStep(dt, p->pose, *(pParts->m_TrajectoryTracker.at(t)->trajectory), trafficLight,control_u);
//				p->pose.pos.x += curr_behavior.maxVelocity * dt * cos(p->pose.pos.a);
//				p->pose.pos.y += curr_behavior.maxVelocity * dt * sin(p->pose.pos.a);
//				p->pose.pos.a += control_u.speed * dt * tan(control_u.steer)  / pParts->obj.l*0.75;
//				//std::cout << "Move Stop: " << curr_behavior.maxVelocity << ", " << pParts->m_TrajectoryTracker.at(t)->m_StopPart.size() << std::endl;
//				  }
//
//				for(int ic=0; ic < n_per_part; ic++)
//				{
//					Particle np = *p;
//					np.pose.pos.x +=  gen_x();
//					np.pose.pos.y +=  gen_x();
//					np.pose.pos.a +=  gen_a();
//					np.vel = 0;
//					part_list.push_back(np);
//				}
//			}
//
//			for(unsigned int ip=0; ip < part_list.size(); ip++)
//				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
//		}
//
//
//		if(pParts->m_TrajectoryTracker.at(t)->nAliveYield > 0 )
//		{
//			int n_per_part = ((BEH_PARTICLES_NUM - pParts->m_TrajectoryTracker.at(t)->nAliveYield)/pParts->m_TrajectoryTracker.at(t)->nAliveYield)+1;
//			part_list.clear();
//			for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size(); i++)
//			{
//				Particle* p = &pParts->m_TrajectoryTracker.at(t)->m_YieldPart.at(i);
//				if(USE_OPEN_PLANNER_MOVE == 0)
//				{
//				p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
//				p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
//				}
//				else
//				  {
//				curr_behavior = pParts->m_TrajectoryTracker.at(t)->m_SinglePathDecisionMaker.DoOneStep(dt, p->pose, *(pParts->m_TrajectoryTracker.at(t)->trajectory), trafficLight,control_u);
//				p->pose.pos.x += curr_behavior.maxVelocity * dt * cos(p->pose.pos.a);
//				p->pose.pos.y += curr_behavior.maxVelocity * dt * sin(p->pose.pos.a);
//				p->pose.pos.a += control_u.speed * dt * tan(control_u.steer)  / pParts->obj.l*0.75;
//			//std::cout << "Move Yield: " << curr_behavior.maxVelocity << ", " << pParts->m_TrajectoryTracker.at(t)->m_YieldPart.size() << std::endl;
//				  }
//
//				for(int ic=0; ic < n_per_part; ic++)
//				{
//					Particle np = *p;
//					np.pose.pos.x +=  gen_x();
//					np.pose.pos.y +=  gen_x();
//					np.pose.pos.a +=  gen_a();
//					np.vel = pParts->obj.center.v - fabs(gen_v());
//					part_list.push_back(np);
//				}
//			}
//
//			for(unsigned int ip=0; ip < part_list.size(); ip++)
//				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(part_list.at(ip));
//		}
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

void BehaviorPrediction::CollectParticles(ObjParticles* pParts)
{
	pParts->m_AllParticles.clear();
	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.size(); i++)
		{
			pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i).original_index = i;
			pParts->m_AllParticles.push_back(&pParts->m_TrajectoryTracker.at(t)->m_ForwardPart.at(i));
		}

		for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_StopPart.size(); i++)
		{
			pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i).original_index = i;
			pParts->m_AllParticles.push_back(&pParts->m_TrajectoryTracker.at(t)->m_StopPart.at(i));
		}
	}
}

void BehaviorPrediction::MoveParticles(ObjParticles* pParts)
{
	double dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_ResamplingTimer);
	if(m_bStepByStep)
		dt = 0.02;

	UtilityHNS::UtilityH::GetTickCount(m_ResamplingTimer);
	PlannerHNS::BehaviorState curr_behavior;
	PlannerHNS::VehicleState control_u;
	PassiveDecisionMaker decision_make;
	PlannerHNS::CAR_BASIC_INFO carInfo;
	carInfo.width = pParts->obj.w;
	carInfo.length = pParts->obj.l;
	carInfo.max_acceleration = 3.0;
	carInfo.max_deceleration = -3;
	carInfo.max_speed_forward = pParts->obj.center.v+1.0;
	carInfo.min_speed_forward = 0;
	carInfo.max_steer_angle = 0.4;
	carInfo.min_steer_angle = -0.4;
	carInfo.turning_radius = 7.2;
	carInfo.wheel_base = carInfo.length*0.75;

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		PlanningHelpers::GenerateRecommendedSpeed(*(pParts->m_TrajectoryTracker.at(t)->trajectory), carInfo.max_speed_forward, 1.0);
	}

	for(unsigned int i=0; i < pParts->m_AllParticles.size(); i++)
	{
		Particle* p = pParts->m_AllParticles.at(i);

		if(USE_OPEN_PLANNER_MOVE == 0)
		  {
			p->pose.pos.x += pParts->obj.center.v * dt * cos(p->pose.pos.a);
			p->pose.pos.y += pParts->obj.center.v * dt * sin(p->pose.pos.a);
		  }
		else
		  {
			curr_behavior = decision_make.MoveStep(dt, p->pose, *(p->pTraj->trajectory),carInfo);
			p->vel = curr_behavior.maxVelocity;
			p->indicator = FromIndicatorToNumber(curr_behavior.indicator);

			if(curr_behavior.state == PlannerHNS::STOPPING_STATE && p->beh == PlannerHNS::BEH_YIELDING_STATE)
				p->vel += 1;
			else if(p->beh == PlannerHNS::BEH_YIELDING_STATE)
				p->vel = p->vel/2.0;
			else if(curr_behavior.state != PlannerHNS::STOPPING_STATE && p->beh == PlannerHNS::BEH_STOPPING_STATE)
				p->vel += 1;
			else if(p->beh == PlannerHNS::BEH_STOPPING_STATE)
			{
				p->vel = 0;
			}

		  }
	}
}

} /* namespace PlannerHNS */
