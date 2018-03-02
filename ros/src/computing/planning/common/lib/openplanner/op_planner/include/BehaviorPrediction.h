/*
 * BehaviorPrediction.h
 *
 *  Created on: Jul 6, 2017
 *      Author: user
 */

#ifndef BEHAVIORPREDICTION_H_
#define BEHAVIORPREDICTION_H_

#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>

#include "PlannerH.h"
#include "UtilityH.h"
#include "PassiveDecisionMaker.h"

namespace PlannerHNS
{

#define MOTION_POSE_ERROR 0.25 // 50 cm pose error
#define MOTION_ANGLE_ERROR 0.08 // 0.05 rad angle error
#define MOTION_VEL_ERROR 0.1
#define MEASURE_POSE_ERROR 0.25
#define MEASURE_ANGLE_ERROR 0.08
#define MEASURE_VEL_ERROR 0.1
#define MEASURE_IND_ERROR 0.1

#define PREDICTION_DISTANCE_PERCENTAGE 0.2

#define BEH_PARTICLES_NUM 50
#define BEH_MIN_PARTICLE_NUM 0

#define POSE_FACTOR 0.0
#define DIRECTION_FACTOR 0.0
#define VELOCITY_FACTOR 0.0
#define INDICATOR_FACTOR 1

#define KEEP_PERCENTAGE 0.75

#define FIXED_PLANNING_DISTANCE 10

#define MIN_PREDICTION_DISTANCE 5

#define USE_OPEN_PLANNER_MOVE 1

typedef boost::mt19937 ENG;
typedef boost::normal_distribution<double> NormalDIST;
typedef boost::variate_generator<ENG, NormalDIST> VariatGEN;

class TrajectoryTracker;

class Particle
{
public:
	bool bDeleted;
	BEH_STATE_TYPE beh; //[Stop, Yielding, Forward, Branching]
	int vel; //[0 -> Stop,1 -> moving]
	int acc; //[-1 ->Slowing, 0, Stopping, 1 -> accelerating]
	int indicator; //[0 -> No, 1 -> Left, 2 -> Right , 3 -> both]
	WayPoint pose;
	bool bStopLine;
	double w;
	double pose_w;
	double dir_w;
	double vel_w;
	double acl_w;
	double ind_w;
	//BehTrajInfo* pTrajInfo;
	TrajectoryTracker* pTraj;
	int original_index;

	Particle()
	{
		original_index = 0;
		//pTrajInfo = 0;
		bStopLine = false;
		bDeleted = false;
		pTraj = nullptr;
		w = 0;
		pose_w = 0;
		dir_w = 0;
		vel_w = 0;
		acl_w = 0;
		ind_w = 0;
		beh = BEH_STOPPING_STATE;
		vel = 0;
		acc = 0;
		indicator = 0;
	}
};

class TrajectoryTracker
{
public:
//	unsigned int index;
//	unsigned int prev_index;
	BEH_STATE_TYPE beh;
	BEH_STATE_TYPE best_beh;
	double best_p;
	std::vector<int> ids;
	std::vector<int> path_ids;
	WayPoint path_last_pose;
	std::vector<WayPoint>* trajectory;

	std::vector<Particle> m_ForwardPart;
	std::vector<Particle> m_StopPart;
	std::vector<Particle> m_YieldPart;
	std::vector<Particle> m_LeftPart;
	std::vector<Particle> m_RightPart;
	BehaviorState m_CurrBehavior;

	int nAliveStop;
	int nAliveYield;
	int nAliveForward;
	int nAliveLeft;
	int nAliveRight;

	double pStop;
	double pYield;
	double pForward;
	double pLeft;
	double pRight;

	PassiveDecisionMaker m_SinglePathDecisionMaker;

	TrajectoryTracker()
	{
		beh = BEH_STOPPING_STATE;
		//prev_index = 0;
		trajectory = 0;
		//index = 0;
		nAliveStop = 0;
		nAliveYield = 0;
		nAliveForward = 0;
		nAliveLeft = 0;
		nAliveRight = 0;

		pStop  = 0;
		pYield = 0;
		pForward = 0;
		pLeft = 0;
		pRight = 0;
		best_beh = PlannerHNS::BEH_STOPPING_STATE;
		best_p = 0;
	}

	virtual ~TrajectoryTracker()
	{
		//std::cout << "TrajectoryTracker::Destructor:" << this << std::endl;
	}

	void InitDecision()
	{
	}

	TrajectoryTracker(const TrajectoryTracker& obj)
	{

		ids = obj.ids;
		path_ids = obj.path_ids;
		path_last_pose = obj.path_last_pose;
		beh = obj.beh;
		//prev_index = obj.prev_index;
		//index = obj.index;
		trajectory = obj.trajectory;
		nAliveStop = obj.nAliveStop;
		nAliveYield = obj.nAliveYield;
		nAliveForward = obj.nAliveForward;
		nAliveLeft = obj.nAliveLeft;
		nAliveRight = obj.nAliveRight;

		pStop  = obj.pStop;
		pYield = obj.pYield;
		pForward = obj.pForward;
		pLeft = obj.pLeft;
		pRight = obj.pRight;
		best_beh = obj.best_beh;
		best_p = obj.best_p;
		m_SinglePathDecisionMaker = obj.m_SinglePathDecisionMaker;

		m_ForwardPart = obj.m_ForwardPart;
		m_StopPart = obj.m_StopPart;
		m_YieldPart = obj.m_YieldPart;
		m_LeftPart = obj.m_LeftPart;
		m_RightPart = obj.m_RightPart;
		m_CurrBehavior = obj.m_CurrBehavior;
	}

	TrajectoryTracker(std::vector<PlannerHNS::WayPoint>& path, const unsigned int& _index)
	{

		if(path.size()>0)
		{
			beh = path.at(0).beh_state;
			//std::cout << "New Path: Beh: " << beh << ", index: " << _index << ", LaneID_0: " << path.at(0).laneId << ", LaneID_1: "<< path.at(1).laneId << std::endl;
		}

		//prev_index = _index;
		//index = _index;
		trajectory = &path;
		int prev_id = -10;
		int curr_id = -10;
		ids.clear();
		path_ids.clear();
		for(unsigned int i = 0; i < path.size(); i++)
		{
			curr_id = path.at(i).laneId;
			path_ids.push_back(curr_id);

			if(curr_id != prev_id)
			{
				ids.push_back(curr_id);
				prev_id = curr_id;
			}
		}

		path_last_pose = path.at(path.size()-1);

		nAliveStop = 0;
		nAliveYield = 0;
		nAliveForward = 0;
		nAliveLeft = 0;
		nAliveRight = 0;

		pStop  = 0;
		pYield = 0;
		pForward = 0;
		pLeft = 0;
		pRight = 0;
		best_beh = PlannerHNS::BEH_STOPPING_STATE;
		best_p = 0;

		InitDecision();
	}

	void UpdatePathAndIndex(std::vector<PlannerHNS::WayPoint>& _path, const unsigned int& _index)
	{
		if(_path.size() == 0) return;

		beh = _path.at(0).beh_state;

		//index = _index;
		trajectory = &_path;
		int prev_id = -10;
		int curr_id = -10;
		ids.clear();
		path_ids.clear();
		for(unsigned int i = 0; i < _path.size(); i++)
		{
			curr_id = _path.at(i).laneId;
			path_ids.push_back(curr_id);
			if(curr_id != prev_id)
			{
				ids.push_back(curr_id);
				prev_id = curr_id;
			}
		}

		path_last_pose = _path.at(_path.size()-1);
	}

	double CalcMatchingPercentage(const std::vector<PlannerHNS::WayPoint>& _path)
	{
		if(_path.size() == 0) return 0;

		if(beh != _path.at(0).beh_state) return 0;

		int nCount = 0, nIds = 0;
		int prev_id = -10;
		int curr_id = -10;
		std::vector<int> _ids;

		for(unsigned int i = 0; i < _path.size(); i++)
		{
			curr_id = _path.at(i).laneId;
			if(i < path_ids.size())
			{
				nCount++;
				if(curr_id == path_ids.at(i))
					nIds++;
			}

			if(curr_id != prev_id)
			{
				_ids.push_back(curr_id);
				prev_id = curr_id;
			}
		}

		int nEqualities = 0;
		for(unsigned int i=0; i < _ids.size(); i++)
		{
			for(unsigned int j=0; j < ids.size(); j++)
			{
				if(_ids.at(i) == ids.at(j))
				{
					nEqualities++;
					break;
				}
			}
		}

		if(_ids.size() == ids.size() && ids.size() == nEqualities) // perfect match
			return 1;

		WayPoint curr_last_pose = _path.at(_path.size()-1);
		double nMatch = (double)nIds/(double)nCount;
		double _d = hypot(path_last_pose.pos.y-curr_last_pose.pos.y, path_last_pose.pos.x-curr_last_pose.pos.x);

		double dCost = _d/FIXED_PLANNING_DISTANCE;
		if(dCost > 1.0)
			dCost = 1.0;
		double dMatch = 1.0 - dCost;

		double _a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(path_last_pose.pos.a, curr_last_pose.pos.a);
		double aCost = _a_diff/M_PI;
		if(aCost > 1.0)
			aCost = 1.0;
		double aMatch = 1.0 - aCost;

		double totalMatch = (nMatch + dMatch + aMatch)/3.0;

		return totalMatch;
	}

	int FindMatchIndex(std::vector<PlannerHNS::WayPoint>& _path, const unsigned int& _index)
	{
		if(_path.size() == 0) return -1;

		if(beh != _path.at(0).beh_state) return -1;

		int prev_id = -10;
		int curr_id = -10;
		std::vector<int> _ids;

		for(unsigned int i = 0; i < _path.size(); i++)
		{
			curr_id = _path.at(i).laneId;
			if(curr_id != prev_id)
			{
				_ids.push_back(curr_id);
				prev_id = curr_id;
			}
		}

		int nEqualities = 0;
		for(unsigned int i=0; i < _ids.size(); i++)
		{
			for(unsigned int j=0; j < ids.size(); j++)
			{
				if(_ids.at(i) == ids.at(j))
				{
					nEqualities++;
					break;
				}
			}
		}

		int nTotal = (int)_ids.size() - nEqualities;

		if(nTotal == 0 || (nTotal == 1 && _ids.size() > 2))
		//if(nTotal == 0 || nEqualities >=2)
		{
			return _index;
		}

		return -1;
	}

	bool MatchIfEqual(const TrajectoryTracker& traj_ids)
	{
		if(beh != traj_ids.beh)
			return false;

		int nEqualities = 0;
		for(unsigned int i=0; i < traj_ids.ids.size(); i++)
		{
			for(unsigned int j=0; j < ids.size(); j++)
			{
				if(traj_ids.ids.at(i) == ids.at(j))
				{
					nEqualities++;
					break;
				}
			}
		}

		int nTotal = (int)traj_ids.ids.size() - nEqualities;

		if(nTotal == 0 || (nTotal == 1 && traj_ids.ids.size() > 2))
		//if(nTotal == 0 || nEqualities >=2)
		{
			//prev_index = index;
			//index = traj_ids.index;
			trajectory = traj_ids.trajectory;
			ids = traj_ids.ids;
			return true;
		}

		return false;
	}

	void InsertNewParticle(const Particle& p)
	{
		if(p.beh == PlannerHNS::BEH_STOPPING_STATE && nAliveStop < BEH_PARTICLES_NUM)
		{
			m_StopPart.push_back(p);
			m_StopPart.at(m_StopPart.size()-1).pTraj = this;
			nAliveStop++;
		}
		else if(p.beh == PlannerHNS::BEH_YIELDING_STATE && nAliveYield < BEH_PARTICLES_NUM)
		{
			m_YieldPart.push_back(p);
			m_YieldPart.at(m_YieldPart.size()-1).pTraj = this;
			nAliveYield++;
		}
		else if(p.beh == PlannerHNS::BEH_FORWARD_STATE && nAliveForward < BEH_PARTICLES_NUM)
		{
			m_ForwardPart.push_back(p);
			m_ForwardPart.at(m_ForwardPart.size()-1).pTraj = this;
			nAliveForward++;
		}
		else if(p.beh == PlannerHNS::BEH_BRANCH_LEFT_STATE && nAliveLeft < BEH_PARTICLES_NUM)
		{
			m_LeftPart.push_back(p);
			m_LeftPart.at(m_LeftPart.size()-1).pTraj = this;
			nAliveLeft++;
		}
		else if(p.beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE && nAliveRight < BEH_PARTICLES_NUM)
		{
			m_RightPart.push_back(p);
			m_RightPart.at(m_RightPart.size()-1).pTraj = this;
			nAliveRight++;
		}
	}

	void DeleteParticle(const Particle& p, const int& _i)
	{
		if(p.beh == PlannerHNS::BEH_STOPPING_STATE && nAliveStop > BEH_MIN_PARTICLE_NUM)
		{
//			m_StopPart.at(_i).bDeleted = true;
//			m_StopPart.at(_i).pTraj = 0;
			m_StopPart.erase(m_StopPart.begin()+_i);
			nAliveStop--;
		}
		else if(p.beh == PlannerHNS::BEH_YIELDING_STATE && nAliveYield > BEH_MIN_PARTICLE_NUM)
		{
//			m_YieldPart.at(_i).bDeleted = true;
//			m_YieldPart.at(_i).pTraj = 0;
			m_YieldPart.erase(m_YieldPart.begin()+_i);
			nAliveYield--;
		}
		else if(p.beh == PlannerHNS::BEH_FORWARD_STATE && nAliveForward > BEH_MIN_PARTICLE_NUM)
		{
//			m_ForwardPart.at(_i).bDeleted = true;
//			m_ForwardPart.at(_i).pTraj = 0;
			m_ForwardPart.erase(m_ForwardPart.begin()+_i);
			nAliveForward--;
		}
		else if(p.beh == PlannerHNS::BEH_BRANCH_LEFT_STATE && nAliveLeft > BEH_MIN_PARTICLE_NUM)
		{
//			m_LeftPart.at(_i).bDeleted = true;
//			m_LeftPart.at(_i).pTraj = 0;
			m_LeftPart.erase(m_LeftPart.begin()+_i);
			nAliveLeft--;
		}
		else if(p.beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE && nAliveRight > BEH_MIN_PARTICLE_NUM)
		{
//			m_RightPart.at(_i).bDeleted = true;
//			m_RightPart.at(_i).pTraj = 0;
			m_RightPart.erase(m_RightPart.begin()+_i);
			nAliveRight--;
		}
	}

	void CalcProbabilities()
	{
		best_beh = PlannerHNS::BEH_STOPPING_STATE;
		pStop  = (double)nAliveStop/(double)BEH_PARTICLES_NUM;
		best_p = pStop;

		pYield = (double)nAliveYield/(double)BEH_PARTICLES_NUM;
		if(pYield > best_p)
		{
			best_p = pYield;
			best_beh = PlannerHNS::BEH_YIELDING_STATE;
		}

		pForward = (double)nAliveForward/(double)BEH_PARTICLES_NUM;
		if(pForward > best_p)
		{
			best_p = pForward;
			best_beh = PlannerHNS::BEH_FORWARD_STATE;
		}

		pLeft = (double)nAliveLeft/(double)BEH_PARTICLES_NUM;
		if(pLeft > best_p)
		{
			best_p = pLeft;
			best_beh = PlannerHNS::BEH_BRANCH_LEFT_STATE;
		}

		pRight = (double)nAliveRight/(double)BEH_PARTICLES_NUM;
		if(pRight > best_p)
		{
			best_p = pRight;
			best_beh = PlannerHNS::BEH_BRANCH_RIGHT_STATE;
		}
	}
};

class ObjParticles
{
public:
	DetectedObject obj;
	std::vector<TrajectoryTracker*> m_TrajectoryTracker;
	std::vector<TrajectoryTracker*> m_TrajectoryTracker_temp;

	std::vector<Particle*> m_AllParticles;

	TrajectoryTracker* best_beh_track;
	int i_best_track;

	PlannerHNS::BehaviorState m_beh;
	double m_PredictionTime;

	double all_w;
	double max_w;
	double min_w;
	double avg_w;
	double pose_w_t;
	double dir_w_t;
	double vel_w_t;
	double acl_w_t;
	double ind_w_t;

	double pose_w_max;
	double dir_w_max;
	double vel_w_max;
	double acl_w_max;
	double ind_w_max;

	double pose_w_min;
	double dir_w_min;
	double vel_w_min;
	double acl_w_min;
	double ind_w_min;

	int n_stop;
	int n_yield;
	int n_left_branch;
	int n_right_branch;

	double p_stop;
	double p_yield;
	double p_left_branch;
	double p_right_branch;

	virtual ~ObjParticles()
	{
		DeleteTheRest(m_TrajectoryTracker);
		m_TrajectoryTracker_temp.clear();
	}

	ObjParticles()
	{
		m_PredictionTime = 0;
		best_beh_track = nullptr;
		i_best_track = -1;
		all_w = 0;
		pose_w_t = 0;
		dir_w_t = 0;
		vel_w_t = 0;
		acl_w_t = 0;
		ind_w_t = 0;
		max_w = -100;
		min_w = 100;
		avg_w = 0;

		pose_w_max = -999999;
		dir_w_max=-999999;
		vel_w_max=-999999;
		acl_w_max=-999999;
		ind_w_max=-999999;

		pose_w_min=999999;
		dir_w_min=999999;
		vel_w_min=999999;
		acl_w_min=999999;
		ind_w_min=999999;

		n_stop = 0;
		n_yield = 0;
		n_left_branch = 0;
		n_right_branch = 0;

		p_stop = 0;
		p_yield = 0;
		p_left_branch = 0;
		p_right_branch = 0;
	}

	void CalculateProbabilities()
	{
		for(unsigned int i = 0; i < m_TrajectoryTracker.size(); i++)
		{
			m_TrajectoryTracker.at(i)->CalcProbabilities();
		}

		if(m_TrajectoryTracker.size() > 0)
		{
			best_beh_track = m_TrajectoryTracker.at(0);
			i_best_track = 0;
		}

		for(unsigned int i = 1; i < m_TrajectoryTracker.size(); i++)
		{
			if(m_TrajectoryTracker.at(i)->best_p > best_beh_track->best_p)
			{
				best_beh_track = m_TrajectoryTracker.at(i);
				i_best_track = i;
			}
		}
	}

	class LLP
	{
	public:
		int new_index;
		double match_percent;
		TrajectoryTracker* pTrack;

		LLP()
		{
			new_index = -1;
			match_percent = -1;
			pTrack = 0;

		}
	};

	void DeleteFromList(std::vector<TrajectoryTracker*>& delete_me_track, TrajectoryTracker* track)
	{
		for(unsigned int k = 0; k < delete_me_track.size(); k++)
		{
			if(delete_me_track.at(k) == track)
			{
				delete_me_track.erase(delete_me_track.begin()+k);
				return;
			}
		}
	}

	void DeleteTheRest(std::vector<TrajectoryTracker*>& delete_me_track)
	{
		for(unsigned int k = 0; k < delete_me_track.size(); k++)
		{
			delete delete_me_track.at(k);
		}

		delete_me_track.clear();
	}

	static bool IsBeggerPercentage(const LLP& p1, const LLP& p2)
	{
		return p1.match_percent > p2.match_percent;
	}

	void MatchWithMax(std::vector<LLP>& matching_list, std::vector<TrajectoryTracker*>& delete_me_track, std::vector<TrajectoryTracker*>& check_list)
	{
		if(matching_list.size() == 0 ) return;

		std::sort(matching_list.begin(), matching_list.end(), IsBeggerPercentage);

		while(matching_list.size()>0)
		{
			LLP f = matching_list.at(0);
			f.pTrack->UpdatePathAndIndex(obj.predTrajectories.at(f.new_index), f.new_index);

			bool bFound = false;
			for(unsigned int k=0; k < check_list.size(); k++)
			{
				if(check_list.at(k) == f.pTrack)
				{
					bFound = true;
					break;
				}
			}

			if(!bFound)
				check_list.push_back(f.pTrack);

			DeleteFromList(delete_me_track, f.pTrack);
			for(int i=0; i < matching_list.size(); i++)
			{
				if(matching_list.at(i).new_index == f.new_index || matching_list.at(i).pTrack == f.pTrack)
				{
					matching_list.erase(matching_list.begin()+i);
					i--;
				}
			}
		}
	}

	void MatchTrajectories()
	{
		m_TrajectoryTracker_temp.clear();
		std::vector<LLP> matching_list;
		std::vector<TrajectoryTracker*> delete_me_track = m_TrajectoryTracker;

		for(unsigned int t = 0; t < obj.predTrajectories.size();t++)
		{
			bool bMatched = false;
			LLP match_item;
			match_item.new_index = t;

			for(unsigned int i = 0; i < m_TrajectoryTracker.size(); i++)
			{

				double vMatch = m_TrajectoryTracker.at(i)->CalcMatchingPercentage(obj.predTrajectories.at(t));
				if(vMatch == 1.0) // perfect match
				{
					m_TrajectoryTracker.at(i)->UpdatePathAndIndex(obj.predTrajectories.at(t), t);

					bool bFound = false;
					for(unsigned int k=0; k < m_TrajectoryTracker_temp.size(); k++)
					{
						if(m_TrajectoryTracker_temp.at(k) == m_TrajectoryTracker.at(i))
						{
							bFound = true;
							break;
						}
					}

					if(!bFound)
						m_TrajectoryTracker_temp.push_back(m_TrajectoryTracker.at(i));


					DeleteFromList(delete_me_track, m_TrajectoryTracker.at(i));

					for(unsigned int k=0; k < matching_list.size(); k++)
					{
						if(matching_list.at(k).pTrack == m_TrajectoryTracker.at(i))
						{
							matching_list.erase(matching_list.begin()+k);
							break;
						}
					}
					m_TrajectoryTracker.erase(m_TrajectoryTracker.begin()+i);
					bMatched = true;
					break;
				}
				else if(vMatch > 0.5) // any matching less than 50%, the trajectory will be considered new
				{
					bMatched = true;
					match_item.match_percent = vMatch;
					match_item.pTrack = m_TrajectoryTracker.at(i);
					matching_list.push_back(match_item);
				}
			}

			if(!bMatched)
			{
				m_TrajectoryTracker_temp.push_back(new TrajectoryTracker(obj.predTrajectories.at(t), t));
			}
		}

		MatchWithMax(matching_list,delete_me_track, m_TrajectoryTracker_temp);
//		std::cout << "Delete Trajecotyr Trackers  : " << delete_me_track.size() << ", New Ones: "  <<m_TrajectoryTracker_temp.size() << ", Extracted: " << obj.predTrajectories.size() << std::endl;
		m_TrajectoryTracker.clear();
		DeleteTheRest(delete_me_track);
		m_TrajectoryTracker = m_TrajectoryTracker_temp;
	}
};

class BehaviorPrediction
{
public:
	BehaviorPrediction();
	virtual ~BehaviorPrediction();
	void DoOneStep(const std::vector<DetectedObject>& obj_list, const WayPoint& currPose, const double& minSpeed, const double& maxDeceleration, RoadNetwork& map);

public:
	std::vector<PassiveDecisionMaker*> m_d_makers;
	double m_MaxLaneDetectionDistance;
	double m_PredictionDistance;
	bool m_bGenerateBranches;
	bool m_bUseFixedPrediction;
	bool m_bStepByStep;
	std::vector<DetectedObject> m_PredictedObjects;
	std::vector<DetectedObject*> m_PredictedObjectsII;

	std::vector<ObjParticles> m_temp_list;
	std::vector<ObjParticles> m_ParticleInfo;

	std::vector<ObjParticles*> m_temp_list_ii;
	std::vector<ObjParticles*> m_ParticleInfo_II;

	struct timespec m_GenerationTimer;
	timespec m_ResamplingTimer;

	bool m_bCanDecide;


protected:
	//int GetTrajectoryPredictedDirection(const std::vector<WayPoint>& path, const PlannerHNS::WayPoint& pose, const double& pred_distance);
	int FromIndicatorToNumber(const PlannerHNS::LIGHT_INDICATOR& ind);
	PlannerHNS::LIGHT_INDICATOR FromNumbertoIndicator(const int& num);
	double CalcIndicatorWeight(PlannerHNS::LIGHT_INDICATOR p_ind, PlannerHNS::LIGHT_INDICATOR obj_ind);

	void CalPredictionTimeForObject(ObjParticles* pCarPart);
	void PredictCurrentTrajectory(RoadNetwork& map, ObjParticles* pCarPart);
	void FilterObservations(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<DetectedObject>& filtered_list);
	void ExtractTrajectoriesFromMap(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<ObjParticles*>& old_list);
	void CalculateCollisionTimes(const double& minSpeed);

	void PredictionStepII(std::vector<ObjParticles*>& part_info);
	void CorrectionStepII(std::vector<ObjParticles*>& part_info);

	void SamplesParticlesII(ObjParticles* parts);
	void SamplesFreshParticles(ObjParticles* pParts);
	void ReSamplesParticlesII(ObjParticles* parts);
	void MoveParticles(ObjParticles* parts);
	void CalculateWeightsII(ObjParticles* pParts);

	void CalOnePartWeightII(ObjParticles* pParts,Particle& p);
	void NormalizeOnePartWeightII(ObjParticles* pParts,Particle& p);

	void CollectParticles(ObjParticles* pParts);

	void RemoveWeakParticles(ObjParticles* pParts);
	void CalculateProbabilities(ObjParticles* pParts);
	static bool sort_weights(const Particle* p1, const Particle* p2)
	{
		return p1->w > p2->w;
	}

	static bool sort_trajectories(const std::pair<int, double>& p1, const std::pair<int, double>& p2)
	{
		return p1.second > p2.second;
	}


public:
	//move to CPP later
	void DeleteFromList(std::vector<ObjParticles*>& delete_me, ObjParticles* pElement)
	{
		for(unsigned int k = 0; k < delete_me.size(); k++)
		{
			if(delete_me.at(k) == pElement)
			{
				delete_me.erase(delete_me.begin()+k);
				return;
			}
		}
	}

	void DeleteTheRest(std::vector<ObjParticles*>& delete_me)
	{
		for(unsigned int k = 0; k < delete_me.size(); k++)
		{
			delete delete_me.at(k);
		}

		delete_me.clear();
	}
};



} /* namespace PlannerHNS */

#endif /* BEHAVIORPREDICTION_H_ */
