/*
 * TrajectoryCostsOnGPU.h
 *
 */

#ifndef TRAJECTORYCOSTSONGPU_H_
#define TRAJECTORYCOSTSONGPU_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>

#include "RoadNetwork.h"
#include "PlannerCommonDef.h"
#include "PlanningHelpers.h"
#include "TrajectoryCostsOnGPU_Kernel.h"

using namespace std;

namespace PlannerHNS
{

/* class TrajectoryCostsOnGPU : private TrajectoryCostsOnGPU_private */
class TrajectoryCostsOnGPU
{
public:
	TrajectoryCostsOnGPU();
	virtual ~TrajectoryCostsOnGPU();

	TrajectoryCost DoOneStep(const vector<vector<vector<WayPoint> > >& rollOuts, const vector<vector<WayPoint> >& totalPaths,
			const WayPoint& currState, const int& currTrajectoryIndex, const int& currLaneIndex, const PlanningParams& params,
			const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState, const std::vector<PlannerHNS::DetectedObject>& obj_list);

public:
	int m_PrevCostIndex;
	vector<TrajectoryCost> m_TrajectoryCosts;
	PlanningParams m_Params;
	PolygonShape m_SafetyBorder;
	//vector<GPSPoint> m_SafetyBox;



private:
	/* Data tranporter structure */
	struct TrajectoryCostsDevVectors
	{
		thrust::device_vector<int> relative_index_dev_vector;
		thrust::device_vector<double> closest_obj_velocity_dev_vector;
		thrust::device_vector<double> distance_from_center_dev_vector;
		thrust::device_vector<double> priority_cost_dev_vector;
		thrust::device_vector<double> transition_cost_dev_vector;
		thrust::device_vector<double> closest_obj_cost_dev_vector;
		thrust::device_vector<double> cost_dev_vector;
		thrust::device_vector<double> closest_obj_distance_dev_vector;
		thrust::device_vector<int> lane_index_dev_vector;
		thrust::device_vector<double> lane_change_cost_dev_vector;
		thrust::device_vector<double> lateral_cost_dev_vector;
		thrust::device_vector<double> longitudinal_cost_dev_vector;
		thrust::device_vector<bool> bBlocked_dev_vector;
	};

	bool ValidateRollOutsInput(const vector<vector<vector<WayPoint> > >& rollOuts);
	vector<TrajectoryCost> CalculatePriorityAndLaneChangeCosts(const vector<vector<WayPoint> >& laneRollOuts, const int& lane_index, const PlanningParams& params);
	void NormalizeCosts(vector<TrajectoryCost>& trajectoryCosts);
	void CalculateLateralAndLongitudinalCosts(vector<TrajectoryCost>& trajectoryCosts, const vector<vector<vector<WayPoint> > >& rollOuts, const vector<vector<WayPoint> >& totalPaths, const WayPoint& currState, const vector<WayPoint>& contourPoints, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState);
	void CalculateLateralAndLongitudinalCostsOnGPU(struct TrajectoryCostsDevVectors& trajectoryCostsDev, const vector<vector<vector<WayPoint> > >& rollOuts, const vector<vector<WayPoint> >& totalPaths, const WayPoint& currState, const vector<WayPoint>& contourPoints, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState);
	void CalculateTransitionCosts(vector<TrajectoryCost>& trajectoryCosts, const int& currTrajectoryIndex, const PlanningParams& params);
	void CalculateTransitionCostsOnGPU(struct TrajectoryCostsDevVectors& trajectoryCostsDev, const int& currTrajectoryIndex, const PlanningParams& params);
	void InitDevVectors(const vector<TrajectoryCost>& trajectoryCosts, struct TrajectoryCostsDevVectors& devVectors);

	struct TrajectoryCostsDevVectors m_TrajectoryCostsDevVectors;

	static const int kBlockSize = 32;
};

}

#endif /* TRAJECTORYCOSTSONGPU_H_ */
