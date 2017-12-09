/*
 * TrajectoryCostsOnGPU.cpp
 *
 * GPU accelerated version of TrajectoryCosts.cpp
 */

#include "TrajectoryCostsOnGPU.h"
#include "TrajectoryCostsOnGPU_Kernel.h"
#include "GPUCalculationHelper.h"
#include "MatrixOperations.h"
#include <thrust/host_vector.h>
#include <thrust/execution_policy.h>

#include <chrono>
#include <iostream>
#include <iomanip>
#include <float.h>

namespace PlannerHNS
{


TrajectoryCostsOnGPU::TrajectoryCostsOnGPU()
{
	m_PrevCostIndex = -1;
}

TrajectoryCostsOnGPU::~TrajectoryCostsOnGPU()
{
}

TrajectoryCost TrajectoryCostsOnGPU::DoOneStep(const vector<vector<vector<WayPoint> > >& rollOuts,
		const vector<vector<WayPoint> >& totalPaths, const WayPoint& currState, const int& currIndex,
		const int& currLaneIndex,
		const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState,
		const std::vector<PlannerHNS::DetectedObject>& obj_list)
{
	auto start  = std::chrono::system_clock::now();
	TrajectoryCost bestTrajectory;
	bestTrajectory.bBlocked = true;
	bestTrajectory.closest_obj_distance = params.horizonDistance;
	bestTrajectory.closest_obj_velocity = 0;
	bestTrajectory.index = -1;

	if(!ValidateRollOutsInput(rollOuts) || rollOuts.size() != totalPaths.size()) return bestTrajectory;

	if(m_PrevCostIndex == -1)
		m_PrevCostIndex = params.rollOutNumber/2;

	m_TrajectoryCosts.clear();

	for(unsigned int il = 0; il < rollOuts.size(); il++)
	{
		if(rollOuts.at(il).size()>0 && rollOuts.at(il).at(0).size()>0)
		{
			vector<TrajectoryCost> costs = CalculatePriorityAndLaneChangeCosts(rollOuts.at(il), il, params);
			m_TrajectoryCosts.insert(m_TrajectoryCosts.end(), costs.begin(), costs.end());
		}
	}

	// (Re)Allocate and copy arrays used to calculate costs on GPU
	InitDevVectors(m_TrajectoryCosts, m_TrajectoryCostsDevVectors);

	// Calcurate transition cost
	CalculateTransitionCostsOnGPU(m_TrajectoryCostsDevVectors, currIndex, params);

	vector<WayPoint> contourPoints;
	WayPoint p;
	for(unsigned int io=0; io<obj_list.size(); io++)
	{
		for(unsigned int icon=0; icon < obj_list.at(io).contour.size(); icon++)
		{
			p.pos = obj_list.at(io).contour.at(icon);
			p.v = obj_list.at(io).center.v;
			contourPoints.push_back(p);
		}
	}

	CalculateLateralAndLongitudinalCostsOnGPU(m_TrajectoryCostsDevVectors, rollOuts, totalPaths, currState, contourPoints, params, carInfo, vehicleState);


	NormalizeCostsOnGPU(m_TrajectoryCostsDevVectors);

	int smallestIndex = -1;
	double smallestCost = 9999999999;
	double smallestDistance = 9999999999;
	double velo_of_next = 0;

	for(unsigned int ic = 0; ic < m_TrajectoryCosts.size(); ic++)
	{
	    //if(!m_TrajectoryCosts.at(ic).bBlocked && m_TrajectoryCosts.at(ic).cost < smallestCost)
	    if(!m_TrajectoryCostsDevVectors.bBlocked_dev_vector[ic] && m_TrajectoryCostsDevVectors.cost_dev_vector[ic] < smallestCost)
		{
		    //smallestCost = m_TrajectoryCosts.at(ic).cost;
		    smallestCost = m_TrajectoryCostsDevVectors.cost_dev_vector[ic];
		    smallestIndex = ic;
		}

	    //if(m_TrajectoryCosts.at(ic).closest_obj_distance < smallestDistance)
	    if(m_TrajectoryCostsDevVectors.closest_obj_distance_dev_vector[ic] < smallestDistance)
		{
		    //smallestDistance = m_TrajectoryCosts.at(ic).closest_obj_distance;
		    smallestDistance = m_TrajectoryCostsDevVectors.closest_obj_distance_dev_vector[ic];
		    //velo_of_next = m_TrajectoryCosts.at(ic).closest_obj_velocity;
		    velo_of_next = m_TrajectoryCostsDevVectors.closest_obj_velocity_dev_vector[ic];
		}
	}

	//All is blocked !
	if(smallestIndex == -1 && m_PrevCostIndex < (int)m_TrajectoryCosts.size())
	{
		bestTrajectory.bBlocked = true;
		bestTrajectory.lane_index = currLaneIndex;
		bestTrajectory.index = currIndex;
		bestTrajectory.closest_obj_distance = smallestDistance;
		bestTrajectory.closest_obj_velocity = velo_of_next;
		//bestTrajectory.index = smallestIndex;
	}
	else if(smallestIndex >= 0)
	{
	    bestTrajectory = m_TrajectoryCosts.at(smallestIndex);
	    bestTrajectory.index = smallestIndex;
	    bestTrajectory.relative_index = m_TrajectoryCostsDevVectors.relative_index_dev_vector[smallestIndex];
	    bestTrajectory.closest_obj_velocity = m_TrajectoryCostsDevVectors.closest_obj_velocity_dev_vector[smallestIndex];
	    bestTrajectory.distance_from_center = m_TrajectoryCostsDevVectors.distance_from_center_dev_vector[smallestIndex];
	    bestTrajectory.priority_cost = m_TrajectoryCostsDevVectors.priority_cost_dev_vector[smallestIndex];
	    bestTrajectory.transition_cost = m_TrajectoryCostsDevVectors.transition_cost_dev_vector[smallestIndex];
	    bestTrajectory.closest_obj_cost = m_TrajectoryCostsDevVectors.closest_obj_cost_dev_vector[smallestIndex];
	    bestTrajectory.cost = m_TrajectoryCostsDevVectors.cost_dev_vector[smallestIndex];
	    bestTrajectory.closest_obj_distance = m_TrajectoryCostsDevVectors.closest_obj_distance_dev_vector[smallestIndex];
	    bestTrajectory.lane_index = m_TrajectoryCostsDevVectors.lane_index_dev_vector[smallestIndex];
	    bestTrajectory.lane_change_cost = m_TrajectoryCostsDevVectors.lane_change_cost_dev_vector[smallestIndex];
	    bestTrajectory.lateral_cost = m_TrajectoryCostsDevVectors.lateral_cost_dev_vector[smallestIndex];
	    bestTrajectory.longitudinal_cost = m_TrajectoryCostsDevVectors.longitudinal_cost_dev_vector[smallestIndex];
	    bestTrajectory.bBlocked = m_TrajectoryCostsDevVectors.bBlocked_dev_vector[smallestIndex];
	    //bestTrajectory.index = smallestIndex;
	}

//	cout << "smallestI: " <<  smallestIndex << ", C_Size: " << m_TrajectoryCosts.size()
//			<< ", LaneI: " << bestTrajectory.lane_index << "TrajI: " << bestTrajectory.index
//			<< ", prevSmalI: " << m_PrevCostIndex << ", distance: " << bestTrajectory.closest_obj_distance
//			<< ", Blocked: " << bestTrajectory.bBlocked
//			<< endl;

	m_PrevCostIndex = smallestIndex;

        auto end = std::chrono::system_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::cerr << "Do one step: " << elapsed << "[us]" << std::endl;

	return bestTrajectory;
}


void TrajectoryCostsOnGPU::CalculateLateralAndLongitudinalCosts(vector<TrajectoryCost>& trajectoryCosts,
		const vector<vector<vector<WayPoint> > >& rollOuts, const vector<vector<WayPoint> >& totalPaths,
		const WayPoint& currState, const vector<WayPoint>& contourPoints, const PlanningParams& params,
		const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState)
{
	double critical_lateral_distance =  carInfo.width/2.0 + params.horizontalSafetyDistancel;
	double critical_long_front_distance =  carInfo.wheel_base/2.0 + carInfo.length/2.0 + params.verticalSafetyDistance;
	double critical_long_back_distance =  carInfo.length/2.0 + params.verticalSafetyDistance - carInfo.wheel_base/2.0;
	int iCostIndex = 0;
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);
	PlannerHNS::Mat3 invRotationMat(currState.pos.a-M_PI_2);
	PlannerHNS::Mat3 invTranslationMat(currState.pos.x, currState.pos.y);

	double corner_slide_distance = critical_lateral_distance/2.0;
	double ratio_to_angle = corner_slide_distance/carInfo.max_steer_angle;
	double slide_distance = vehicleState.steer * ratio_to_angle;

	GPSPoint bottom_left(-critical_lateral_distance ,-critical_long_back_distance,  currState.pos.z, 0);
	GPSPoint bottom_right(critical_lateral_distance, -critical_long_back_distance,  currState.pos.z, 0);

	GPSPoint top_right_car(critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0,  currState.pos.z, 0);
	GPSPoint top_left_car(-critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0, currState.pos.z, 0);

	GPSPoint top_right(critical_lateral_distance - slide_distance, critical_long_front_distance,  currState.pos.z, 0);
	GPSPoint top_left(-critical_lateral_distance - slide_distance , critical_long_front_distance, currState.pos.z, 0);

	bottom_left = invRotationMat*bottom_left;
	bottom_left = invTranslationMat*bottom_left;

	top_right = invRotationMat*top_right;
	top_right = invTranslationMat*top_right;

	bottom_right = invRotationMat*bottom_right;
	bottom_right = invTranslationMat*bottom_right;

	top_left = invRotationMat*top_left;
	top_left = invTranslationMat*top_left;

	top_right_car = invRotationMat*top_right_car;
	top_right_car = invTranslationMat*top_right_car;

	top_left_car = invRotationMat*top_left_car;
	top_left_car = invTranslationMat*top_left_car;

//	m_SafetyBox.clear();
//	m_SafetyBox.push_back(bottom_left);
//	m_SafetyBox.push_back(bottom_right);
//	m_SafetyBox.push_back(top_right);
//	m_SafetyBox.push_back(top_left);

	m_SafetyBorder.points.clear();
	m_SafetyBorder.points.push_back(bottom_left) ;
	m_SafetyBorder.points.push_back(bottom_right) ;
	m_SafetyBorder.points.push_back(top_right_car) ;
	m_SafetyBorder.points.push_back(top_right) ;
	m_SafetyBorder.points.push_back(top_left) ;
	m_SafetyBorder.points.push_back(top_left_car) ;

	for(unsigned int il=0; il < rollOuts.size(); il++)
	{
		if(rollOuts.at(il).size() > 0 && rollOuts.at(il).at(0).size()>0)
		{
			RelativeInfo car_info;
			PlanningHelpers::GetRelativeInfo(totalPaths.at(il), currState, car_info);

			for(unsigned int it=0; it< rollOuts.at(il).size(); it++)
			{
				for(unsigned int icon = 0; icon < contourPoints.size(); icon++)
				{
					RelativeInfo obj_info;
					PlanningHelpers::GetRelativeInfo(totalPaths.at(il), contourPoints.at(icon), obj_info);
					double longitudinalDist = PlanningHelpers::GetExactDistanceOnTrajectory(totalPaths.at(il), car_info, obj_info);
					// std::cerr << "longitudinalDist: " << longitudinalDist << std::endl;
					if(obj_info.iFront == 0 && longitudinalDist > 0)
						longitudinalDist = -longitudinalDist;

					double close_in_percentage = 1;
//					close_in_percentage = ((longitudinalDist- critical_long_front_distance)/params.rollInMargin)*4.0;
//
//					if(close_in_percentage <= 0 || close_in_percentage > 1) close_in_percentage = 1;

					double distance_from_center = trajectoryCosts.at(iCostIndex).distance_from_center;

					if(close_in_percentage < 1)
						distance_from_center = distance_from_center - distance_from_center * (1.0-close_in_percentage);

					double lateralDist = fabs(obj_info.perp_distance - distance_from_center);

					if(longitudinalDist < -carInfo.length || lateralDist > 6)
					{
						continue;
					}

					longitudinalDist = longitudinalDist - critical_long_front_distance;

					if(m_SafetyBorder.PointInsidePolygon(m_SafetyBorder, contourPoints.at(icon).pos) == true)
						trajectoryCosts.at(iCostIndex).bBlocked = true;

					if(lateralDist <= critical_lateral_distance
							&& longitudinalDist >= -carInfo.length/1.5
							&& longitudinalDist < params.minFollowingDistance)
						trajectoryCosts.at(iCostIndex).bBlocked = true;


					trajectoryCosts.at(iCostIndex).lateral_cost += 1.0/lateralDist;
					trajectoryCosts.at(iCostIndex).longitudinal_cost += 1.0/fabs(longitudinalDist);


					if(longitudinalDist >= -critical_long_front_distance && longitudinalDist < trajectoryCosts.at(iCostIndex).closest_obj_distance)
					{
						trajectoryCosts.at(iCostIndex).closest_obj_distance = longitudinalDist;
						trajectoryCosts.at(iCostIndex).closest_obj_velocity = contourPoints.at(icon).v;
					}
				}

				iCostIndex++;
			}
		}
	}
}


void TrajectoryCostsOnGPU::CalculateLateralAndLongitudinalCostsOnGPU(struct TrajectoryCostsDevVectors& trajectoryCostsDev,
		const vector<vector<vector<WayPoint> > >& rollOuts, const vector<vector<WayPoint> >& totalPaths,
		const WayPoint& currState, const vector<WayPoint>& contourPoints, const PlanningParams& params,
		const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState)
{
    	double critical_lateral_distance =  carInfo.width/2.0 + params.horizontalSafetyDistancel;
	double critical_long_front_distance =  carInfo.wheel_base/2.0 + carInfo.length/2.0 + params.verticalSafetyDistance;
	double critical_long_back_distance =  carInfo.length/2.0 + params.verticalSafetyDistance - carInfo.wheel_base/2.0;
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);
	PlannerHNS::Mat3 invRotationMat(currState.pos.a-M_PI_2);
	PlannerHNS::Mat3 invTranslationMat(currState.pos.x, currState.pos.y);

	double corner_slide_distance = critical_lateral_distance/2.0;
	double ratio_to_angle = corner_slide_distance/carInfo.max_steer_angle;
	double slide_distance = vehicleState.steer * ratio_to_angle;

	GPSPoint bottom_left(-critical_lateral_distance ,-critical_long_back_distance,  currState.pos.z, 0);
	GPSPoint bottom_right(critical_lateral_distance, -critical_long_back_distance,  currState.pos.z, 0);

	GPSPoint top_right_car(critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0,  currState.pos.z, 0);
	GPSPoint top_left_car(-critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0, currState.pos.z, 0);

	GPSPoint top_right(critical_lateral_distance - slide_distance, critical_long_front_distance,  currState.pos.z, 0);
	GPSPoint top_left(-critical_lateral_distance - slide_distance , critical_long_front_distance, currState.pos.z, 0);

	bottom_left = invRotationMat*bottom_left;
	bottom_left = invTranslationMat*bottom_left;

	top_right = invRotationMat*top_right;
	top_right = invTranslationMat*top_right;

	bottom_right = invRotationMat*bottom_right;
	bottom_right = invTranslationMat*bottom_right;

	top_left = invRotationMat*top_left;
	top_left = invTranslationMat*top_left;

	top_right_car = invRotationMat*top_right_car;
	top_right_car = invTranslationMat*top_right_car;

	top_left_car = invRotationMat*top_left_car;
	top_left_car = invTranslationMat*top_left_car;

	m_SafetyBorder.points.clear();
	m_SafetyBorder.points.push_back(bottom_left) ;
	m_SafetyBorder.points.push_back(bottom_right) ;
	m_SafetyBorder.points.push_back(top_right_car) ;
	m_SafetyBorder.points.push_back(top_right) ;
	m_SafetyBorder.points.push_back(top_left) ;
	m_SafetyBorder.points.push_back(top_left_car) ;

	// Prepare to copy totalPaths to GPU
	thrust::device_vector<int> totalPaths_each_length(totalPaths.size());
	thrust::fill(thrust::device, totalPaths_each_length.begin(), totalPaths_each_length.end(), 0);
	for (unsigned int i = 0; i < totalPaths.size(); i++) {
	    totalPaths_each_length[i] = totalPaths.at(i).size();
	}

	// Copy necessary data of totalPaths to GPU
	thrust::host_vector<ForGPU::WayPoint> totalPaths_host;
	for (const auto path: totalPaths) {
	    for (const auto point: path) {
		ForGPU::WayPoint tmp;
		tmp = point;
		totalPaths_host.push_back(tmp);	// Once copy to host vector
	    }
	}
	thrust::device_vector<ForGPU::WayPoint> totalPaths_dev = totalPaths_host; // Then copy to device_vector via host_vector

	// Copy necessary data of currState to GPU
	std::vector<WayPoint> std_currState_vector(1, currState); // Create std::vector which have only one element to copy
	thrust::host_vector<ForGPU::WayPoint> currState_host(std_currState_vector);  // Once copy to host vector
	thrust::device_vector<ForGPU::WayPoint> currState_dev = currState_host;  // Then copy to device_vector via host_vector

	// Copy necessary data of contourPoints to GPU
	thrust::host_vector<ForGPU::WayPoint>  contourPoints_host;
	for (const auto contour: contourPoints) {
	    ForGPU::WayPoint tmp;
	    tmp = contour;
	    contourPoints_host.push_back(tmp);  // Once copy to host_vector
	}
	thrust::device_vector<ForGPU::WayPoint>  contourPoints_dev = contourPoints_host; // Then copy to device_vector via host_vector

	// Copy necessary data of m_SafetyBorder
	thrust::host_vector<ForGPU::GPSPoint> safetyBorder_host;
	for (const auto point: m_SafetyBorder.points) {
	    ForGPU::GPSPoint tmp;
	    tmp = point;
	    safetyBorder_host.push_back(tmp); // Once copy to host_vector
	}
	thrust::device_vector<ForGPU::GPSPoint> safetyBorder_dev = safetyBorder_host; // Then copy to device_vector via host_vector

	// Define GPU kernel grid shape
	int grid_total_x_dim = 0;
	thrust::device_vector<int> rollOuts_each_length(rollOuts.size());
	thrust::fill(thrust::device, rollOuts_each_length.begin(), rollOuts_each_length.end(), 0);
	for (int il=0; il < rollOuts.size(); il++) {
	    grid_total_x_dim = (grid_total_x_dim <= rollOuts.at(il).size()) ? rollOuts.at(il).size() : grid_total_x_dim;
	    if (rollOuts.at(il).size() > 0 && rollOuts.at(il).at(0).size() > 0) {
		rollOuts_each_length[il] = rollOuts.at(il).size();
	    }
	}

	int grid_total_y_dim = rollOuts.size();

	dim3 block_dim(kBlockSize, kBlockSize, 1);

	dim3 grid_dim((grid_total_x_dim + block_dim.x - 1) / block_dim.x,
		      (grid_total_y_dim + block_dim.y - 1) / block_dim.y,
		      1);

	dim3 valid_id_limit(grid_total_x_dim, grid_total_y_dim, 1);

	// std::cerr << "**************************************** kernel Launch" << std::endl;
	// std::cerr << "grid dimension: (" << grid_dim.x << ", " << grid_dim.y << ", " << grid_dim.z << ")" << std::endl
	// 	  << "block dimension: (" << block_dim.x << ", " << block_dim.y << ", " << block_dim.z << ")" << std::endl
	// 	  << "valid_id_limit: (" << valid_id_limit.x << ", " << valid_id_limit.y << ", " << valid_id_limit.z << ")" << std::endl;


	// Allocate global memory on GPU for used as temporary buffer
	int num_of_buffer_element = grid_total_x_dim * grid_total_y_dim * contourPoints.size();

	// Allocate and initialize temporary buffers that are used in GPU kernel calculation
	thrust::device_vector<bool> bBlocked_tmp_buffer(num_of_buffer_element);
	thrust::fill(thrust::device, bBlocked_tmp_buffer.begin(), bBlocked_tmp_buffer.end(), false);

	thrust::device_vector<double> lateral_cost_tmp_buffer(num_of_buffer_element);
	thrust::fill(thrust::device, lateral_cost_tmp_buffer.begin(), lateral_cost_tmp_buffer.end(), 0.f);

	thrust::device_vector<double> longitudinal_cost_tmp_buffer(num_of_buffer_element);
	thrust::fill(thrust::device, longitudinal_cost_tmp_buffer.begin(), longitudinal_cost_tmp_buffer.end(), 0.f);

	thrust::device_vector<double> closest_obj_distance_tmp_buffer(num_of_buffer_element);
	thrust::fill(thrust::device, closest_obj_distance_tmp_buffer.begin(), closest_obj_distance_tmp_buffer.end(), DBL_MAX);

	// Because no value are updated when contourPoints has no contents,
	// Return if contourPoints is empty so that unnecessary GPU kernel launch can be avoided
	if (contourPoints.size() == 0) return;

	// Launch GPU kernel
	CalculateLateralAndLongitudinalCosts_kernel <<<grid_dim, block_dim>>> ((bool*)thrust::raw_pointer_cast(trajectoryCostsDev.bBlocked_dev_vector.data()),
									       (bool*)thrust::raw_pointer_cast(bBlocked_tmp_buffer.data()),
									       (double*)thrust::raw_pointer_cast(trajectoryCostsDev.lateral_cost_dev_vector.data()),
									       (double*)thrust::raw_pointer_cast(lateral_cost_tmp_buffer.data()),
									       (double*)thrust::raw_pointer_cast(trajectoryCostsDev.longitudinal_cost_dev_vector.data()),
									       (double*)thrust::raw_pointer_cast(longitudinal_cost_tmp_buffer.data()),
									       (double*)thrust::raw_pointer_cast(trajectoryCostsDev.closest_obj_distance_dev_vector.data()),
									       (double*)thrust::raw_pointer_cast(closest_obj_distance_tmp_buffer.data()),
									       (double*)thrust::raw_pointer_cast(trajectoryCostsDev.closest_obj_velocity_dev_vector.data()),
									       valid_id_limit,
									       (const ForGPU::WayPoint*)thrust::raw_pointer_cast(totalPaths_dev.data()),
									       (const int*)thrust::raw_pointer_cast(totalPaths_each_length.data()),
									       (const ForGPU::WayPoint*)thrust::raw_pointer_cast(currState_dev.data()),
									       (const ForGPU::WayPoint*)thrust::raw_pointer_cast(contourPoints_dev.data()),
									       contourPoints.size(),
									       (const double*)thrust::raw_pointer_cast(trajectoryCostsDev.distance_from_center_dev_vector.data()),
									       critical_long_front_distance,
									       critical_lateral_distance,
									       (const ForGPU::GPSPoint*)thrust::raw_pointer_cast(safetyBorder_dev.data()),
									       m_SafetyBorder.points.size(),
									       carInfo.length,
									       params.minFollowingDistance,
									       (const int*)thrust::raw_pointer_cast(rollOuts_each_length.data())
									       );
}


void TrajectoryCostsOnGPU::NormalizeCosts(vector<TrajectoryCost>& trajectoryCosts)
{
	//Normalize costs
	double totalPriorities = 0;
	double totalChange = 0;
	double totalLateralCosts = 0;
	double totalLongitudinalCosts = 0;
	double transitionCosts = 0;

	for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		totalPriorities += trajectoryCosts.at(ic).priority_cost;
		transitionCosts += trajectoryCosts.at(ic).transition_cost;
	}

	for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		totalChange += trajectoryCosts.at(ic).lane_change_cost;
		totalLateralCosts += trajectoryCosts.at(ic).lateral_cost;
		totalLongitudinalCosts += trajectoryCosts.at(ic).longitudinal_cost;
	}

	for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		if(totalPriorities != 0)
			trajectoryCosts.at(ic).priority_cost = trajectoryCosts.at(ic).priority_cost / totalPriorities;
		else
			trajectoryCosts.at(ic).priority_cost = 0;

		if(transitionCosts != 0)
			trajectoryCosts.at(ic).transition_cost = trajectoryCosts.at(ic).transition_cost / transitionCosts;
		else
			trajectoryCosts.at(ic).transition_cost = 0;

		if(totalChange != 0)
			trajectoryCosts.at(ic).lane_change_cost = trajectoryCosts.at(ic).lane_change_cost / totalChange;
		else
			trajectoryCosts.at(ic).lane_change_cost = 0;

		if(totalLateralCosts != 0)
			trajectoryCosts.at(ic).lateral_cost = trajectoryCosts.at(ic).lateral_cost / totalLateralCosts;
		else
			trajectoryCosts.at(ic).lateral_cost = 0;

		if(totalLongitudinalCosts != 0)
			trajectoryCosts.at(ic).longitudinal_cost = trajectoryCosts.at(ic).longitudinal_cost / totalLongitudinalCosts;
		else
			trajectoryCosts.at(ic).longitudinal_cost = 0;

		trajectoryCosts.at(ic).cost = (
				trajectoryCosts.at(ic).priority_cost +
				trajectoryCosts.at(ic).lane_change_cost +
				trajectoryCosts.at(ic).lateral_cost +
				trajectoryCosts.at(ic).longitudinal_cost +
				1.5*trajectoryCosts.at(ic).transition_cost) / 5.0;
	}
}


void TrajectoryCostsOnGPU::NormalizeCostsOnGPU(struct TrajectoryCostsDevVectors& trajectoryCostsDev)
{
	//Normalize costs
	double totalPriorities = 0;
	double totalChange = 0;
	double totalLateralCosts = 0;
	double totalLongitudinalCosts = 0;
	double transitionCosts = 0;

	// Confirm that all GPU calculations executed before are completed
	cudaError_t synch_status = cudaDeviceSynchronize();
	if (synch_status != cudaSuccess) {
	    // This is fatal status because one CUDA operations issued until here was failed.
	    // Exit program with EXIT_FAILURE status.
	    std::cerr << "*** CUDA internal error is occured at line " << __LINE__ << " at " << __FILE__ << std::endl
		      << "*** The error was: " << cudaGetErrorString(synch_status) << std::endl
		      << "*** Program will be terminated." << std::endl;
	    exit(EXIT_FAILURE);
	}

	// Calculate total sum of elements in each buffers
	totalPriorities = thrust::reduce(trajectoryCostsDev.priority_cost_dev_vector.begin(),
					 trajectoryCostsDev.priority_cost_dev_vector.end());

	transitionCosts = thrust::reduce(trajectoryCostsDev.transition_cost_dev_vector.begin(),
					 trajectoryCostsDev.transition_cost_dev_vector.end());

	totalChange = thrust::reduce(trajectoryCostsDev.lane_change_cost_dev_vector.begin(),
				     trajectoryCostsDev.lane_change_cost_dev_vector.end());

	totalLateralCosts = thrust::reduce(trajectoryCostsDev.lateral_cost_dev_vector.begin(),
					   trajectoryCostsDev.lateral_cost_dev_vector.end());

	totalLongitudinalCosts = thrust::reduce(trajectoryCostsDev.longitudinal_cost_dev_vector.begin(),
						trajectoryCostsDev.longitudinal_cost_dev_vector.end());

	// Calculate normalized cost based on values above
	thrust::for_each(thrust::make_zip_iterator(thrust::make_tuple(trajectoryCostsDev.priority_cost_dev_vector.begin(),
								      trajectoryCostsDev.transition_cost_dev_vector.begin(),
								      trajectoryCostsDev.lane_change_cost_dev_vector.begin(),
								      trajectoryCostsDev.lateral_cost_dev_vector.begin(),
								      trajectoryCostsDev.longitudinal_cost_dev_vector.begin(),
								      trajectoryCostsDev.cost_dev_vector.begin()
								      )),
			 thrust::make_zip_iterator(thrust::make_tuple(trajectoryCostsDev.priority_cost_dev_vector.end(),
								      trajectoryCostsDev.transition_cost_dev_vector.end(),
								      trajectoryCostsDev.lane_change_cost_dev_vector.end(),
								      trajectoryCostsDev.lateral_cost_dev_vector.end(),
								      trajectoryCostsDev.longitudinal_cost_dev_vector.end(),
								      trajectoryCostsDev.cost_dev_vector.end()
								      )),
			 NormalizeCostsFunctor(totalPriorities,
					       totalChange,
					       totalLateralCosts,
					       totalLongitudinalCosts,
					       transitionCosts));
}

vector<TrajectoryCost> TrajectoryCostsOnGPU::CalculatePriorityAndLaneChangeCosts(const vector<vector<WayPoint> >& laneRollOuts,
		const int& lane_index, const PlanningParams& params)
{
	vector<TrajectoryCost> costs;
	TrajectoryCost tc;
	int centralIndex = params.rollOutNumber/2;

	tc.lane_index = lane_index;
	for(unsigned int it=0; it< laneRollOuts.size(); it++)
	{
		tc.index = it;
		tc.relative_index = it - centralIndex;
		tc.distance_from_center = params.rollOutDensity*tc.relative_index;
		tc.priority_cost = fabs(tc.distance_from_center);
		tc.closest_obj_distance = params.horizonDistance;
		tc.lane_change_cost = laneRollOuts.at(it).at(0).laneChangeCost;

//		if(laneRollOuts.at(it).at(0).bDir == FORWARD_LEFT_DIR || laneRollOuts.at(it).at(0).bDir == FORWARD_RIGHT_DIR)
//			tc.lane_change_cost = 1;
//		else if(laneRollOuts.at(it).at(0).bDir == BACKWARD_DIR || laneRollOuts.at(it).at(0).bDir == BACKWARD_RIGHT_DIR || laneRollOuts.at(it).at(0).bDir == BACKWARD_LEFT_DIR)
//			tc.lane_change_cost = 2;
//		else
//			tc.lane_change_cost = 0;

		costs.push_back(tc);
	}

	return costs;
}

void TrajectoryCostsOnGPU::CalculateTransitionCosts(vector<TrajectoryCost>& trajectoryCosts, const int& currTrajectoryIndex, const PlanningParams& params)
{
	for(int ic = 0; ic< trajectoryCosts.size(); ic++)
	{
		trajectoryCosts.at(ic).transition_cost = fabs(params.rollOutDensity * (ic - currTrajectoryIndex));
	}
}


void TrajectoryCostsOnGPU::CalculateTransitionCostsOnGPU(struct TrajectoryCostsDevVectors& trajectoryCostsDev, const int& currTrajectoryIndex, const PlanningParams& params)
{
	// Execute transition cost calculation in parallel
	thrust::counting_iterator<int> begin_iterator(0);
	thrust::transform(begin_iterator,
			  begin_iterator + trajectoryCostsDev.transition_cost_dev_vector.size(),
			  trajectoryCostsDev.transition_cost_dev_vector.begin(),
			  TransitionCostCalculateFunctor(params.rollOutDensity, currTrajectoryIndex));
}

/**
 * @brief Validate input, each trajectory must have at least 1 way point
 * @param rollOuts
 * @return true if data isvalid for cost calculation
 */
bool TrajectoryCostsOnGPU::ValidateRollOutsInput(const vector<vector<vector<WayPoint> > >& rollOuts)
{
	if(rollOuts.size()==0)
		return false;

	for(unsigned int il = 0; il < rollOuts.size(); il++)
	{
		if(rollOuts.at(il).size() == 0)
			return false;
	}

	return true;
}


void TrajectoryCostsOnGPU::InitDevVectors(const vector<TrajectoryCost>& trajectoryCosts, struct TrajectoryCostsDevVectors& devVectors)
{
    // Re-allocate device vectors
    int new_size = trajectoryCosts.size();
    devVectors.relative_index_dev_vector.resize(new_size);
    devVectors.closest_obj_velocity_dev_vector.resize(new_size);
    devVectors.distance_from_center_dev_vector.resize(new_size);
    devVectors.priority_cost_dev_vector.resize(new_size);
    devVectors.transition_cost_dev_vector.resize(new_size);
    devVectors.closest_obj_cost_dev_vector.resize(new_size);
    devVectors.cost_dev_vector.resize(new_size);
    devVectors.closest_obj_distance_dev_vector.resize(new_size);
    devVectors.lane_index_dev_vector.resize(new_size);
    devVectors.lane_change_cost_dev_vector.resize(new_size);
    devVectors.lateral_cost_dev_vector.resize(new_size);
    devVectors.longitudinal_cost_dev_vector.resize(new_size);
    devVectors.bBlocked_dev_vector.resize(new_size);

    // Prepare the thurst::host_vector for efficient copy from host to device
    thrust::host_vector<int> relative_index_host_vector;
    thrust::host_vector<double> closest_obj_velocity_host_vector;
    thrust::host_vector<double> distance_from_center_host_vector;
    thrust::host_vector<double> priority_cost_host_vector;
    thrust::host_vector<double> transition_cost_host_vector;
    thrust::host_vector<double> closest_obj_cost_host_vector;
    thrust::host_vector<double> cost_host_vector;
    thrust::host_vector<double> closest_obj_distance_host_vector;
    thrust::host_vector<int> lane_index_host_vector;
    thrust::host_vector<double> lane_change_cost_host_vector;
    thrust::host_vector<double> lateral_cost_host_vector;
    thrust::host_vector<double> longitudinal_cost_host_vector;
    thrust::host_vector<bool> bBlocked_host_vector;

    // Copy current data to temporary host memory
    for (int index = 0; index < trajectoryCosts.size(); index++){
	TrajectoryCost cost = trajectoryCosts.at(index);

	relative_index_host_vector.push_back(cost.relative_index);
	closest_obj_velocity_host_vector.push_back(cost.closest_obj_velocity);
	distance_from_center_host_vector.push_back(cost.distance_from_center);
	priority_cost_host_vector.push_back(cost.priority_cost);
	transition_cost_host_vector.push_back(cost.transition_cost);
	closest_obj_cost_host_vector.push_back(cost.closest_obj_cost);
	cost_host_vector.push_back(cost.cost);
	closest_obj_distance_host_vector.push_back(cost.closest_obj_distance);
	lane_index_host_vector.push_back(cost.lane_index);
	lane_change_cost_host_vector.push_back(cost.lane_change_cost);
	lateral_cost_host_vector.push_back(cost.lateral_cost);
	longitudinal_cost_host_vector.push_back(cost.longitudinal_cost);
	bBlocked_host_vector.push_back(cost.bBlocked);
    }

    // Copy data to GPU
    devVectors.relative_index_dev_vector = relative_index_host_vector;
    devVectors.closest_obj_velocity_dev_vector = closest_obj_velocity_host_vector;
    devVectors.distance_from_center_dev_vector = distance_from_center_host_vector;
    devVectors.priority_cost_dev_vector = priority_cost_host_vector;
    devVectors.transition_cost_dev_vector = transition_cost_host_vector;
    devVectors.closest_obj_cost_dev_vector = closest_obj_cost_host_vector;
    devVectors.cost_dev_vector = cost_host_vector;
    devVectors.closest_obj_distance_dev_vector = closest_obj_distance_host_vector;
    devVectors.lane_index_dev_vector = lane_index_host_vector;
    devVectors.lane_change_cost_dev_vector = lane_change_cost_host_vector;
    devVectors.lateral_cost_dev_vector = lateral_cost_host_vector;
    devVectors.longitudinal_cost_dev_vector = longitudinal_cost_host_vector;
    devVectors.bBlocked_dev_vector = bBlocked_host_vector;
}

}  // namespace PlannerHNS
