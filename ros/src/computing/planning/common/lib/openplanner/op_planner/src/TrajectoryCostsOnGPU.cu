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

#include <chrono>
#include <iostream>
#include <iomanip>

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
			// TODO: この関数もGPU化だろ。てか、このループ自体展開した方がいいと思う
			vector<TrajectoryCost> costs = CalculatePriorityAndLaneChangeCosts(rollOuts.at(il), il, params);
			// costsをm_TrajectoryCostsの後ろに連結
			m_TrajectoryCosts.insert(m_TrajectoryCosts.end(), costs.begin(), costs.end());
		}
	}

	// (Re)Allocate and copy arrays used to calculate costs on GPU
	InitDevVectors(m_TrajectoryCosts, m_TrajectoryCostsDevVectors);

	// Calcurate transition cost
	CalculateTransitionCosts(m_TrajectoryCosts, currIndex, params);
	CalculateTransitionCostsOnGPU(m_TrajectoryCostsDevVectors, currIndex, params); // この計算あってること確認済

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

#if 0
	for (int i=0; i<m_TrajectoryCosts.size(); i++) {
	    auto cpu = m_TrajectoryCosts.at(i);
	    std::cerr << "-------------------" << std::endl
		      << "index: " << i << std::endl
		      << "bBlocked: " << cpu.bBlocked << ", " << m_TrajectoryCostsDevVectors.bBlocked_dev_vector[i] << std::endl
		      << "lateral_cost: " << cpu.lateral_cost << ", " << m_TrajectoryCostsDevVectors.lateral_cost_dev_vector[i] << std::endl
		      << "longitudinal_cost: " << cpu.longitudinal_cost << ", " << m_TrajectoryCostsDevVectors.longitudinal_cost_dev_vector[i] << std::endl
		      << "closest_obj_distance: " << cpu.closest_obj_distance << ", " << m_TrajectoryCostsDevVectors.closest_obj_distance_dev_vector[i] << std::endl
		      << "closest_obj_velocity: " << cpu.closest_obj_velocity << ", " << m_TrajectoryCostsDevVectors.closest_obj_velocity_dev_vector[i] << std::endl
		      << "-------------------" << std::endl;
	}
#else
	//std::cerr << "params.horizonDistance: " << params.horizonDistance << std::endl;
#endif

	CalculateLateralAndLongitudinalCosts(m_TrajectoryCosts, rollOuts, totalPaths, currState, contourPoints, params, carInfo, vehicleState);
	CalculateLateralAndLongitudinalCostsOnGPU(m_TrajectoryCostsDevVectors, rollOuts, totalPaths, currState, contourPoints, params, carInfo, vehicleState);


#if 0
	for (int i=0; i<m_TrajectoryCosts.size(); i++) {
	    auto cpu = m_TrajectoryCosts.at(i);
	    std::cerr << "-------------------" << std::endl
		      << "index: " << i << std::endl
		      << "bBlocked: " << cpu.bBlocked << ", " << m_TrajectoryCostsDevVectors.bBlocked_dev_vector[i] << std::endl
		      << "lateral_cost: " << cpu.lateral_cost << ", " << m_TrajectoryCostsDevVectors.lateral_cost_dev_vector[i] << std::endl
		      << "longitudinal_cost: " << cpu.longitudinal_cost << ", " << m_TrajectoryCostsDevVectors.longitudinal_cost_dev_vector[i] << std::endl
		      << "closest_obj_distance: " << cpu.closest_obj_distance << ", " << m_TrajectoryCostsDevVectors.closest_obj_distance_dev_vector[i] << std::endl
		      << "closest_obj_velocity: " << cpu.closest_obj_velocity << ", " << m_TrajectoryCostsDevVectors.closest_obj_velocity_dev_vector[i] << std::endl
		      << "-------------------" << std::endl;
	}
#endif

	NormalizeCosts(m_TrajectoryCosts);
	NormalizeCostsOnGPU(m_TrajectoryCostsDevVectors);

#if 0
	std::cerr << "m_TrajectoryCosts.size(): " << m_TrajectoryCosts.size() << std::endl;
	for (int i=0; i<m_TrajectoryCosts.size(); i++) {
	    auto cpu = m_TrajectoryCosts.at(i);
	    std::cerr << "-------------------" << std::endl
		      << "index: " << i << std::endl
		      << "priority_cost: " << cpu.priority_cost << ", " << m_TrajectoryCostsDevVectors.priority_cost_dev_vector[i] << std::endl
		      << "transition_cost: " << cpu.transition_cost << ", " << m_TrajectoryCostsDevVectors.transition_cost_dev_vector[i] << std::endl
		      << "lane_change_cost: " << cpu.lane_change_cost << ", " << m_TrajectoryCostsDevVectors.lane_change_cost_dev_vector[i] << std::endl
		      << "lateral_cost: " << cpu.lateral_cost << ", " << m_TrajectoryCostsDevVectors.lateral_cost_dev_vector[i] << std::endl
		      << "longitudinal_cost: " << cpu.longitudinal_cost << ", " << m_TrajectoryCostsDevVectors.longitudinal_cost_dev_vector[i] << std::endl
		      << "cost: " << cpu.cost << ", " << m_TrajectoryCostsDevVectors.cost_dev_vector[i] << std::endl
		      << "-------------------" << std::endl;
	}
#endif

	int smallestIndex = -1;
	double smallestCost = 9999999999;
	double smallestDistance = 9999999999;
	double velo_of_next = 0;

//	cout << "Trajectory Costs Log : CurrIndex: " << currIndex << " --------------------- " << endl;
	for(unsigned int ic = 0; ic < m_TrajectoryCosts.size(); ic++)
	{
//		cout << m_TrajectoryCosts.at(ic).ToString();
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

//	cout << "Smallest Distance: " <<  smallestDistance << "------------------------------------------------------------- " << endl;

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
	std::cerr << "smallestI: " <<  smallestIndex << ", C_Size: " << m_TrajectoryCosts.size()
		  << ", LaneI: " << bestTrajectory.lane_index << "TrajI: " << bestTrajectory.index
		  << ", prevSmalI: " << m_PrevCostIndex << ", distance: " << bestTrajectory.closest_obj_distance
		  << ", Blocked: " << bestTrajectory.bBlocked
		  << endl;

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
					std::cerr << "longitudinalDist: " << longitudinalDist << std::endl;
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
	int iCostIndex = 0;
	PlannerHNS::Mat3 rotationMat(-currState.pos.a); // ループ内不使用
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y); // ループ内不使用
	PlannerHNS::Mat3 invRotationMat(currState.pos.a-M_PI_2); // ループ内不使用
	PlannerHNS::Mat3 invTranslationMat(currState.pos.x, currState.pos.y); // ループ内不使用

	double corner_slide_distance = critical_lateral_distance/2.0;
	double ratio_to_angle = corner_slide_distance/carInfo.max_steer_angle;
	double slide_distance = vehicleState.steer * ratio_to_angle;

	GPSPoint bottom_left(-critical_lateral_distance ,-critical_long_back_distance,  currState.pos.z, 0); // ループ内不使用
	GPSPoint bottom_right(critical_lateral_distance, -critical_long_back_distance,  currState.pos.z, 0); // ループ内不使用

	GPSPoint top_right_car(critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0,  currState.pos.z, 0); // ループ内不使用
	GPSPoint top_left_car(-critical_lateral_distance, carInfo.wheel_base/3.0 + carInfo.length/3.0, currState.pos.z, 0); // ループ内不使用

	GPSPoint top_right(critical_lateral_distance - slide_distance, critical_long_front_distance,  currState.pos.z, 0); // ループ内不使用
	GPSPoint top_left(-critical_lateral_distance - slide_distance , critical_long_front_distance, currState.pos.z, 0); // ループ内不使用

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
	int num_of_element = 0;
	for (unsigned int i = 0; i < totalPaths.size(); i++) {
	    num_of_element += totalPaths.at(i).size();
	    // totalPaths_each_length[i] = num_of_element;
	    totalPaths_each_length[i] = totalPaths.at(i).size();
	}

	std::cerr << "********************** num_of_element: " << num_of_element << std::endl;

	// Copy necessary data of totalPaths to GPU
	thrust::device_vector<ForGPU::WayPoint> totalPaths_dev(num_of_element);
	int idx = 0;
	for (const auto path: totalPaths) {
	    for (const auto point: path) {
		static_cast<ForGPU::WayPoint>(totalPaths_dev[idx])  = point;
		idx++;
	    }
	}

	std::cerr << "********************** contourPoints.size(): " << contourPoints.size() << std::endl;

	// Copy necessary data of currState to GPU
	thrust::device_vector<ForGPU::WayPoint> currState_dev(1);
	static_cast<ForGPU::WayPoint>(currState_dev[0]) = currState;

	// Copy necessary data of contourPoints to GPU
	thrust::device_vector<ForGPU::WayPoint>  contourPoints_dev(contourPoints.size());
	idx = 0;
	for (const auto contour: contourPoints) {
	    static_cast<ForGPU::WayPoint>(contourPoints_dev[idx]) = contour;
	    idx++;
	}

	std::cerr << "********************** safetyBorder.points.size(): " << m_SafetyBorder.points.size() << std::endl;

	// Copy necessary data of m_SafetyBorder
	thrust::device_vector<ForGPU::GPSPoint> safetyBorder_dev(m_SafetyBorder.points.size());
	idx = 0;
	for (const auto point: m_SafetyBorder.points) {
	    static_cast<ForGPU::GPSPoint>(safetyBorder_dev[idx]) = point;
	    idx++;
	}

	// Define GPU kernel grid shape
	int grid_total_x_dim = 0;
	for (int il=0; il < rollOuts.size(); il++) {
	    grid_total_x_dim += rollOuts.at(il).size();
	}

	int grid_total_y_dim = rollOuts.size();


	dim3 block_dim(kBlockSize, kBlockSize, 1);
	dim3 grid_dim((grid_total_x_dim + block_dim.x - 1) / block_dim.x,
		      (grid_total_y_dim + block_dim.y - 1) / block_dim.y,
		      1);
	dim3 valid_id_limit(grid_total_x_dim, grid_total_y_dim, 1);

	std::cerr << "**************************************** kernel Launch" << std::endl;
	std::cerr << "grid dimension: (" << grid_dim.x << ", " << grid_dim.y << ", " << grid_dim.z << ")" << std::endl
		  << "block dimension: (" << block_dim.x << ", " << block_dim.y << ", " << block_dim.z << ")" << std::endl
		  << "valid_id_limit: (" << valid_id_limit.x << ", " << valid_id_limit.y << ", " << valid_id_limit.z << ")" << std::endl;


	// Allocate global memory on GPU for used as temporary buffer
	int num_of_buffer_element = grid_total_x_dim * grid_total_y_dim * contourPoints.size();
	thrust::device_vector<bool> bBlocked_tmp_buffer(num_of_buffer_element);
	thrust::device_vector<double> lateral_cost_tmp_buffer(num_of_buffer_element);
	thrust::device_vector<double> longitudinal_cost_tmp_buffer(num_of_buffer_element);
	thrust::device_vector<double> closest_obj_distance_tmp_buffer(num_of_buffer_element);

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
									       params.minFollowingDistance
									       );

	cudaError_t code = cudaDeviceSynchronize();
	std::cerr << "Device syc result: " << cudaGetErrorString(code) << std::endl;

#if 0
	for(unsigned int il=0; il < rollOuts.size(); il++)
	{
		if(rollOuts.at(il).size() > 0 && rollOuts.at(il).at(0).size()>0)
		{
			RelativeInfo car_info; // ループ内で書き換わらない
			PlanningHelpers::GetRelativeInfo(totalPaths.at(il), currState, car_info);

			for(unsigned int it=0; it< rollOuts.at(il).size(); it++)
			{
				for(unsigned int icon = 0; icon < contourPoints.size(); icon++)
				{
					RelativeInfo obj_info;
					PlanningHelpers::GetRelativeInfo(totalPaths.at(il), contourPoints.at(icon), obj_info);
					double longitudinalDist = PlanningHelpers::GetExactDistanceOnTrajectory(totalPaths.at(il), car_info, obj_info);
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

					// ****************************************************************************
					// ****************************************************************************
					// bBlockedは下記2つのif中の条件のorの値をバッファ→iconループ内の全バッファのorと元々の値のorが最終結果として挿入される
					// ****************************************************************************
					// ****************************************************************************
					if(m_SafetyBorder.PointInsidePolygon(m_SafetyBorder, contourPoints.at(icon).pos) == true)
						trajectoryCosts.at(iCostIndex).bBlocked = true;

					if(lateralDist <= critical_lateral_distance
							&& longitudinalDist >= -carInfo.length/1.5
							&& longitudinalDist < params.minFollowingDistance)
						trajectoryCosts.at(iCostIndex).bBlocked = true;


					// ****************************************************************************
					// ****************************************************************************
					// ここ2つはひたすら値をバッファして、後でreduction
					// ****************************************************************************
					// ****************************************************************************
					trajectoryCosts.at(iCostIndex).lateral_cost += 1.0/lateralDist;
					trajectoryCosts.at(iCostIndex).longitudinal_cost += 1.0/fabs(longitudinalDist);


					if(longitudinalDist >= -critical_long_front_distance && longitudinalDist < trajectoryCosts.at(iCostIndex).closest_obj_distance)
					{
					    // ****************************************************************************
					    // ****************************************************************************
					    // longitudinalDist >= -critical_long_front_distanceを満たすlogitudinalDistをバッファに格納→iconループ内におけるMinを取り、Minを取る際のiconの値を取得。→closest_obj_velocityにはicon値を元に値を挿入
					    // ****************************************************************************
					    // ****************************************************************************
						trajectoryCosts.at(iCostIndex).closest_obj_distance = longitudinalDist;
						trajectoryCosts.at(iCostIndex).closest_obj_velocity = contourPoints.at(icon).v;
					}
				}

				iCostIndex++;
			}
		}
	}
#endif
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

	// for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	// {
	// 	totalPriorities += trajectoryCosts.at(ic).priority_cost;
	// 	transitionCosts += trajectoryCosts.at(ic).transition_cost;
	// }
	totalPriorities = thrust::reduce(trajectoryCostsDev.priority_cost_dev_vector.begin(),
					 trajectoryCostsDev.priority_cost_dev_vector.end());

	transitionCosts = thrust::reduce(trajectoryCostsDev.transition_cost_dev_vector.begin(),
					 trajectoryCostsDev.transition_cost_dev_vector.end());

	// for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	// {
	// 	totalChange += trajectoryCosts.at(ic).lane_change_cost;
	// 	totalLateralCosts += trajectoryCosts.at(ic).lateral_cost;
	// 	totalLongitudinalCosts += trajectoryCosts.at(ic).longitudinal_cost;
	// }
	totalChange = thrust::reduce(trajectoryCostsDev.lane_change_cost_dev_vector.begin(),
				     trajectoryCostsDev.lane_change_cost_dev_vector.end());

	totalLateralCosts = thrust::reduce(trajectoryCostsDev.lateral_cost_dev_vector.begin(),
					   trajectoryCostsDev.lateral_cost_dev_vector.end());

	totalLongitudinalCosts = thrust::reduce(trajectoryCostsDev.longitudinal_cost_dev_vector.begin(),
						trajectoryCostsDev.longitudinal_cost_dev_vector.end());


	// for(unsigned int ic = 0; ic< trajectoryCosts.size(); ic++)
	// {
	// 	if(totalPriorities != 0)
	// 		trajectoryCosts.at(ic).priority_cost = trajectoryCosts.at(ic).priority_cost / totalPriorities;
	// 	else
	// 		trajectoryCosts.at(ic).priority_cost = 0;

	// 	if(transitionCosts != 0)
	// 		trajectoryCosts.at(ic).transition_cost = trajectoryCosts.at(ic).transition_cost / transitionCosts;
	// 	else
	// 		trajectoryCosts.at(ic).transition_cost = 0;

	// 	if(totalChange != 0)
	// 		trajectoryCosts.at(ic).lane_change_cost = trajectoryCosts.at(ic).lane_change_cost / totalChange;
	// 	else
	// 		trajectoryCosts.at(ic).lane_change_cost = 0;

	// 	if(totalLateralCosts != 0)
	// 		trajectoryCosts.at(ic).lateral_cost = trajectoryCosts.at(ic).lateral_cost / totalLateralCosts;
	// 	else
	// 		trajectoryCosts.at(ic).lateral_cost = 0;

	// 	if(totalLongitudinalCosts != 0)
	// 		trajectoryCosts.at(ic).longitudinal_cost = trajectoryCosts.at(ic).longitudinal_cost / totalLongitudinalCosts;
	// 	else
	// 		trajectoryCosts.at(ic).longitudinal_cost = 0;

	// 	trajectoryCosts.at(ic).cost = (
	// 			trajectoryCosts.at(ic).priority_cost +
	// 			trajectoryCosts.at(ic).lane_change_cost +
	// 			trajectoryCosts.at(ic).lateral_cost +
	// 			trajectoryCosts.at(ic).longitudinal_cost +
	// 			1.5*trajectoryCosts.at(ic).transition_cost) / 5.0;
	// }
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

    // Copy current data to GPU
    for (int index = 0; index < trajectoryCosts.size(); index++){
	TrajectoryCost cost = trajectoryCosts.at(index);

	devVectors.relative_index_dev_vector[index] = cost.relative_index;
	devVectors.closest_obj_velocity_dev_vector[index] = cost.closest_obj_velocity;
	devVectors.distance_from_center_dev_vector[index] = cost.distance_from_center;
	devVectors.priority_cost_dev_vector[index] = cost.priority_cost;
	devVectors.transition_cost_dev_vector[index] = cost.transition_cost;
	devVectors.closest_obj_cost_dev_vector[index] = cost.closest_obj_cost;
	devVectors.cost_dev_vector[index] = cost.cost;
	devVectors.closest_obj_distance_dev_vector[index] = cost.closest_obj_distance;
	devVectors.lane_index_dev_vector[index] = cost.lane_index;
	devVectors.lane_change_cost_dev_vector[index] = cost.lane_change_cost;
	devVectors.lateral_cost_dev_vector[index] = cost.lateral_cost;
	devVectors.longitudinal_cost_dev_vector[index] = cost.longitudinal_cost;
	devVectors.bBlocked_dev_vector[index] = cost.bBlocked;
    }
}

}
