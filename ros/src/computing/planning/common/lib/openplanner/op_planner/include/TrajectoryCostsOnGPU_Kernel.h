/*
 * TrajectoryCostsOnGPU_Kernel.h
 */

#ifndef TRAJECTORYCOSTSONGPU_KERNEL_H_
#define TRAJECTORYCOSTSONGPU_KERNEL_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>

#include <thrust/fill.h>
#include <thrust/transform.h>
#include <thrust/tuple.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/zip_iterator.h>

#include "GPUCalculationHelper.h"

namespace PlannerHNS
{
    /*
     * A Functor to calculate transition cost using thrust
     */
    struct TransitionCostCalculateFunctor
    {
	double rollOutDensity_;
	int currTrajectoryIndex_;

	TransitionCostCalculateFunctor(double roll_out_density, int curr_trajectory_index):
	    rollOutDensity_(roll_out_density),
	    currTrajectoryIndex_(curr_trajectory_index) {}

	__device__
	double operator()(int index)
	{
            return fabs(rollOutDensity_ * (index - currTrajectoryIndex_));
	}
    };

    /*
     * A Functor to normalize costs
     */
    struct NormalizeCostsFunctor
    {
	double totalPriorities_;
	double totalChange_;
	double totalLateralCosts_;
	double totalLongitudinalCosts_;
	double transitionCosts_;

	NormalizeCostsFunctor(double totalPriorities,
			      double totalChange,
			      double totalLateralCosts,
			      double totalLongitudinalCosts,
			      double transitionCosts):
	    totalPriorities_(totalPriorities),
	    totalChange_(totalChange),
	    totalLateralCosts_(totalLateralCosts),
	    totalLongitudinalCosts_(totalLongitudinalCosts),
	    transitionCosts_(transitionCosts) {}

	template <typename Tuple>
	__device__
	double operator()(Tuple t)
	{
	    auto &priority_cost = thrust::get<0>(t);
	    auto &transition_cost = thrust::get<1>(t);
	    auto &lane_change_cost = thrust::get<2>(t);
	    auto &lateral_cost = thrust::get<3>(t);
	    auto &longitudinal_cost = thrust::get<4>(t);
	    auto &cost = thrust::get<5>(t);

	    priority_cost = (totalPriorities_ != 0)*(priority_cost / totalPriorities_);
	    transition_cost = (transitionCosts_ != 0)*(transition_cost / transitionCosts_);
	    lane_change_cost = (totalChange_ != 0)*(lane_change_cost / totalChange_);
	    lateral_cost = (totalLateralCosts_ != 0)*(lateral_cost / totalLateralCosts_);
	    longitudinal_cost = (totalLongitudinalCosts_ != 0)*(longitudinal_cost / totalLongitudinalCosts_);

	    cost = (priority_cost +
		    lane_change_cost +
		    lateral_cost +
		    longitudinal_cost +
		    1.5 * transition_cost) / 5.0;
	}
    };

    /*
     * A kernel to calculate lateral and longitudinal cost on GPU
     */
    __global__
    void CalculateLateralAndLongitudinalCosts_kernel(bool* bBlocked_vector,
						     bool* bBlocked_tmp_buffer_begin,
						     double* lateral_cost_vector,
						     double* lateral_cost_tmp_buffer_begin,
						     double* longitudinal_cost_vector,
						     double* longitudinal_cost_tmp_buffer_begin,
						     double* closest_obj_distance_vector,
						     double* closest_obj_distance_tmp_buffer_begin,
						     double* closest_obj_velocity_vector,
						     const dim3 valid_id_limit,
						     const ForGPU::WayPoint* totalPaths,
						     const int* totalPaths_each_length,
						     const ForGPU::WayPoint* currState,
						     const ForGPU::WayPoint* contourPoints,
						     const int num_of_contourPoints,
						     const double* distance_from_center_vector,
						     const double critical_long_front_distance,
						     const double critical_lateral_distance,
						     const ForGPU::GPSPoint* safetyBorder,
						     const int num_of_safetyBorder,
						     const double carInfo_length,
						     const double params_minFollowingDistance);
}

#endif /* TRAJECTORYCOSTSONGPU_KERNEL_H_ */
