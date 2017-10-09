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
