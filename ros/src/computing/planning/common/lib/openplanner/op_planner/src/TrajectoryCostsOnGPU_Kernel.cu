/*
 * TrajectoryCostsOnGPU_Kernel.cpp
 *
 * GPU Kernels called from TrajectoryCostsOnGPU.cpp
 */

#include "TrajectoryCostsOnGPU_Kernel.h"
#include "GPUCalculationHelper.h"

#include <float.h>
#include <thrust/logical.h>
#include <thrust/functional.h>
#include <thrust/extrema.h>

namespace PlannerHNS
{
    // The calculation core for lateral and longitudinal cost by
    // CUDA dynamic parallelism
    __global__
    void CalculateLateralAndLongitudinalCosts_parallel(bool* bBlocked_buffer,
                                                       double* lateral_cost_buffer,
                                                       double* longitudinal_cost_buffer,
                                                       double* closest_obj_distance_buffer,
                                                       const int num_of_contourPoints,
                                                       const ForGPU::WayPoint* contourPoints,
                                                       const ForGPU::WayPoint* trajectory,
                                                       const int trajectory_length,
                                                       const ForGPU::RelativeInfo car_info,
                                                       const double distance_from_center,
                                                       const double carInfo_length,
                                                       const double critical_long_front_distance,
                                                       const ForGPU::GPSPoint* safetyBorder,
                                                       const int num_of_safetyBorder,
                                                       const double critical_lateral_distance,
                                                       const double params_minFollowingDistance) {
        // "icon" is the index value in this kernel
        int icon = blockIdx.x * blockDim.x + threadIdx.x;
        if (num_of_contourPoints <= icon) return;

        // Initialize output buffer
        bBlocked_buffer[icon] = false;
        lateral_cost_buffer[icon] = 0.f;
        longitudinal_cost_buffer[icon] = 0.f;
        closest_obj_distance_buffer[icon] = DBL_MAX; // set large value as the minimum value will be extracted later

        // Calculate each value
        ForGPU::RelativeInfo obj_info;
        ForGPU::GetRelativeInfo(trajectory, trajectory_length, contourPoints[icon], obj_info);
        double longitudinalDist = ForGPU::GetExactDistanceOnTrajectory(trajectory, trajectory_length, car_info, obj_info);

        if (obj_info.iFront == 0 && longitudinalDist > 0)
            longitudinalDist = -longitudinalDist;

        double lateralDist = fabs(obj_info.perp_distance - distance_from_center);

        if(longitudinalDist < -carInfo_length || lateralDist > 6)
            {
                // continue;
                return;
            }

        longitudinalDist = longitudinalDist - critical_long_front_distance;

        // Calculate bBlocked value for each buffer.
        // Here, if conditions are converted into bool operation to avoid divergent branch.
        // Original processes are following:
        // ==============================================
        // if(ForGPU::PolygonShape::PointInsidePolygon(safetyBorder, num_of_safetyBorder, contourPoints[icon].pos) == true)
        //     bBlocked_buffer[icon] = true;

        // if(lateralDist <= critical_lateral_distance
        //    && longitudinalDist >= -carInfo_length/1.5
        //    && longitudinalDist < params_minFollowingDistance)
        //     bBlocked_buffer[icon] = true;
        // ==============================================
        bool bBlocked_condition1 = ForGPU::PolygonShape::PointInsidePolygon(safetyBorder, num_of_safetyBorder, contourPoints[icon].pos);
        bool bBlocked_condition2 = (lateralDist <= critical_lateral_distance)
            && (longitudinalDist >= -carInfo_length/1.5)
            && (longitudinalDist < params_minFollowingDistance);

        bBlocked_buffer[icon] = bBlocked_condition1 && bBlocked_condition2;

        // Calculate lateral_cost and longitudinal_cost for each buffer
        lateral_cost_buffer[icon] = 1.0/lateralDist;
        longitudinal_cost_buffer[icon] = 1.0/fabs(longitudinalDist);

        // Calculate closest_obj_distance.
        // Here, if condition is converted into bool operation to avoid divergent branch.
        // Original processes are following:
        // ==============================================
        // if(longitudinalDist >= -critical_long_front_distance && longitudinalDist < closest_obj_distance_vector[iCostIndex])
        //     {
        //         closest_obj_distance_buffer[icon] = longitudinalDist;
        //     }
        // ==============================================
        bool closest_obj_distance_condition = longitudinalDist >= -critical_long_front_distance;
        closest_obj_distance_buffer[icon] = closest_obj_distance_condition * longitudinalDist
            + !(closest_obj_distance_condition) * DBL_MAX;

    } // void CalculateLateralAndLongitudinalCosts_parallel()


    // The entry point kernel for  calculate lateral and longitudinal cost
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
                                                     const double params_minFollowingDistance) {
        // Define indices of this thread
        int it = blockIdx.x * blockDim.x + threadIdx.x;
        int il = blockIdx.y * blockDim.y + threadIdx.y;
        int iCostIndex = it + il * valid_id_limit.x;

        // Check indices range
        if (valid_id_limit.x <= it || valid_id_limit.y <= il) return;

        ForGPU::RelativeInfo car_info;
        int trajectory_index;
        for (int i = 0; i < il; i++) {
            trajectory_index += totalPaths_each_length[i];
        }
        int trajectory_length = totalPaths_each_length[il];
        ForGPU::GetRelativeInfo(&(totalPaths[trajectory_index]),
                                trajectory_length,
                                currState[0],
                                car_info);

        // Adjust temporary buffer location
        // Values in each buffer will be merged by this thread after dynamic parallelism execution
        bool* bBlocked_tmp_buffer = bBlocked_tmp_buffer_begin + iCostIndex * num_of_contourPoints;
        double* lateral_cost_tmp_buffer = lateral_cost_tmp_buffer_begin + iCostIndex * num_of_contourPoints;
        double* longitudinal_cost_tmp_buffer = longitudinal_cost_tmp_buffer_begin + iCostIndex * num_of_contourPoints;
        double* closest_obj_distance_tmp_buffer = closest_obj_distance_tmp_buffer_begin + iCostIndex * num_of_contourPoints;

        // Define kernel shape of dynamic parallelism kernel
        dim3 block_dim(32, 1, 1);
        dim3 grid_dim((num_of_contourPoints + block_dim.x - 1) / block_dim.x, 1, 1);

        // Execute dynamic kernel
        CalculateLateralAndLongitudinalCosts_parallel<<<grid_dim, block_dim>>>(bBlocked_tmp_buffer,
                                                                               lateral_cost_tmp_buffer,
                                                                               longitudinal_cost_tmp_buffer,
                                                                               closest_obj_distance_tmp_buffer,
                                                                               num_of_contourPoints,
                                                                               contourPoints,
                                                                               &(totalPaths[trajectory_index]),
                                                                               trajectory_length,
                                                                               car_info,
                                                                               distance_from_center_vector[iCostIndex],
                                                                               carInfo_length,
                                                                               critical_long_front_distance,
                                                                               safetyBorder,
                                                                               num_of_safetyBorder,
                                                                               critical_lateral_distance,
                                                                               params_minFollowingDistance);
        // Synchronize only child kernel launched by this thread
        cudaDeviceSynchronize();

        // Merge values in temporary buffer

        // bBlocked_vector[iCostIndex] will be true if bBlocked_tmp_buffer contains more than one "true" value
        bBlocked_vector[iCostIndex] = thrust::any_of(thrust::device,
                                                     bBlocked_tmp_buffer,
                                                     bBlocked_tmp_buffer + num_of_contourPoints,
                                                     thrust::identity<bool>());

        // lateral_cost_vector[iCostIndex] and longitudinal_cost_vector[iCostIndex]
        // will be simply sum of all element of corresponding buffer
        lateral_cost_vector[iCostIndex] = thrust::reduce(thrust::device,
                                                         lateral_cost_tmp_buffer,
                                                         lateral_cost_tmp_buffer + num_of_contourPoints);

        longitudinal_cost_vector[iCostIndex] = thrust::reduce(thrust::device,
                                                              longitudinal_cost_tmp_buffer,
                                                              longitudinal_cost_tmp_buffer + num_of_contourPoints);

        // closest_obj_distance_vector[iCostIndex] will be the minimum value in correspoinding buffer
        double* minimum_element = thrust::min_element(thrust::device,
                                                      closest_obj_distance_tmp_buffer,
                                                      closest_obj_distance_tmp_buffer + num_of_contourPoints);
        int minimum_element_index = thrust::distance(closest_obj_distance_tmp_buffer, minimum_element);

        closest_obj_distance_vector[iCostIndex] = *minimum_element;
        closest_obj_velocity_vector[iCostIndex] = contourPoints[minimum_element_index].v;

#if 0
        // ******************************************************************
        // ******************************************************************
        // ******************************************************************
        // ******************************************************************
        // ******************************************************************
        // ここのループをdynamic parallel executionで並列化する
        // ******************************************************************
        // ******************************************************************
        // ******************************************************************
        // ******************************************************************
        // ******************************************************************
        for (int icon = 0; icon < num_of_contourPoints; icon++) {
            ForGPU::RelativeInfo obj_info;
            ForGPU::GetRelativeInfo(&(totalPaths[trajectory_index]), trajectory_length, contourPoints[icon], obj_info);
            double longitudinalDist = ForGPU::GetExactDistanceOnTrajectory(&(totalPaths[trajectory_index]), trajectory_length, car_info, obj_info);

            if (obj_info.iFront == 0 && longitudinalDist > 0)
                longitudinalDist = -longitudinalDist;

            double close_in_percentage = 1;
            double distance_from_center = distance_from_center_vector[iCostIndex];

            double lateralDist = fabs(obj_info.perp_distance - distance_from_center);

            if(longitudinalDist < -carInfo_length || lateralDist > 6)
                {
                    continue;
                }

            longitudinalDist = longitudinalDist - critical_long_front_distance;

            if(ForGPU::PolygonShape::PointInsidePolygon(safetyBorder, num_of_safetyBorder, contourPoints[icon].pos) == true)
                bBlocked_vector[iCostIndex] = true;

            if(lateralDist <= critical_lateral_distance
               && longitudinalDist >= -carInfo_length/1.5
               && longitudinalDist < params_minFollowingDistance)
                bBlocked_vector[iCostIndex] = true;

            lateral_cost_vector[iCostIndex] += 1.0/lateralDist;
            longitudinal_cost_vector[iCostIndex] += 1.0/fabs(longitudinalDist);

            if(longitudinalDist >= -critical_long_front_distance && longitudinalDist < closest_obj_distance_vector[iCostIndex])
                {
                    closest_obj_distance_vector[iCostIndex] = longitudinalDist;
                    closest_obj_velocity_vector[iCostIndex] = contourPoints[icon].v;
                }
        }
#endif
    }  // void CalculateLateralAndLongitudinalCosts_kernel()
}  // namespace PlannerHNS
