#include "include/euclidean_cluster.h"
#include <cuda.h>

// Build edge set
__global__ void edgeCount(float *x, float *y, float *z, int point_num, int *edge_count, float threshold)
{
	__shared__ float local_x[BLOCK_SIZE_X];
	__shared__ float local_y[BLOCK_SIZE_X];
	__shared__ float local_z[BLOCK_SIZE_X];
	int pid;
	int last_point = (point_num / blockDim.x) * blockDim.x;	// Exclude the last block
	float dist;

	for (pid = threadIdx.x + blockIdx.x * blockDim.x; pid < last_point; pid += blockDim.x * gridDim.x) {
		float tmp_x = x[pid];
		float tmp_y = y[pid];
		float tmp_z = z[pid];
		int count = 0;

		int block_id;

		for (block_id = blockIdx.x * blockDim.x; block_id + blockDim.x < point_num; block_id += blockDim.x) {
			local_x[threadIdx.x] = x[block_id + threadIdx.x];
			local_y[threadIdx.x] = y[block_id + threadIdx.x];
			local_z[threadIdx.x] = z[block_id + threadIdx.x];
			__syncthreads();

			for (int i = 0; i < blockDim.x; i++) {
				dist = norm3df(tmp_x - local_x[i], tmp_y - local_y[i], tmp_z - local_z[i]);
				count += (i + block_id > pid && dist < threshold) ? 1 : 0;
			}
			__syncthreads();
		}

		__syncthreads();

		// Compare with last block
		if (threadIdx.x < point_num - block_id) {
			local_x[threadIdx.x] = x[block_id + threadIdx.x];
			local_y[threadIdx.x] = y[block_id + threadIdx.x];
			local_z[threadIdx.x] = z[block_id + threadIdx.x];
		}
		__syncthreads();

		for (int i = 0; i < point_num - block_id; i++) {
			dist = norm3df(tmp_x - local_x[i], tmp_y - local_y[i], tmp_z - local_z[i]);
			count += (i + block_id > pid && dist < threshold) ? 1 : 0;
		}

		edge_count[pid] = count;
		__syncthreads();
	}
	__syncthreads();


	// Handle last block
	if (pid >= last_point) {
		int count = 0;
		float tmp_x, tmp_y, tmp_z;

		if (pid < point_num) {
			tmp_x = x[pid];
			tmp_y = y[pid];
			tmp_z = z[pid];
		}

		int block_id = blockIdx.x * blockDim.x;

		__syncthreads();

		if (pid < point_num) {
			local_x[threadIdx.x] = x[pid];
			local_y[threadIdx.x] = y[pid];
			local_z[threadIdx.x] = z[pid];
			__syncthreads();

			for (int i = 0; i < point_num - block_id; i++) {
				dist = norm3df(tmp_x - local_x[i], tmp_y - local_y[i], tmp_z - local_z[i]);
				count += (i + block_id > pid && dist < threshold) ? 1 : 0;
			}
			__syncthreads();

			edge_count[pid] = count;
		}
	}
}


__global__ void buildEdgeSet(float *x, float *y, float *z, int point_num, int *edge_count, int2 *edge_set, float threshold)
{
	__shared__ float local_x[BLOCK_SIZE_X];
	__shared__ float local_y[BLOCK_SIZE_X];
	__shared__ float local_z[BLOCK_SIZE_X];
	int pid;
	int last_point = (point_num / blockDim.x) * blockDim.x;
	int2 new_edge;

	for (pid = threadIdx.x + blockIdx.x * blockDim.x; pid < last_point; pid += blockDim.x * gridDim.x) {
		int writing_location = edge_count[pid];
		float tmp_x = x[pid];
		float tmp_y = y[pid];
		float tmp_z = z[pid];

		int block_id;

		new_edge.x = pid;

		for (block_id = blockIdx.x * blockDim.x; block_id + blockDim.x < point_num; block_id += blockDim.x) {
			local_x[threadIdx.x] = x[block_id + threadIdx.x];
			local_y[threadIdx.x] = y[block_id + threadIdx.x];
			local_z[threadIdx.x] = z[block_id + threadIdx.x];
			__syncthreads();

			for (int i = 0; i < blockDim.x; i++) {
				if (i + block_id > pid && norm3df(tmp_x - local_x[i], tmp_y - local_y[i], tmp_z - local_z[i]) < threshold) {
					new_edge.y = i + block_id;
					edge_set[writing_location++] = new_edge;
				}
			}
			__syncthreads();
		}
		__syncthreads();


		if (threadIdx.x < point_num - block_id) {
			local_x[threadIdx.x] = x[block_id + threadIdx.x];
			local_y[threadIdx.x] = y[block_id + threadIdx.x];
			local_z[threadIdx.x] = z[block_id + threadIdx.x];
		}
		__syncthreads();

		for (int i = 0; i < point_num - block_id; i++) {
			if (i + block_id > pid && norm3df(tmp_x - local_x[i], tmp_y - local_y[i], tmp_z - local_z[i]) < threshold) {
				new_edge.y = i + block_id;
				edge_set[writing_location++] = new_edge;
			}
		}
		__syncthreads();

	}

	if (pid >= last_point) {
		float tmp_x, tmp_y, tmp_z;
		int writing_location;

		if (pid < point_num) {
			new_edge.x = pid;
			tmp_x = x[pid];
			tmp_y = y[pid];
			tmp_z = z[pid];
			writing_location = edge_count[pid];
		}

		int block_id = blockIdx.x * blockDim.x;

		__syncthreads();

		if (pid < point_num) {
			local_x[threadIdx.x] = x[pid];
			local_y[threadIdx.x] = y[pid];
			local_z[threadIdx.x] = z[pid];
			__syncthreads();

			for (int i = 0; i < point_num - block_id; i++) {
				if (i + block_id > pid && norm3df(tmp_x - local_x[i], tmp_y - local_y[i], tmp_z - local_z[i]) < threshold) {
					new_edge.y = i + block_id;
					edge_set[writing_location++] = new_edge;
				}
			}
		}
	}
}


__global__ void clustering(int2 *edge_set, int size, int *cluster_name, bool *changed)
{
	for (int i = threadIdx.x + blockIdx.x * blockDim.x; i < size; i += blockDim.x * gridDim.x) {
		int2 cur_edge = edge_set[i];
		int x = cur_edge.x;
		int y = cur_edge.y;

		int x_name = cluster_name[x];
		int y_name = cluster_name[y];
		int *changed_addr = NULL;
		int change_name;

		if (x_name < y_name) {
			changed_addr = cluster_name + y;
			change_name = x_name;
		} else if (x_name > y_name) {
			changed_addr = cluster_name + x;
			change_name = y_name;
		}

		if (changed_addr != NULL) {
			atomicMin(changed_addr, change_name);
			*changed = true;
		}
	}
}

__global__ void clusterCount(int *cluster_name, int *count, int point_num)
{
	for (int i = threadIdx.x + blockIdx.x * blockDim.x; i < point_num; i += blockDim.x * gridDim.x) {
		count[cluster_name[i]] = 1;
	}
}

void GpuEuclideanCluster2::extractClusters3()
{
	initClusters();

	int block_x, grid_x;

	block_x = (point_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : point_num_;
	grid_x = (point_num_ - 1) / block_x + 1;

	int *edge_count;

	checkCudaErrors(cudaMalloc(&edge_count, sizeof(int) * (point_num_ + 1)));

	edgeCount<<<grid_x, block_x>>>(x_, y_, z_, point_num_, edge_count, threshold_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	int edge_num;

	exclusiveScan(edge_count, point_num_ + 1, &edge_num);

	if (edge_num == 0) {
		checkCudaErrors(cudaFree(edge_count));
		cluster_num_ = 0;
	}

	int2 *edge_set;

	checkCudaErrors(cudaMalloc(&edge_set, sizeof(int2) * edge_num));

	buildEdgeSet<<<grid_x, block_x>>>(x_, y_, z_, point_num_, edge_count, edge_set, threshold_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	bool *changed;

	checkCudaErrors(cudaMallocHost(&changed, sizeof(bool)));

	block_x = (edge_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : edge_num;
	grid_x = (edge_num - 1) / block_x + 1;

	do {
		*changed = false;

		clustering<<<grid_x, block_x>>>(edge_set, edge_num, cluster_name_, changed);
		checkCudaErrors(cudaGetLastError());
		checkCudaErrors(cudaDeviceSynchronize());
	} while (*changed);

	int *count;

	checkCudaErrors(cudaMalloc(&count, sizeof(int) * (point_num_ + 1)));
	checkCudaErrors(cudaMemset(count, 0, sizeof(int) * (point_num_ + 1)));

	block_x = (point_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : point_num_;
	grid_x = (point_num_ - 1) / block_x + 1;

	clusterCount<<<grid_x, block_x>>>(cluster_name_, count, point_num_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());


	exclusiveScan(count, point_num_ + 1, &cluster_num_);

	renamingClusters(cluster_name_, count, point_num_);

	checkCudaErrors(cudaMemcpy(cluster_name_host_, cluster_name_, sizeof(int) * point_num_, cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaFree(edge_count));
	checkCudaErrors(cudaFree(edge_set));
	checkCudaErrors(cudaFreeHost(changed));
	checkCudaErrors(cudaFree(count));
}
