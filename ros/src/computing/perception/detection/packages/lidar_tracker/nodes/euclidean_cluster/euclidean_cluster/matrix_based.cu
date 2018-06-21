#include "include/euclidean_cluster.h"
#include <cuda.h>

#define NON_ATOMIC_ 1


/* Arrays to remember:
 * cluster_name: the index of the cluster that each point belong to
 * 				i.e. point at index i belong to cluster cluster_name[i]
 * cluster_list: the list of remaining clusters
 * cluster_location: location of the remaining clusters in the cluster list
 * 					i.e. cluster A locate in index cluster_location[A] in the
 * 					cluster_list
 * matrix: the adjacency matrix of the cluster list, each cluster is a vertex.
 * 			This matrix is rebuilt whenever some clusters are merged together
 */


/* Connected component labeling points at GPU block thread level.
 * Input list of points is divided into multiple smaller groups.
 * Each group of point is assigned to a block of GPU thread.
 * Each thread in a block handles one point in the group. It iterates over
 * points in the group and compare the distance between the current point A
 * and the point B it has to handle.
 *
 * If the distance between A and B is less than the threshold, then those
 * two points belong to a same connected component and the cluster_changed
 * is marked by 1.
 *
 * A synchronization is called to make sure all thread in the block finish A
 * before moving to the update phase.
 * After finishing checking cluster_changed, threads update the cluster
 * index of all points. If a thread has cluster_changed is 1, then the corresponding
 * cluster of the point it is handling is changed to the cluster of B. Otherwise
 * the original cluster of A remains unchanged.
 *
 * Another synchronization is called before all threads in the block move to
 * other points after done checking A.
 *
 * After this kernel finishes, all points in each block are labeled.
 */
__global__ void blockClustering(float *x, float *y, float *z, int point_num, int *cluster_name, float threshold)
{
	int block_start = blockIdx.x * blockDim.x;
	int block_end = (block_start + blockDim.x > point_num) ? point_num : block_start + blockDim.x;
	__shared__ float local_x[BLOCK_SIZE_X];
	__shared__ float local_y[BLOCK_SIZE_X];
	__shared__ float local_z[BLOCK_SIZE_X];
	/* Each thread is in charge of one point in the block.*/
	int pid = threadIdx.x + block_start;
	/* Local cluster to record the change in the name of the cluster each point belong to */
	__shared__ int local_cluster_idx[BLOCK_SIZE_X];
	/* Cluster changed to check if a cluster name has changed after each comparison */
	__shared__ int cluster_changed[BLOCK_SIZE_X];

	if (pid < block_end) {
		local_cluster_idx[threadIdx.x] = threadIdx.x;
		local_x[threadIdx.x] = x[pid];
		local_y[threadIdx.x] = y[pid];
		local_z[threadIdx.x] = z[pid];
		__syncthreads();

		float cx = local_x[threadIdx.x];
		float cy = local_y[threadIdx.x];
		float cz = local_z[threadIdx.x];

		/* Iterate through all points in the block and check if the point at row index
		 * and at column index belong to the same cluster.
		 * If so, then name of the cluster of the row point is changed into the name
		 * of the cluster of column point.
		 * */
		for (int rid = 0; rid < block_end - block_start; rid++) {
			float distance = norm3df(cx - local_x[rid], cy - local_y[rid], cz - local_z[rid]);
			int row_cluster = local_cluster_idx[rid];
			int col_cluster = local_cluster_idx[threadIdx.x];

			cluster_changed[threadIdx.x] = 0;
			__syncthreads();

			if (threadIdx.x > rid && distance < threshold) {
				cluster_changed[col_cluster] = 1;
			}
			__syncthreads();

			local_cluster_idx[threadIdx.x] = (cluster_changed[col_cluster] == 1) ? row_cluster : col_cluster;
			__syncthreads();
		}
		__syncthreads();

		int new_cluster = cluster_name[block_start + local_cluster_idx[threadIdx.x]];
		__syncthreads();

		cluster_name[pid] = new_cluster;
	}
}

/* Iterate through the list of remaining clusters and mark the corresponding
 * location on cluster location array by 1
 */
__global__ void clusterMark(int *cluster_list, int *cluster_location, int cluster_num)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;

	for (int i = idx; i < cluster_num; i += blockDim.x * gridDim.x) {
		cluster_location[cluster_list[i]] = 1;
	}
}

/* Collect the remaining clusters */
__global__ void clusterCollector(int *old_cluster_list, int *new_cluster_list, int *cluster_location, int cluster_num)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;

	for (int i = idx; i < cluster_num; i += blockDim.x * gridDim.x) {
		new_cluster_list[cluster_location[old_cluster_list[i]]] = old_cluster_list[i];
	}
}

__global__ void buildClusterMatrix(float *x, float *y, float *z, int *cluster_name, int *cluster_location, int *matrix, int point_num, int cluster_num, float threshold, bool *non_zero)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ bool snon_zero;

	if (threadIdx.x == 0)
		snon_zero = false;
	__syncthreads();

	for (int pid = idx; pid < point_num; pid += stride) {
		float tmp_x = x[pid];
		float tmp_y = y[pid];
		float tmp_z = z[pid];
		int col_cluster = cluster_name[pid];
		int col = cluster_location[col_cluster];

		for (int pid2 = 0; pid2 < pid; pid2++) {
			float tmp_x2 = tmp_x - x[pid2];
			float tmp_y2 = tmp_y - y[pid2];
			float tmp_z2 = tmp_z - z[pid2];
			int row_cluster = cluster_name[pid2];
			int row = cluster_location[row_cluster];

			if (row_cluster != col_cluster && norm3df(tmp_x2, tmp_y2, tmp_z2) < threshold) {
				matrix[row * cluster_num + col] = 1;
				snon_zero = true;
			}
			__syncthreads();
		}
		__syncthreads();
	}

	__syncthreads();

	if (threadIdx.x == 0 && snon_zero)
		*non_zero = true;
}

__global__ void buildClusterMatrix2(float *x, float *y, float *z, int *cluster_name, int *cluster_location, int *matrix, int point_num, int cluster_num, float threshold, bool *non_zero)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ bool snon_zero;

	if (threadIdx.x == 0)
		snon_zero = false;
	__syncthreads();

	for (int pid = idx; pid < point_num; pid += stride) {
		float tmp_x = x[pid];
		float tmp_y = y[pid];
		float tmp_z = z[pid];
		int col_cluster = cluster_name[pid];
		int col = cluster_location[col_cluster];

		for (int pid2 = blockIdx.y; pid2 < pid; pid2 += gridDim.y) {
			float tmp_x2 = tmp_x - x[pid2];
			float tmp_y2 = tmp_y - y[pid2];
			float tmp_z2 = tmp_z - z[pid2];
			int row_cluster = cluster_name[pid2];
			int row = cluster_location[row_cluster];

			if (row_cluster != col_cluster && norm3df(tmp_x2, tmp_y2, tmp_z2) < threshold) {
				matrix[row * cluster_num + col] = 1;
				snon_zero = true;
			}
			__syncthreads();
		}
		__syncthreads();
	}

	__syncthreads();

	if (threadIdx.x == 0 && snon_zero)
		*non_zero = true;
}


/* Merge clusters that belong to a same block of threads */
__global__ void mergeLocalClusters(int *cluster_list, int *matrix, int cluster_num, bool *changed)
{
	int row_start = blockIdx.x * blockDim.x;
	int row_end = (row_start + blockDim.x <= cluster_num) ? row_start + blockDim.x : cluster_num;
	int col = row_start + threadIdx.x;
	__shared__ int local_cluster_idx[BLOCK_SIZE_X];
	__shared__ int cluster_changed[BLOCK_SIZE_X];
	bool block_changed = false;
	__shared__ bool schanged;

	if (threadIdx.x == 0)
		schanged = false;
	__syncthreads();

	if(col < cluster_num && row_start < row_end) {
		local_cluster_idx[threadIdx.x] = threadIdx.x;
		__syncthreads();

		for (int row = row_start; row < row_end; row++) {
			int col_cluster = local_cluster_idx[threadIdx.x];
			int row_cluster = local_cluster_idx[row - row_start];

			cluster_changed[threadIdx.x] = 0;
			__syncthreads();

			if (row - row_start < threadIdx.x && row_cluster != col_cluster && matrix[row * cluster_num + col] == 1) {
				cluster_changed[threadIdx.x] = 1;
				block_changed = true;
			}
			__syncthreads();

			local_cluster_idx[threadIdx.x] = (cluster_changed[threadIdx.x] == 1) ? row_cluster : col_cluster;
			__syncthreads();
		}

		__syncthreads();
		int new_cluster = cluster_list[row_start + local_cluster_idx[threadIdx.x]];
		__syncthreads();

		cluster_list[col] = new_cluster;

		if (block_changed)
			schanged = true;
	}

	__syncthreads();
	if (threadIdx.x == 0 && schanged)
		*changed = true;
}

/* Merge clusters that belong to different block of threads*/
__global__ void mergeForeignClusters(int *matrix, int *cluster_list,
										int shift_level,
										int sub_mat_size,
										int sub_mat_offset,
										int cluster_num, bool *changed)
{
	// sub_mat_col_base = sub_matrix_size
	// sub_mat_row_base = 0
	int sub_mat_idx = blockIdx.x / sub_mat_size;
//	int local_col = (shift_level + blockIdx.x) % sub_mat_size;		// block column that the thread is in charged of
//	int local_row = blockIdx.x % sub_mat_size;
//	int sub_mat_row = sub_mat_idx * sub_mat_offset;	// First row on the left of the sub matrix
//	int sub_mat_col = sub_mat_size + sub_mat_idx * sub_mat_offset;

	int col_start = (sub_mat_size + sub_mat_idx * sub_mat_offset + (shift_level + blockIdx.x) % sub_mat_size) * blockDim.x;
	int col_end = (col_start + blockDim.x <= cluster_num) ? col_start + blockDim.x : cluster_num;
	int row_start = (sub_mat_idx * sub_mat_offset + blockIdx.x % sub_mat_size) * blockDim.x;
	int row_end = (row_start + blockDim.x <= cluster_num) ? row_start + blockDim.x : cluster_num;
	int col = col_start + threadIdx.x;
	bool block_changed = true;
	__shared__ bool schanged;

	__shared__ int cluster_changed[BLOCK_SIZE_X];
	__shared__ int local_cluster_idx[BLOCK_SIZE_X];

	if (threadIdx.x == 0)
		schanged = false;
	__syncthreads();

	if (col < col_end && row_start < row_end) {
		local_cluster_idx[threadIdx.x] = threadIdx.x;
		__syncthreads();

		for (int row = row_start; row < row_end; row++) {
			int col_cluster = local_cluster_idx[threadIdx.x];
			int row_cluster = local_cluster_idx[row - row_start];

			cluster_changed[threadIdx.x] = 0;
			__syncthreads();

			if (row_cluster != col_cluster && matrix[row * cluster_num + col] == 1) {
				cluster_changed[col_cluster] = 1;
				block_changed = true;
			}
			__syncthreads();

			local_cluster_idx[threadIdx.x] = (cluster_changed[col_cluster] == 1) ? row_cluster : col_cluster;
			__syncthreads();
		}

		__syncthreads();
		int new_cluster = cluster_list[row_start + local_cluster_idx[threadIdx.x]];
		__syncthreads();

		cluster_list[col] = new_cluster;

		if (block_changed)
			schanged = true;
	}

	__syncthreads();
	if (threadIdx.x == 0 && schanged)
		*changed = true;
}

/* Check if are there any 1 element in the adjacency matrix */
__global__ void clusterIntersecCheck(int *matrix, int *changed_diag, int sub_mat_size, int sub_mat_offset, int cluster_num)
{
	int col_idx = ((blockIdx.x / sub_mat_size) * sub_mat_offset + blockIdx.x % sub_mat_size);
	int row_idx = ((blockIdx.y / sub_mat_size) * sub_mat_offset + blockIdx.y % sub_mat_size);

	int col_start = (sub_mat_size + col_idx) * blockDim.x;
	int col_end = (col_start + blockDim.x <= cluster_num) ? col_start + blockDim.x : cluster_num;

	int row_start = row_idx * blockDim.y;
	int row_end = (row_start + blockDim.y <= cluster_num) ? row_start + blockDim.y : cluster_num;

	int col = col_start + threadIdx.x;
	int diag_offset = (sub_mat_size + col_idx - row_idx) % sub_mat_size;
	__shared__ int schanged_diag;

	if (threadIdx.x == 0)
		schanged_diag = -1;
	__syncthreads();

	if (col < col_end && col_start < col_end && row_start < row_end) {
		for (int row = row_start; row < row_end; row ++) {
			if (matrix[row * cluster_num + col] != 0) {
				schanged_diag = diag_offset;
				break;
			}
		}
	}

	__syncthreads();
	if (threadIdx.x == 0 && schanged_diag >= 0)
		*changed_diag = schanged_diag;
}

/* Rename the cluster name of each point after some clusters are joined together */
__global__ void applyClusterChanged(int *cluster_name, int *cluster_list, int *cluster_location, int point_num)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;

	for (int i = idx; i < point_num; i += blockDim.x * gridDim.x) {
		int old_cluster = cluster_name[i];

		cluster_name[i] = cluster_list[cluster_location[old_cluster]];
	}
}

/* Rebuild the adjacency matrix after some clusters are joined together */
__global__ void rebuildMatrix(int *old_matrix, int *updated_cluster_list, int *new_matrix, int *new_cluster_location, int old_cluster_num, int new_cluster_num)
{
	for (int col = threadIdx.x + blockIdx.x * blockDim.x; col < old_cluster_num; col += blockDim.x * gridDim.x) {
		int new_col = new_cluster_location[updated_cluster_list[col]];
		for (int row = 0; row < col; row++) {
			int new_row = new_cluster_location[updated_cluster_list[row]];

			if (old_matrix[row * old_cluster_num + col] != 0) {
				new_matrix[new_row * new_cluster_num + new_col] = 1;
			}
		}
	}
}

__global__ void rebuildMatrix2(int *old_matrix, int *updated_cluster_list, int *new_matrix, int *new_cluster_location, int old_cluster_num, int new_cluster_num)
{
	for (int col = threadIdx.x + blockIdx.x * blockDim.x; col < old_cluster_num; col += blockDim.x * gridDim.x) {
		int new_col = new_cluster_location[updated_cluster_list[col]];
		for (int row = blockIdx.y; row < col; row += gridDim.y) {
			int new_row = new_cluster_location[updated_cluster_list[row]];

			if (old_matrix[row * old_cluster_num + col] != 0) {
				new_matrix[new_row * new_cluster_num + new_col] = 1;
			}
		}
	}

}

void GpuEuclideanCluster2::extractClusters()
{
	struct timeval start, end;

	// Initialize names of clusters
	initClusters();

	bool *check;
	bool hcheck = false;

	checkCudaErrors(cudaMalloc(&check, sizeof(bool)));
	checkCudaErrors(cudaMemcpy(check, &hcheck, sizeof(bool), cudaMemcpyHostToDevice));

	int block_x, grid_x;

	block_x = (point_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : point_num_;
	grid_x = (point_num_ - 1) / block_x + 1;

	gettimeofday(&start, NULL);
	// Divide points into blocks of points and clustering points inside each block
	blockClustering<<<grid_x, block_x>>>(x_, y_, z_, point_num_, cluster_name_, threshold_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());
	gettimeofday(&end, NULL);

	std::cout << "blockClustering = " << timeDiff(start, end) << std::endl;

	// Collect the remaining clusters
	// Locations of clusters in the cluster list
	int *cluster_location;

	gettimeofday(&start, NULL);
	checkCudaErrors(cudaMalloc(&cluster_location, sizeof(int) * (point_num_ + 1)));
	checkCudaErrors(cudaMemset(cluster_location, 0, sizeof(int) * (point_num_ + 1)));
	clusterMark<<<grid_x, block_x>>>(cluster_name_, cluster_location, point_num_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	int current_cluster_num = 0;
	exclusiveScan(cluster_location, point_num_ + 1, &current_cluster_num);

	int *cluster_list;

	checkCudaErrors(cudaMalloc(&cluster_list, sizeof(int) * current_cluster_num));

	clusterCollector<<<grid_x, block_x>>>(cluster_name_, cluster_list, cluster_location, point_num_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	cluster_num_ = current_cluster_num;

	// Build relation matrix which describe the current relationship between clusters
	int *matrix;

	checkCudaErrors(cudaMalloc(&matrix, sizeof(int) * cluster_num_ * cluster_num_));
	checkCudaErrors(cudaMemset(matrix, 0, sizeof(int) * cluster_num_ * cluster_num_));

	dim3 grid_size, block_size;

	block_size.x = block_x;
	block_size.y = block_size.z = 1;
	grid_size.x = grid_x;
	grid_size.y = (cluster_num_ > GRID_SIZE_Y) ? GRID_SIZE_Y : cluster_num_;
	grid_size.z = 1;
	//grid_size.y = 1;

	buildClusterMatrix2<<<grid_size, block_size>>>(x_, y_, z_, cluster_name_,
													cluster_location, matrix,
													point_num_, cluster_num_,
													threshold_, check);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());
	gettimeofday(&end, NULL);

	std::cout << "Build RC and Matrix = " << timeDiff(start, end) << std::endl;

	int *changed_diag;
	int hchanged_diag;
	checkCudaErrors(cudaMalloc(&changed_diag, sizeof(int)));


	int *new_cluster_list;

	gettimeofday(&start, NULL);
	do {
		hcheck = false;
		hchanged_diag = -1;

		checkCudaErrors(cudaMemcpy(check, &hcheck, sizeof(bool), cudaMemcpyHostToDevice));

		int block_x2, grid_x2, grid_y2;

		block_x2 = (cluster_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : cluster_num_;
		grid_x2 = (cluster_num_ - 1) / block_x2 + 1;

		mergeLocalClusters<<<grid_x2, block_x2>>>(cluster_list, matrix, cluster_num_, check);
		checkCudaErrors(cudaGetLastError());
		checkCudaErrors(cudaDeviceSynchronize());

		int sub_matrix_size = 1;
		int sub_matrix_offset = 2;

		checkCudaErrors(cudaMemcpy(&hcheck, check, sizeof(bool), cudaMemcpyDeviceToHost));


		while (!(hcheck) && sub_matrix_size < cluster_num_ && cluster_num_ > BLOCK_SIZE_X) {

			int sub_matrix_num = (cluster_num_ - 1) / sub_matrix_offset + 1;
			block_x2 = BLOCK_SIZE_X;
			grid_x2 = sub_matrix_size * sub_matrix_num;
			grid_y2 = sub_matrix_size;

			block_size.x = block_x2;
			block_size.y = block_size.z = 1;
			grid_size.x = grid_x2;
			grid_size.y = grid_y2;
			grid_size.z = 1;
			hchanged_diag = -1;

			checkCudaErrors(cudaMemcpy(changed_diag, &hchanged_diag, sizeof(int), cudaMemcpyHostToDevice));

			clusterIntersecCheck<<<grid_size, block_size>>>(matrix, changed_diag, sub_matrix_size, sub_matrix_offset, cluster_num_);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());

			checkCudaErrors(cudaMemcpy(&hchanged_diag, changed_diag, sizeof(int), cudaMemcpyDeviceToHost));

			if (hchanged_diag >= 0) {
				mergeForeignClusters<<<grid_x2, block_x2>>>(matrix, cluster_list, hchanged_diag,
															sub_matrix_size, sub_matrix_offset,
															cluster_num_, check);
				checkCudaErrors(cudaGetLastError());
				checkCudaErrors(cudaDeviceSynchronize());

				checkCudaErrors(cudaMemcpy(&hcheck, check, sizeof(bool), cudaMemcpyDeviceToHost));
			}

			sub_matrix_size *= 2;
			sub_matrix_offset *= 2;
		}


		/* If some changes in the cluster list are recorded (some clusters are merged together),
		 * rebuild the matrix, the cluster location, and apply those changes to the cluster_name array
		 */

		if (hcheck) {
			// Apply changes to the cluster_name array
			applyClusterChanged<<<grid_x, block_x>>>(cluster_name_, cluster_list, cluster_location, point_num_);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());

			checkCudaErrors(cudaMemset(cluster_location, 0, sizeof(int) * (point_num_ + 1)));

			block_x2 = (cluster_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : cluster_num_;
			grid_x2 = (cluster_num_ - 1) / block_x2 + 1;

			// Remake the cluster location
			clusterMark<<<grid_x2, block_x2>>>(cluster_list, cluster_location, cluster_num_);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());

			int old_cluster_num = cluster_num_;

			exclusiveScan(cluster_location, point_num_ + 1, &cluster_num_);

			checkCudaErrors(cudaMalloc(&new_cluster_list, sizeof(int) * cluster_num_));

			clusterCollector<<<grid_x2, block_x2>>>(cluster_list, new_cluster_list, cluster_location, old_cluster_num);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());

			// Rebuild matrix
			int *new_matrix;

			checkCudaErrors(cudaMalloc(&new_matrix, sizeof(int) * cluster_num_ * cluster_num_));
			checkCudaErrors(cudaMemset(new_matrix, 0, sizeof(int) * cluster_num_ * cluster_num_));

			block_size.x = block_x2;
			block_size.y = block_size.z = 1;
			grid_size.x = grid_x2;
			grid_size.y = (old_cluster_num > GRID_SIZE_Y) ? GRID_SIZE_Y : old_cluster_num;
			grid_size.z = 1;

			//rebuildMatrix<<<grid_x2, block_x2>>>(matrix, cluster_list, new_matrix, cluster_location, old_cluster_num, cluster_num_);
			rebuildMatrix2<<<grid_x2, block_x2>>>(matrix, cluster_list, new_matrix, cluster_location, old_cluster_num, cluster_num_);
			checkCudaErrors(cudaGetLastError());
			checkCudaErrors(cudaDeviceSynchronize());


			checkCudaErrors(cudaFree(cluster_list));
			cluster_list = new_cluster_list;

			checkCudaErrors(cudaFree(matrix));
			matrix = new_matrix;
		}
	} while (hcheck);


	gettimeofday(&end, NULL);

	std::cout << "Iteration = " << timeDiff(start, end) << std::endl;

	renamingClusters(cluster_name_, cluster_location, point_num_);

	checkCudaErrors(cudaMemcpy(cluster_name_host_, cluster_name_, point_num_ * sizeof(int), cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaFree(matrix));
	checkCudaErrors(cudaFree(cluster_list));
	checkCudaErrors(cudaFree(cluster_location));
	checkCudaErrors(cudaFree(check));
	checkCudaErrors(cudaFree(changed_diag));
}
