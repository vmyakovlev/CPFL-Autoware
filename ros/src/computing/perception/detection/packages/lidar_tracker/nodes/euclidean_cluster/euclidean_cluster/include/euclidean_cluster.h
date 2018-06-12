#ifndef GPU_EUCLIDEAN_CLUSTER_H_
#define GPU_EUCLIDEAN_CLUSTER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <cuda.h>
#include <cuda_runtime.h>
#include <time.h>
#include <sys/time.h>

class GpuEuclideanCluster2 {
public:
	typedef struct {
		int index_value;
		std::vector<int> points_in_cluster;
	} GClusterIndex;

	GpuEuclideanCluster2();

	void setInputPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input);
	void setThreshold(double threshold);
	void setMinClusterPts(int min_cluster_pts);
	void setMaxClusterPts(int max_cluster_pts);
	// Use adjacent matrix
	void extractClusters();

	// Vertex-based: Use adjacent list and atomic operations
	void extractClusters2();

	// Edge-based: Use edge set and atomic oprations
	void extractClusters3();

	std::vector<GClusterIndex> getOutput();

	~GpuEuclideanCluster2();

private:
	void initClusters();

	void exclusiveScan(int *input, int ele_num, int *sum);

	void renamingClusters(int *cluster_names, int *cluster_location, int point_num);

	float *x_, *y_, *z_;
	int point_num_;
	double threshold_;
	int *cluster_name_;
	int *cluster_name_host_;
	int min_cluster_pts_;
	int max_cluster_pts_;
	int cluster_num_;
};

#ifndef BLOCK_SIZE_X
#define BLOCK_SIZE_X (1024)
#endif

#ifndef GRID_SIZE_Y
#define GRID_SIZE_Y (1024)
#endif

inline void gassert(cudaError_t err_code, const char *file, int line)
{
	if (err_code != cudaSuccess) {
		fprintf(stderr, "Error: %s %s %d\n", cudaGetErrorString(err_code), file, line);
		cudaDeviceReset();
		exit(EXIT_FAILURE);
	}
}

#define checkCudaErrors(err_code) gassert(err_code, __FILE__, __LINE__)

#ifndef timeDiff(start, end)
#define timeDiff(start, end) ((end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec))
#endif

#endif
