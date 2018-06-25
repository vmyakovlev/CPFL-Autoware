#include "ndt_gpu/NormalDistributionsTransform.h"
#include "ndt_gpu/common.h"
#include "ndt_gpu/debug.h"
#include <cmath>
#include <iostream>
#include <pcl/common/transforms.h>

namespace gpu {

template <typename eleType>
GNormalDistributionsTransform<eleType>::GNormalDistributionsTransform()
{
	//GRegistration::GRegistration();

	gauss_d1_ = gauss_d2_ = 0;
	outlier_ratio_ = 0.55;
	step_size_ = 0.1;
	resolution_ = 1.0f;
	trans_probability_ = 0;

	eleType gauss_c1, gauss_c2, gauss_d3;

	// Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
	gauss_c1 = 10.0 * (1 - outlier_ratio_);
	gauss_c2 = outlier_ratio_ / pow (resolution_, 3);
	gauss_d3 = -log (gauss_c2);
	gauss_d1_ = -log ( gauss_c1 + gauss_c2 ) - gauss_d3;
	gauss_d2_ = -2 * log ((-log ( gauss_c1 * exp ( -0.5 ) + gauss_c2 ) - gauss_d3) / gauss_d1_);

	transformation_epsilon_ = 0.1;
	max_iterations_ = 35;

	j_ang_ = MatrixHost<eleType>(24, 1);

	h_ang_ = MatrixHost<eleType>(45, 1);

	dj_ang_ = MatrixDevice<eleType>(24, 1);

	dh_ang_ = MatrixDevice<eleType>(45, 1);

	real_iterations_ = 0;
}

template <typename eleType>
GNormalDistributionsTransform<eleType>::GNormalDistributionsTransform(const GNormalDistributionsTransform<eleType> &other)
{
	gauss_d1_ = other.gauss_d1_;
	gauss_d2_ = other.gauss_d2_;

	outlier_ratio_ = other.outlier_ratio_;

	j_ang_ = other.j_ang_;
	h_ang_ = other.h_ang_;
	dj_ang_ = other.dj_ang_;
	dh_ang_ = other.dh_ang_;

	step_size_ = other.step_size_;
	resolution_ = other.resolution_;
	trans_probability_ = other.trans_probability_;
	real_iterations_ = other.real_iterations_;

	voxel_grid_ = other.voxel_grid_;
}

template <typename eleType>
GNormalDistributionsTransform<eleType>::~GNormalDistributionsTransform()
{
	dj_ang_.memFree();
	dh_ang_.memFree();

}

template <typename eleType>
void GNormalDistributionsTransform<eleType>::setStepSize(eleType step_size)
{
	step_size_ = step_size;
}

template <typename eleType>
void GNormalDistributionsTransform<eleType>::setResolution(float resolution)
{
	resolution_ = resolution;
}

template <typename eleType>
void GNormalDistributionsTransform<eleType>::setOutlierRatio(eleType olr)
{
	outlier_ratio_ = olr;
}

template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::getStepSize() const
{
	return step_size_;
}

template <typename eleType>
float GNormalDistributionsTransform<eleType>::getResolution() const
{
	return resolution_;
}

template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::getOutlierRatio() const
{
	return outlier_ratio_;
}

template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::getTransformationProbability() const
{
	return trans_probability_;
}

template <typename eleType>
int GNormalDistributionsTransform<eleType>::getRealIterations()
{
	 return real_iterations_;
}

template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::auxilaryFunction_PsiMT(eleType a, eleType f_a, eleType f_0, eleType g_0, eleType mu)
{
  return (f_a - f_0 - mu * g_0 * a);
}

template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::auxilaryFunction_dPsiMT(eleType g_a, eleType g_0, eleType mu)
{
  return (g_a - mu * g_0);
}

template <typename eleType>
void GNormalDistributionsTransform<eleType>::setInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
	// Copy input map data from the host memory to the GPU memory
	GRegistration<eleType>::setInputTarget(input);

	// Build the voxel grid
	if (target_points_number_ != 0) {
		voxel_grid_.setLeafSize(resolution_, resolution_, resolution_);
		voxel_grid_.setInput(target_x_, target_y_, target_z_, target_points_number_);
	}
}

template <typename eleType>
void GNormalDistributionsTransform<eleType>::setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	// Copy input map data from the host memory to the GPU memory
	GRegistration<eleType>::setInputTarget(input);

	// Build the voxel grid
	if (target_points_number_ != 0) {
		voxel_grid_.setLeafSize(resolution_, resolution_, resolution_);
		voxel_grid_.setInput(target_x_, target_y_, target_z_, target_points_number_);
	}
}

template <typename eleType>
void GNormalDistributionsTransform<eleType>::computeTransformation(const Eigen::Matrix<float, 4, 4> &guess)
{

	if (dj_ang_.isEmpty()) {
		dj_ang_.memAlloc();
	}

	if (dh_ang_.isEmpty()) {
		dh_ang_.memAlloc();
	}

	nr_iterations_ = 0;
	converged_ = false;

	eleType gauss_c1, gauss_c2, gauss_d3;

	gauss_c1 = 10 * ( 1 - outlier_ratio_);
	gauss_c2 = outlier_ratio_ / pow(resolution_, 3);
	gauss_d3 = - log(gauss_c2);
	gauss_d1_ = -log(gauss_c1 + gauss_c2) - gauss_d3;
	gauss_d2_ = -2 * log((-log(gauss_c1 * exp(-0.5) + gauss_c2) - gauss_d3) / gauss_d1_);

	if (guess != Eigen::Matrix4f::Identity()) {
		final_transformation_ = guess;

		transformPointCloud(x_, y_, z_, trans_x_, trans_y_, trans_z_, points_number_, guess);
	}

	Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
	eig_transformation.matrix() = final_transformation_;

	Eigen::Matrix<eleType, 6, 1> p, delta_p, score_gradient;
	Eigen::Vector3f init_translation = eig_transformation.translation();
	Eigen::Vector3f init_rotation = eig_transformation.rotation().eulerAngles(0, 1, 2);

	p << init_translation(0), init_translation(1), init_translation(2), init_rotation(0), init_rotation(1), init_rotation(2);

	Eigen::Matrix<eleType, 6, 6> hessian;

	eleType score = 0;
	eleType delta_p_norm;

	score = computeDerivatives(score_gradient, hessian, trans_x_, trans_y_, trans_z_, points_number_, p);

	int loop_time = 0;

	while (!converged_) {
		previous_transformation_ = transformation_;

		Eigen::JacobiSVD<Eigen::Matrix<eleType, 6, 6>> sv(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);

		delta_p = sv.solve(-score_gradient);

		delta_p_norm = delta_p.norm();

		if (delta_p_norm == 0 || delta_p_norm != delta_p_norm) {

			trans_probability_ = score / static_cast<eleType>(points_number_);
			converged_ = delta_p_norm == delta_p_norm;
			return;
		}

		delta_p.normalize();
		delta_p_norm = computeStepLengthMT(p, delta_p, delta_p_norm, step_size_, transformation_epsilon_ / 2, score, score_gradient, hessian, trans_x_, trans_y_, trans_z_, points_number_);

		delta_p *= delta_p_norm;

		Eigen::Translation<float, 3> translation(static_cast<float>(delta_p(0)), static_cast<float>(delta_p(1)), static_cast<float>(delta_p(2)));
		Eigen::AngleAxis<float> tmp1(static_cast<float>(delta_p(3)), Eigen::Vector3f::UnitX());
		Eigen::AngleAxis<float> tmp2(static_cast<float>(delta_p(4)), Eigen::Vector3f::UnitY());
		Eigen::AngleAxis<float> tmp3(static_cast<float>(delta_p(5)), Eigen::Vector3f::UnitZ());
		Eigen::AngleAxis<float> tmp4(tmp1 * tmp2 * tmp3);

		transformation_ = (translation * tmp4).matrix();

		p = p + delta_p;

		//Not update visualizer

		if (nr_iterations_ > max_iterations_ || (nr_iterations_ && (std::fabs(delta_p_norm) < transformation_epsilon_)))
			converged_ = true;

		nr_iterations_++;

		loop_time++;
	}

	trans_probability_ = score / static_cast<eleType>(points_number_);
}

/* First step of computing point gradients */
template <typename eleType>
__global__ void computePointGradients0(float *x, float *y, float *z, int points_num,
													int *valid_points, int valid_points_num,
													eleType *dj_ang,
													eleType *pg00, eleType *pg11, eleType *pg22,
													eleType *pg13, eleType *pg23, eleType *pg04, eleType *pg14)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ eleType j_ang[12];


	if (threadIdx.x < 12) {
		j_ang[threadIdx.x] = dj_ang[threadIdx.x];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		eleType o_x = static_cast<eleType>(x[pid]);
		eleType o_y = static_cast<eleType>(y[pid]);
		eleType o_z = static_cast<eleType>(z[pid]);

		//Set the 3x3 block start from (0, 0) to identity matrix
		pg00[i] = 1;
		pg11[i] = 1;
		pg22[i] = 1;

		//Compute point derivatives
		pg13[i] = o_x * j_ang[0] + o_y * j_ang[1] + o_z * j_ang[2];
		pg23[i] = o_x * j_ang[3] + o_y * j_ang[4] + o_z * j_ang[5];
		pg04[i] = o_x * j_ang[6] + o_y * j_ang[7] + o_z * j_ang[8];
		pg14[i] = o_x * j_ang[9] + o_y * j_ang[10] + o_z * j_ang[11];
	}
}

/* Float only */
template <>
__global__ void computePointGradients0<float>(float *x, float *y, float *z, int points_num,
													int *valid_points, int valid_points_num,
													float *dj_ang,
													float *pg00, float *pg11, float *pg22,
													float *pg13, float *pg23, float *pg04, float *pg14)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ float j_ang[12];


	if (threadIdx.x < 12) {
		j_ang[threadIdx.x] = dj_ang[threadIdx.x];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		float o_x = x[pid];
		float o_y = y[pid];
		float o_z = z[pid];

		//Set the 3x3 block start from (0, 0) to identity matrix
		pg00[i] = 1;
		pg11[i] = 1;
		pg22[i] = 1;

		//Compute point derivatives
		pg13[i] = o_x * j_ang[0] + o_y * j_ang[1] + o_z * j_ang[2];
		pg23[i] = o_x * j_ang[3] + o_y * j_ang[4] + o_z * j_ang[5];
		pg04[i] = o_x * j_ang[6] + o_y * j_ang[7] + o_z * j_ang[8];
		pg14[i] = o_x * j_ang[9] + o_y * j_ang[10] + o_z * j_ang[11];
	}
}

/* Second step of computing point gradients */
template <typename eleType>
__global__ void computePointGradients1(float *x, float *y, float *z, int points_num,
													int *valid_points, int valid_points_num,
													eleType *dj_ang,
													eleType *pg24, eleType *pg05, eleType *pg15, eleType *pg25)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ eleType j_ang[12];


	if (threadIdx.x < 12) {
		j_ang[threadIdx.x] = dj_ang[threadIdx.x + 12];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		eleType o_x = static_cast<eleType>(x[pid]);
		eleType o_y = static_cast<eleType>(y[pid]);
		eleType o_z = static_cast<eleType>(z[pid]);

		//Compute point derivatives

		pg24[i] = o_x * j_ang[0] + o_y * j_ang[1] + o_z * j_ang[2];
		pg05[i] = o_x * j_ang[3] + o_y * j_ang[4] + o_z * j_ang[5];
		pg15[i] = o_x * j_ang[6] + o_y * j_ang[7] + o_z * j_ang[8];
		pg25[i] = o_x * j_ang[9] + o_y * j_ang[10] + o_z * j_ang[11];
	}
}

/* Float only */
template <>
__global__ void computePointGradients1<float>(float *x, float *y, float *z, int points_num,
													int *valid_points, int valid_points_num,
													float *dj_ang,
													float *pg24, float *pg05, float *pg15, float *pg25)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ float j_ang[12];


	if (threadIdx.x < 12) {
		j_ang[threadIdx.x] = dj_ang[threadIdx.x + 12];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		float o_x = x[pid];
		float o_y = y[pid];
		float o_z = z[pid];

		//Compute point derivatives

		pg24[i] = o_x * j_ang[0] + o_y * j_ang[1] + o_z * j_ang[2];
		pg05[i] = o_x * j_ang[3] + o_y * j_ang[4] + o_z * j_ang[5];
		pg15[i] = o_x * j_ang[6] + o_y * j_ang[7] + o_z * j_ang[8];
		pg25[i] = o_x * j_ang[9] + o_y * j_ang[10] + o_z * j_ang[11];
	}
}


/* First step of computing point hessians */
template <typename eleType>
__global__ void computePointHessian0(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												eleType *dh_ang,
												eleType *ph93, eleType *ph103, eleType *ph113,
												eleType *ph123, eleType *ph94, eleType *ph133,
												eleType *ph104, eleType *ph143, eleType *ph114,
												eleType *ph153, eleType *ph95, eleType *ph163,
												eleType *ph105, eleType *ph173, eleType *ph115)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ eleType h_ang[18];

	if (threadIdx.x < 18) {
		h_ang[threadIdx.x] = dh_ang[threadIdx.x];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		eleType o_x = static_cast<eleType>(x[pid]);
		eleType o_y = static_cast<eleType>(y[pid]);
		eleType o_z = static_cast<eleType>(z[pid]);


		ph93[i] = 0;
		ph103[i] = o_x * h_ang[0] + o_y * h_ang[1] + o_z * h_ang[2];
		ph113[i] = o_x * h_ang[3] + o_y * h_ang[4] + o_z * h_ang[5];

		ph123[i] = ph94[i] = 0;
		ph133[i] = ph104[i] = o_x * h_ang[6] + o_y * h_ang[7] + o_z * h_ang[8];
		ph143[i] = ph114[i] = o_x * h_ang[9] + o_y * h_ang[10] + o_z * h_ang[11];

		ph153[i] = ph95[i] = 0;
		ph163[i] = ph105[i] = o_x * h_ang[12] + o_y * h_ang[13] + o_z * h_ang[14];
		ph173[i] = ph115[i] = o_x * h_ang[15] + o_y * h_ang[16] + o_z * h_ang[17];

	}
}

/* Float only*/
template <>
__global__ void computePointHessian0<float>(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												float *dh_ang,
												float *ph93, float *ph103, float *ph113,
												float *ph123, float *ph94, float *ph133,
												float *ph104, float *ph143, float *ph114,
												float *ph153, float *ph95, float *ph163,
												float *ph105, float *ph173, float *ph115)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ float h_ang[18];

	if (threadIdx.x < 18) {
		h_ang[threadIdx.x] = dh_ang[threadIdx.x];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		float o_x = x[pid];
		float o_y = y[pid];
		float o_z = z[pid];


		ph93[i] = 0;
		ph103[i] = o_x * h_ang[0] + o_y * h_ang[1] + o_z * h_ang[2];
		ph113[i] = o_x * h_ang[3] + o_y * h_ang[4] + o_z * h_ang[5];

		ph123[i] = ph94[i] = 0;
		ph133[i] = ph104[i] = o_x * h_ang[6] + o_y * h_ang[7] + o_z * h_ang[8];
		ph143[i] = ph114[i] = o_x * h_ang[9] + o_y * h_ang[10] + o_z * h_ang[11];

		ph153[i] = ph95[i] = 0;
		ph163[i] = ph105[i] = o_x * h_ang[12] + o_y * h_ang[13] + o_z * h_ang[14];
		ph173[i] = ph115[i] = o_x * h_ang[15] + o_y * h_ang[16] + o_z * h_ang[17];

	}
}


template <typename eleType>
__global__ void computePointHessian1(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												eleType *dh_ang,
												eleType *ph124, eleType *ph134, eleType *ph144,
												eleType *ph154, eleType *ph125, eleType *ph164,
												eleType *ph135, eleType *ph174, eleType *ph145)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ eleType h_ang[18];

	if (threadIdx.x < 18) {
		h_ang[threadIdx.x] = dh_ang[18 + threadIdx.x];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		eleType o_x = static_cast<eleType>(x[pid]);
		eleType o_y = static_cast<eleType>(y[pid]);
		eleType o_z = static_cast<eleType>(z[pid]);

		ph124[i] = o_x * h_ang[0] + o_y * h_ang[1] + o_z * h_ang[2];
		ph134[i] = o_x * h_ang[3] + o_y * h_ang[4] + o_z * h_ang[5];
		ph144[i] = o_x * h_ang[6] + o_y * h_ang[7] + o_z * h_ang[8];

		ph154[i] = ph125[i] = o_x * h_ang[9] + o_y * h_ang[10] + o_z * h_ang[11];
		ph164[i] = ph135[i] = o_x * h_ang[12] + o_y * h_ang[13] + o_z * h_ang[14];
		ph174[i] = ph145[i] = o_x * h_ang[15] + o_y * h_ang[16] + o_z * h_ang[17];
	}
}

template <>
__global__ void computePointHessian1<float>(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												float *dh_ang,
												float *ph124, float *ph134, float *ph144,
												float *ph154, float *ph125, float *ph164,
												float *ph135, float *ph174, float *ph145)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ float h_ang[18];

	if (threadIdx.x < 18) {
		h_ang[threadIdx.x] = dh_ang[18 + threadIdx.x];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		float o_x = x[pid];
		float o_y = y[pid];
		float o_z = z[pid];

		ph124[i] = o_x * h_ang[0] + o_y * h_ang[1] + o_z * h_ang[2];
		ph134[i] = o_x * h_ang[3] + o_y * h_ang[4] + o_z * h_ang[5];
		ph144[i] = o_x * h_ang[6] + o_y * h_ang[7] + o_z * h_ang[8];

		ph154[i] = ph125[i] = o_x * h_ang[9] + o_y * h_ang[10] + o_z * h_ang[11];
		ph164[i] = ph135[i] = o_x * h_ang[12] + o_y * h_ang[13] + o_z * h_ang[14];
		ph174[i] = ph145[i] = o_x * h_ang[15] + o_y * h_ang[16] + o_z * h_ang[17];
	}
}

template <typename eleType>
__global__ void computePointHessian2(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												eleType *dh_ang,
												eleType *ph155, eleType *ph165, eleType *ph175)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ eleType h_ang[9];

	if (threadIdx.x < 9) {
		h_ang[threadIdx.x] = dh_ang[36 + threadIdx.x];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		eleType o_x = static_cast<eleType>(x[pid]);
		eleType o_y = static_cast<eleType>(y[pid]);
		eleType o_z = static_cast<eleType>(z[pid]);

		ph155[i] = o_x * h_ang[0] + o_y * h_ang[1] + o_z * h_ang[2];
		ph165[i] = o_x * h_ang[3] + o_y * h_ang[4] + o_z * h_ang[5];
		ph175[i] = o_x * h_ang[6] + o_y * h_ang[7] + o_z * h_ang[8];

	}
}

template <>
__global__ void computePointHessian2<float>(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												float *dh_ang,
												float *ph155, float *ph165, float *ph175)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	__shared__ float h_ang[9];

	if (threadIdx.x < 9) {
		h_ang[threadIdx.x] = dh_ang[36 + threadIdx.x];
	}

	__syncthreads();

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		float o_x = x[pid];
		float o_y = y[pid];
		float o_z = z[pid];

		ph155[i] = o_x * h_ang[0] + o_y * h_ang[1] + o_z * h_ang[2];
		ph165[i] = o_x * h_ang[3] + o_y * h_ang[4] + o_z * h_ang[5];
		ph175[i] = o_x * h_ang[6] + o_y * h_ang[7] + o_z * h_ang[8];

	}
}

/* compute score_inc list for input points.
 * The final score_inc is calculated by a reduction sum
 * on this score_inc list. */
template <typename eleType>
__global__ void computeScoreList(int *starting_voxel_id, int *voxel_id, int valid_points_num,
												eleType *e_x_cov_x, eleType gauss_d1, eleType *score)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {

		eleType score_inc = 0;

		for (int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			eleType tmp_ex = e_x_cov_x[vid];

			score_inc += (tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex) ? 0 : -gauss_d1 * tmp_ex;
		}

		score[i] = score_inc;
	}
}

/* First step to compute score gradient list for input points */
template <typename eleType>
__global__ void computeScoreGradientList(float *trans_x, float *trans_y, float *trans_z,
														int *valid_points,
														int *starting_voxel_id, int *voxel_id, int valid_points_num,
														eleType *centroid_x, eleType *centroid_y, eleType *centroid_z,
														int voxel_num, eleType *e_x_cov_x,
														eleType *cov_dxd_pi, eleType gauss_d1, int valid_voxel_num,
														eleType *score_gradients)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int col = blockIdx.y;

	if (col < 6) {
		eleType *sg = score_gradients + col * valid_points_num;
		eleType *cov_dxd_pi_mat0 = cov_dxd_pi + col * valid_voxel_num;
		eleType *cov_dxd_pi_mat1 = cov_dxd_pi_mat0 + 6 * valid_voxel_num;
		eleType *cov_dxd_pi_mat2 = cov_dxd_pi_mat1 + 6 * valid_voxel_num;

		for (int i = id; i < valid_points_num; i += stride) {
			int pid = valid_points[i];
			eleType d_x = static_cast<eleType>(trans_x[pid]);
			eleType d_y = static_cast<eleType>(trans_y[pid]);
			eleType d_z = static_cast<eleType>(trans_z[pid]);

			eleType tmp_sg = 0.0;

			for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
				int vid = voxel_id[j];
				eleType tmp_ex = e_x_cov_x[j];

				if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
					tmp_ex *= gauss_d1;

					tmp_sg += ((d_x - centroid_x[vid]) * cov_dxd_pi_mat0[j] + (d_y - centroid_y[vid]) * cov_dxd_pi_mat1[j] + (d_z - centroid_z[vid]) * cov_dxd_pi_mat2[j]) * tmp_ex;
				}
			}

			sg[i] = tmp_sg;
		}
	}
}

template <>
__global__ void computeScoreGradientList<float>(float *trans_x, float *trans_y, float *trans_z,
														int *valid_points,
														int *starting_voxel_id, int *voxel_id, int valid_points_num,
														float *centroid_x, float *centroid_y, float *centroid_z,
														int voxel_num, float *e_x_cov_x,
														float *cov_dxd_pi, float gauss_d1, int valid_voxel_num,
														float *score_gradients)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int col = blockIdx.y;

	if (col < 6) {
		float *sg = score_gradients + col * valid_points_num;
		float *cov_dxd_pi_mat0 = cov_dxd_pi + col * valid_voxel_num;
		float *cov_dxd_pi_mat1 = cov_dxd_pi_mat0 + 6 * valid_voxel_num;
		float *cov_dxd_pi_mat2 = cov_dxd_pi_mat1 + 6 * valid_voxel_num;

		for (int i = id; i < valid_points_num; i += stride) {
			int pid = valid_points[i];
			float d_x = trans_x[pid];
			float d_y = trans_y[pid];
			float d_z = trans_z[pid];

			float tmp_sg = 0.0;

			for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
				int vid = voxel_id[j];
				float tmp_ex = e_x_cov_x[j];

				if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
					tmp_ex *= gauss_d1;

					tmp_sg += ((d_x - centroid_x[vid]) * cov_dxd_pi_mat0[j] + (d_y - centroid_y[vid]) * cov_dxd_pi_mat1[j] + (d_z - centroid_z[vid]) * cov_dxd_pi_mat2[j]) * tmp_ex;
				}
			}

			sg[i] = tmp_sg;
		}
	}
}

/* Intermediate step to compute e_x_cov_x */
template <typename eleType>
__global__ void computeExCovX(float *trans_x, float *trans_y, float *trans_z, int *valid_points,
											int *starting_voxel_id, int *voxel_id, int valid_points_num,
											eleType *centr_x, eleType *centr_y, eleType *centr_z,
											eleType gauss_d1, eleType gauss_d2,
											eleType *e_x_cov_x,
											eleType *icov00, eleType *icov01, eleType *icov02,
											eleType *icov10, eleType *icov11, eleType *icov12,
											eleType *icov20, eleType *icov21, eleType *icov22)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];
		eleType d_x = static_cast<eleType>(trans_x[pid]);
		eleType d_y = static_cast<eleType>(trans_y[pid]);
		eleType d_z = static_cast<eleType>(trans_z[pid]);
		eleType t_x, t_y, t_z;


		for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
			int vid = voxel_id[j];

			t_x = d_x - centr_x[vid];
			t_y = d_y - centr_y[vid];
			t_z = d_z - centr_z[vid];

			e_x_cov_x[j] =  exp(-gauss_d2 * ((t_x * icov00[vid] + t_y * icov01[vid] + t_z * icov02[vid]) * t_x
										+ ((t_x * icov10[vid] + t_y * icov11[vid] + t_z * icov12[vid]) * t_y)
										+ ((t_x * icov20[vid] + t_y * icov21[vid] + t_z * icov22[vid]) * t_z)) / 2.0);
		}
	}
}

template <>
__global__ void computeExCovX<float>(float *trans_x, float *trans_y, float *trans_z, int *valid_points,
											int *starting_voxel_id, int *voxel_id, int valid_points_num,
											float *centr_x, float *centr_y, float *centr_z,
											float gauss_d1, float gauss_d2,
											float *e_x_cov_x,
											float *icov00, float *icov01, float *icov02,
											float *icov10, float *icov11, float *icov12,
											float *icov20, float *icov21, float *icov22)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];
		float d_x = trans_x[pid];
		float d_y = trans_y[pid];
		float d_z = trans_z[pid];
		float t_x, t_y, t_z;


		for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
			int vid = voxel_id[j];

			t_x = d_x - centr_x[vid];
			t_y = d_y - centr_y[vid];
			t_z = d_z - centr_z[vid];

			e_x_cov_x[j] =  exp(-gauss_d2 * ((t_x * icov00[vid] + t_y * icov01[vid] + t_z * icov02[vid]) * t_x
										+ ((t_x * icov10[vid] + t_y * icov11[vid] + t_z * icov12[vid]) * t_y)
										+ ((t_x * icov20[vid] + t_y * icov21[vid] + t_z * icov22[vid]) * t_z)) / 2.0);
		}
	}
}

/* update e_x_cov_x - Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009] */
template <typename eleType>
__global__ void updateExCovX(eleType *e_x_cov_x, eleType gauss_d2, int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_voxel_num; i += stride) {
		e_x_cov_x[i] *= gauss_d2;
	}
}

/* compute cov_dxd_pi as reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]*/
template <typename eleType>
__global__ void computeCovDxdPi(int *valid_points, int *starting_voxel_id, int *voxel_id, int valid_points_num,
											eleType *inverse_covariance, int voxel_num,
											eleType gauss_d1, eleType gauss_d2, eleType *point_gradients,
											eleType *cov_dxd_pi, int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	if (row < 3 && col < 6) {
		eleType *icov0 = inverse_covariance + row * 3 * voxel_num;
		eleType *icov1 = icov0 + voxel_num;
		eleType *icov2 = icov1 + voxel_num;
		eleType *cov_dxd_pi_tmp = cov_dxd_pi + (row * 6 + col) * valid_voxel_num;
		eleType *pg_tmp0 = point_gradients + col * valid_points_num;
		eleType *pg_tmp1 = pg_tmp0 + 6 * valid_points_num;
		eleType *pg_tmp2 = pg_tmp1 + 6 * valid_points_num;

		for (int i = id; i < valid_points_num; i += stride) {
			eleType pg0 = pg_tmp0[i];
			eleType pg1 = pg_tmp1[i];
			eleType pg2 = pg_tmp2[i];

			for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
				int vid = voxel_id[j];

				cov_dxd_pi_tmp[j] = icov0[vid] * pg0 + icov1[vid] * pg1 + icov2[vid] * pg2;
			}
		}
	}
}


/* First step to compute hessian list for input points */
template <typename eleType>
__global__ void computeHessianListS0(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													eleType *centroid_x, eleType *centroid_y, eleType *centroid_z,
													eleType *icov00, eleType *icov01, eleType *icov02,
													eleType *icov10, eleType *icov11, eleType *icov12,
													eleType *icov20, eleType *icov21, eleType *icov22,
													eleType *point_gradients,
													eleType *tmp_hessian,
													int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int col = blockIdx.y;

	if (col < 6) {
		eleType *tmp_pg0 = point_gradients + col * valid_points_num;
		eleType *tmp_pg1 = tmp_pg0 + 6 * valid_points_num;
		eleType *tmp_pg2 = tmp_pg1 + 6 * valid_points_num;
		eleType *tmp_h = tmp_hessian + col * valid_voxel_num;

		for (int i = id; i < valid_points_num; i += stride) {
			int pid = valid_points[i];
			eleType d_x = static_cast<eleType>(trans_x[pid]);
			eleType d_y = static_cast<eleType>(trans_y[pid]);
			eleType d_z = static_cast<eleType>(trans_z[pid]);

			eleType pg0 = tmp_pg0[i];
			eleType pg1 = tmp_pg1[i];
			eleType pg2 = tmp_pg2[i];

			for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
				int vid = voxel_id[j];

				tmp_h[j] = (d_x - centroid_x[vid]) * (icov00[vid] * pg0 + icov01[vid] * pg1 + icov02[vid] * pg2)
							+ (d_y - centroid_y[vid]) * (icov10[vid] * pg0 + icov11[vid] * pg1 + icov12[vid] * pg2)
							+ (d_z - centroid_z[vid]) * (icov20[vid] * pg0 + icov21[vid] * pg1 + icov22[vid] * pg2);
			}
		}
	}
}

template <>
__global__ void computeHessianListS0<float>(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													float *centroid_x, float *centroid_y, float *centroid_z,
													float *icov00, float *icov01, float *icov02,
													float *icov10, float *icov11, float *icov12,
													float *icov20, float *icov21, float *icov22,
													float *point_gradients,
													float *tmp_hessian,
													int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int col = blockIdx.y;

	if (col < 6) {
		float *tmp_pg0 = point_gradients + col * valid_points_num;
		float *tmp_pg1 = tmp_pg0 + 6 * valid_points_num;
		float *tmp_pg2 = tmp_pg1 + 6 * valid_points_num;
		float *tmp_h = tmp_hessian + col * valid_voxel_num;

		for (int i = id; i < valid_points_num; i += stride) {
			int pid = valid_points[i];
			float d_x = trans_x[pid];
			float d_y = trans_y[pid];
			float d_z = trans_z[pid];

			float pg0 = tmp_pg0[i];
			float pg1 = tmp_pg1[i];
			float pg2 = tmp_pg2[i];

			for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
				int vid = voxel_id[j];

				tmp_h[j] = (d_x - centroid_x[vid]) * (icov00[vid] * pg0 + icov01[vid] * pg1 + icov02[vid] * pg2)
							+ (d_y - centroid_y[vid]) * (icov10[vid] * pg0 + icov11[vid] * pg1 + icov12[vid] * pg2)
							+ (d_z - centroid_z[vid]) * (icov20[vid] * pg0 + icov21[vid] * pg1 + icov22[vid] * pg2);
			}
		}
	}
}

/* Fourth step to compute hessian list */
template <typename eleType>
__global__ void computeHessianListS1(float *trans_x, float *trans_y, float *trans_z,
												int *valid_points,
												int *starting_voxel_id, int *voxel_id, int valid_points_num,
												eleType *centroid_x, eleType *centroid_y, eleType *centroid_z,
												eleType gauss_d1, eleType gauss_d2, eleType *hessians,
												eleType *e_x_cov_x, eleType *tmp_hessian, eleType *cov_dxd_pi,
												eleType *point_gradients,
												int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	if (row < 6 && col < 6) {
		eleType *cov_dxd_pi_mat0 = cov_dxd_pi + row * valid_voxel_num;
		eleType *cov_dxd_pi_mat1 = cov_dxd_pi_mat0 + 6 * valid_voxel_num;
		eleType *cov_dxd_pi_mat2 = cov_dxd_pi_mat1 + 6 * valid_voxel_num;
		eleType *tmp_h = tmp_hessian + col * valid_voxel_num;
		eleType *h = hessians + (row * 6 + col) * valid_points_num;
		eleType *tmp_pg0 = point_gradients + col * valid_points_num;
		eleType *tmp_pg1 = tmp_pg0 + 6 * valid_points_num;
		eleType *tmp_pg2 = tmp_pg1 + 6 * valid_points_num;

		for (int i = id; i < valid_points_num; i += stride) {
			int pid = valid_points[i];
			eleType d_x = static_cast<eleType>(trans_x[pid]);
			eleType d_y = static_cast<eleType>(trans_y[pid]);
			eleType d_z = static_cast<eleType>(trans_z[pid]);

			eleType pg0 = tmp_pg0[i];
			eleType pg1 = tmp_pg1[i];
			eleType pg2 = tmp_pg2[i];

			eleType final_hessian = 0.0;

			for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
				//Transformed coordinates
				int vid = voxel_id[j];

				eleType tmp_ex = e_x_cov_x[j];

				if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
					eleType cov_dxd0 = cov_dxd_pi_mat0[j];
					eleType cov_dxd1 = cov_dxd_pi_mat1[j];
					eleType cov_dxd2 = cov_dxd_pi_mat2[j];

					tmp_ex *= gauss_d1;

					final_hessian += -gauss_d2 * ((d_x - centroid_x[vid]) * cov_dxd0 + (d_y - centroid_y[vid]) * cov_dxd1 + (d_z - centroid_z[vid]) * cov_dxd2) * tmp_h[j] * tmp_ex;
					final_hessian += (pg0 * cov_dxd0 + pg1 * cov_dxd1 + pg2 * cov_dxd2) * tmp_ex;
				}
			}

			h[i] = final_hessian;
		}
	}
}

template <>
__global__ void computeHessianListS1<float>(float *trans_x, float *trans_y, float *trans_z,
												int *valid_points,
												int *starting_voxel_id, int *voxel_id, int valid_points_num,
												float *centroid_x, float *centroid_y, float *centroid_z,
												float gauss_d1, float gauss_d2, float *hessians,
												float *e_x_cov_x, float *tmp_hessian, float *cov_dxd_pi,
												float *point_gradients,
												int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	if (row < 6 && col < 6) {
		float *cov_dxd_pi_mat0 = cov_dxd_pi + row * valid_voxel_num;
		float *cov_dxd_pi_mat1 = cov_dxd_pi_mat0 + 6 * valid_voxel_num;
		float *cov_dxd_pi_mat2 = cov_dxd_pi_mat1 + 6 * valid_voxel_num;
		float *tmp_h = tmp_hessian + col * valid_voxel_num;
		float *h = hessians + (row * 6 + col) * valid_points_num;
		float *tmp_pg0 = point_gradients + col * valid_points_num;
		float *tmp_pg1 = tmp_pg0 + 6 * valid_points_num;
		float *tmp_pg2 = tmp_pg1 + 6 * valid_points_num;

		for (int i = id; i < valid_points_num; i += stride) {
			int pid = valid_points[i];
			float d_x = trans_x[pid];
			float d_y = trans_y[pid];
			float d_z = trans_z[pid];

			float pg0 = tmp_pg0[i];
			float pg1 = tmp_pg1[i];
			float pg2 = tmp_pg2[i];

			float final_hessian = 0.0;

			for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
				//Transformed coordinates
				int vid = voxel_id[j];

				float tmp_ex = e_x_cov_x[j];

				if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
					float cov_dxd0 = cov_dxd_pi_mat0[j];
					float cov_dxd1 = cov_dxd_pi_mat1[j];
					float cov_dxd2 = cov_dxd_pi_mat2[j];

					tmp_ex *= gauss_d1;

					final_hessian += -gauss_d2 * ((d_x - centroid_x[vid]) * cov_dxd0 + (d_y - centroid_y[vid]) * cov_dxd1 + (d_z - centroid_z[vid]) * cov_dxd2) * tmp_h[j] * tmp_ex;
					final_hessian += (pg0 * cov_dxd0 + pg1 * cov_dxd1 + pg2 * cov_dxd2) * tmp_ex;
				}
			}

			h[i] = final_hessian;
		}
	}
}

template <typename eleType>
__global__ void computeHessianListS2(float *trans_x, float *trans_y, float *trans_z,
												int *valid_points,
												int *starting_voxel_id, int *voxel_id, int valid_points_num,
												eleType *centroid_x, eleType *centroid_y, eleType *centroid_z,
												eleType gauss_d1, eleType *e_x_cov_x,
												eleType *icov00, eleType *icov01, eleType *icov02,
												eleType *icov10, eleType *icov11, eleType *icov12,
												eleType *icov20, eleType *icov21, eleType *icov22,
												eleType *point_hessians, eleType *hessians,
												int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	if (row < 6 && col < 6) {
		eleType *h = hessians + (row * 6 + col) * valid_points_num;
		eleType *tmp_ph0 = point_hessians + ((3 * row) * 6 + col) * valid_points_num;
		eleType *tmp_ph1 = tmp_ph0 + 6 * valid_points_num;
		eleType *tmp_ph2 = tmp_ph1 + 6 * valid_points_num;

		for (int i = id; i < valid_points_num; i += stride) {
			int pid = valid_points[i];
			eleType d_x = static_cast<eleType>(trans_x[pid]);
			eleType d_y = static_cast<eleType>(trans_y[pid]);
			eleType d_z = static_cast<eleType>(trans_z[pid]);
			eleType ph0 = tmp_ph0[i];
			eleType ph1 = tmp_ph1[i];
			eleType ph2 = tmp_ph2[i];

			eleType final_hessian = h[i];

			for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
				//Transformed coordinates
				int vid = voxel_id[j];
				eleType tmp_ex = e_x_cov_x[j];

				if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
					tmp_ex *= gauss_d1;

					final_hessian += (d_x - centroid_x[vid]) * (icov00[vid] * ph0 + icov01[vid] * ph1 + icov02[vid] * ph2) * tmp_ex;
					final_hessian += (d_y - centroid_y[vid]) * (icov10[vid] * ph0 + icov11[vid] * ph1 + icov12[vid] * ph2) * tmp_ex;
					final_hessian += (d_z - centroid_z[vid]) * (icov20[vid] * ph0 + icov21[vid] * ph1 + icov22[vid] * ph2) * tmp_ex;

				}
			}

			h[i] = final_hessian;
		}
	}
}

template <>
__global__ void computeHessianListS2<float>(float *trans_x, float *trans_y, float *trans_z,
												int *valid_points,
												int *starting_voxel_id, int *voxel_id, int valid_points_num,
												float *centroid_x, float *centroid_y, float *centroid_z,
												float gauss_d1, float *e_x_cov_x,
												float *icov00, float *icov01, float *icov02,
												float *icov10, float *icov11, float *icov12,
												float *icov20, float *icov21, float *icov22,
												float *point_hessians, float *hessians,
												int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	if (row < 6 && col < 6) {
		float *h = hessians + (row * 6 + col) * valid_points_num;
		float *tmp_ph0 = point_hessians + ((3 * row) * 6 + col) * valid_points_num;
		float *tmp_ph1 = tmp_ph0 + 6 * valid_points_num;
		float *tmp_ph2 = tmp_ph1 + 6 * valid_points_num;

		for (int i = id; i < valid_points_num; i += stride) {
			int pid = valid_points[i];
			float d_x = trans_x[pid];
			float d_y = trans_y[pid];
			float d_z = trans_z[pid];
			float ph0 = tmp_ph0[i];
			float ph1 = tmp_ph1[i];
			float ph2 = tmp_ph2[i];

			float final_hessian = h[i];

			for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
				//Transformed coordinates
				int vid = voxel_id[j];
				float tmp_ex = e_x_cov_x[j];

				if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
					tmp_ex *= gauss_d1;

					final_hessian += (d_x - centroid_x[vid]) * (icov00[vid] * ph0 + icov01[vid] * ph1 + icov02[vid] * ph2) * tmp_ex;
					final_hessian += (d_y - centroid_y[vid]) * (icov10[vid] * ph0 + icov11[vid] * ph1 + icov12[vid] * ph2) * tmp_ex;
					final_hessian += (d_z - centroid_z[vid]) * (icov20[vid] * ph0 + icov21[vid] * ph1 + icov22[vid] * ph2) * tmp_ex;

				}
			}

			h[i] = final_hessian;
		}
	}
}

/* Compute sum of a list of matrices */
template <typename eleType>
__global__ void matrixSum(eleType *matrix_list, int full_size, int half_size, int rows, int cols, int offset)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	for (int i = index; i < half_size && row < rows && col < cols; i += stride) {
		MatrixDevice<eleType> left(rows, cols, offset, matrix_list + i);
		eleType *right_ptr = (i + half_size < full_size) ? matrix_list + i + half_size : NULL;
		MatrixDevice<eleType> right(rows, cols, offset, right_ptr);

		if (right_ptr != NULL) {
			left(row, col) += right(row, col);
		}
	}
}

/* Compute sum of score_inc list */
template <typename eleType>
__global__ void sumScore(eleType *score, int full_size, int half_size)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = index; i < half_size; i += stride) {
		score[i] += (i + half_size < full_size) ? score[i + half_size] : 0;
	}
}

template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::computeDerivatives(Eigen::Matrix<eleType, 6, 1> &score_gradient, Eigen::Matrix<eleType, 6, 6> &hessian,
																	float *trans_x, float *trans_y, float *trans_z,
																	int points_num, Eigen::Matrix<eleType, 6, 1> pose, bool compute_hessian)
{
	MatrixHost<eleType> p(6, 1);

	for (int i = 0; i < 6; i++) {
		p(i) = pose(i, 0);
	}

	score_gradient.setZero();
	hessian.setZero();

	//Compute Angle Derivatives
	computeAngleDerivatives(p);

	//Radius Search
	int *valid_points, *voxel_id, *starting_voxel_id;
	int valid_voxel_num, valid_points_num;

	valid_points = voxel_id = starting_voxel_id = NULL;

	voxel_grid_.radiusSearch(trans_x, trans_y, trans_z, points_num, resolution_, INT_MAX, &valid_points, &starting_voxel_id, &voxel_id, &valid_voxel_num, &valid_points_num);

	eleType *covariance = voxel_grid_.getCovarianceList();
	eleType *inverse_covariance = voxel_grid_.getInverseCovarianceList();
	eleType *centroid = voxel_grid_.getCentroidList();
	int *points_per_voxel = voxel_grid_.getPointsPerVoxelList();
	int voxel_num = voxel_grid_.getVoxelNum();

	if (valid_points_num == 0)
		return 0;

	//Update score gradient and hessian matrix

	eleType *gradients, *hessians, *point_gradients, *point_hessians, *score;

	checkCudaErrors(cudaMalloc(&gradients, sizeof(eleType) * valid_points_num * 6));
	checkCudaErrors(cudaMalloc(&hessians, sizeof(eleType) * valid_points_num * 6 * 6));
	checkCudaErrors(cudaMalloc(&point_gradients, sizeof(eleType) * valid_points_num * 3 * 6));
	checkCudaErrors(cudaMalloc(&point_hessians, sizeof(eleType) * valid_points_num * 18 * 6));
	checkCudaErrors(cudaMalloc(&score, sizeof(eleType) * valid_points_num));

	checkCudaErrors(cudaMemset(gradients, 0, sizeof(eleType) * valid_points_num * 6));
	checkCudaErrors(cudaMemset(hessians, 0, sizeof(eleType) * valid_points_num * 6 * 6));
	checkCudaErrors(cudaMemset(point_gradients, 0, sizeof(eleType) * valid_points_num * 3 * 6));
	checkCudaErrors(cudaMemset(point_hessians, 0, sizeof(eleType) * valid_points_num * 18 * 6));

	int block_x = (valid_points_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : valid_points_num;

	int grid_x = (valid_points_num - 1) / block_x + 1;

	dim3 grid;

	computePointGradients0<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dj_ang_.buffer(),
												point_gradients,
												point_gradients + valid_points_num * 7,
												point_gradients + valid_points_num * 14,
												point_gradients + valid_points_num * 9,
												point_gradients + valid_points_num * 15,
												point_gradients + valid_points_num * 4,
												point_gradients + valid_points_num * 10);
	checkCudaErrors(cudaGetLastError());

	computePointGradients1<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dj_ang_.buffer(),
												point_gradients + valid_points_num * 16,
												point_gradients + valid_points_num * 5,
												point_gradients + valid_points_num * 11,
												point_gradients + valid_points_num * 17);
	checkCudaErrors(cudaGetLastError());

	if (compute_hessian) {
		computePointHessian0<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_.buffer(),
												point_hessians + valid_points_num * 57, point_hessians + valid_points_num * 63, point_hessians + valid_points_num * 69,
												point_hessians + valid_points_num * 75, point_hessians + valid_points_num * 58, point_hessians + valid_points_num * 81,
												point_hessians + valid_points_num * 64, point_hessians + valid_points_num * 87, point_hessians + valid_points_num * 70,
												point_hessians + valid_points_num * 93, point_hessians + valid_points_num * 59, point_hessians + valid_points_num * 99,
												point_hessians + valid_points_num * 65, point_hessians + valid_points_num * 105, point_hessians + valid_points_num * 71);

		checkCudaErrors(cudaGetLastError());

		computePointHessian1<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_.buffer(),
												point_hessians + valid_points_num * 76, point_hessians + valid_points_num * 82, point_hessians + valid_points_num * 88,
												point_hessians + valid_points_num * 94, point_hessians + valid_points_num * 77, point_hessians + valid_points_num * 100,
												point_hessians + valid_points_num * 83, point_hessians + valid_points_num * 106, point_hessians + valid_points_num * 89);
		checkCudaErrors(cudaGetLastError());

		computePointHessian2<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_.buffer(),
												point_hessians + valid_points_num * 95, point_hessians + valid_points_num * 101, point_hessians + valid_points_num * 107);
		checkCudaErrors(cudaGetLastError());

	}

	checkCudaErrors(cudaDeviceSynchronize());


	eleType *tmp_hessian;

	checkCudaErrors(cudaMalloc(&tmp_hessian, sizeof(eleType) * valid_voxel_num * 6));

	eleType *e_x_cov_x;

	checkCudaErrors(cudaMalloc(&e_x_cov_x, sizeof(eleType) * valid_voxel_num));

	eleType *cov_dxd_pi;

	checkCudaErrors(cudaMalloc(&cov_dxd_pi, sizeof(eleType) * valid_voxel_num * 3 * 6));

	computeExCovX<eleType><<<grid_x, block_x>>>(trans_x, trans_y, trans_z, valid_points,
										starting_voxel_id, voxel_id, valid_points_num,
										centroid, centroid + voxel_num, centroid + 2 * voxel_num,
										gauss_d1_, gauss_d2_,
										e_x_cov_x,
										inverse_covariance, inverse_covariance + voxel_num, inverse_covariance + 2 * voxel_num,
										inverse_covariance + 3 * voxel_num, inverse_covariance + 4 * voxel_num, inverse_covariance + 5 * voxel_num,
										inverse_covariance + 6 * voxel_num, inverse_covariance + 7 * voxel_num, inverse_covariance + 8 * voxel_num);
	checkCudaErrors(cudaGetLastError());

	computeScoreList<eleType><<<grid_x, block_x>>>(starting_voxel_id, voxel_id, valid_points_num, e_x_cov_x, gauss_d1_, score);
	checkCudaErrors(cudaGetLastError());

	int block_x2 = (valid_voxel_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : valid_voxel_num;
	int grid_x2 = (valid_voxel_num - 1) / block_x2 + 1;

	updateExCovX<eleType><<<grid_x2, block_x2>>>(e_x_cov_x, gauss_d2_, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());

	grid.x = grid_x;
	grid.y = 3;
	grid.z = 6;

	computeCovDxdPi<eleType><<<grid, block_x>>>(valid_points, starting_voxel_id, voxel_id, valid_points_num,
											inverse_covariance, voxel_num,
											gauss_d1_, gauss_d2_, point_gradients,
											cov_dxd_pi, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());

	grid.x = grid_x;
	grid.y = 6;
	grid.z = 1;

	computeScoreGradientList<eleType><<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, centroid + voxel_num, centroid + 2 * voxel_num,
													voxel_num, e_x_cov_x,
													cov_dxd_pi, gauss_d1_, valid_voxel_num, gradients);

	checkCudaErrors(cudaGetLastError());


	if (compute_hessian) {

		grid.y = 6;
		grid.z = 1;


		computeHessianListS0<eleType><<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												centroid, centroid + voxel_num, centroid + 2 * voxel_num,
												inverse_covariance, inverse_covariance + voxel_num, inverse_covariance + 2 * voxel_num,
												inverse_covariance + 3 * voxel_num, inverse_covariance + 4 * voxel_num, inverse_covariance + 5 * voxel_num,
												inverse_covariance + 6 * voxel_num, inverse_covariance + 7 * voxel_num, inverse_covariance + 8 * voxel_num,
												point_gradients,
												tmp_hessian, valid_voxel_num);
		checkCudaErrors(cudaGetLastError());
		grid.z = 6;

		computeHessianListS1<eleType><<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, centroid + voxel_num, centroid + 2 * voxel_num,
													gauss_d1_, gauss_d2_, hessians,
													e_x_cov_x, tmp_hessian, cov_dxd_pi,
													point_gradients,
													valid_voxel_num);
		checkCudaErrors(cudaGetLastError());

		computeHessianListS2<eleType><<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, centroid + voxel_num, centroid + 2 * voxel_num,
													gauss_d1_, e_x_cov_x,
													inverse_covariance, inverse_covariance + voxel_num, inverse_covariance + 2 * voxel_num,
													inverse_covariance + 3 * voxel_num, inverse_covariance + 4 * voxel_num, inverse_covariance + 5 * voxel_num,
													inverse_covariance + 6 * voxel_num, inverse_covariance + 7 * voxel_num, inverse_covariance + 8 * voxel_num,
													point_hessians, hessians, valid_voxel_num);
		checkCudaErrors(cudaGetLastError());

	}

	int full_size = valid_points_num;
	int half_size = (full_size - 1) / 2 + 1;

	while (full_size > 1) {
		block_x = (half_size > BLOCK_SIZE_X) ? BLOCK_SIZE_X : half_size;
		grid_x = (half_size - 1) / block_x + 1;

		grid.x = grid_x;
		grid.y = 1;
		grid.z = 6;
		matrixSum<eleType><<<grid, block_x>>>(gradients, full_size, half_size, 1, 6, valid_points_num);
		checkCudaErrors(cudaGetLastError());

		grid.y = 6;
		matrixSum<eleType><<<grid, block_x>>>(hessians, full_size, half_size, 6, 6, valid_points_num);
		checkCudaErrors(cudaGetLastError());

		sumScore<eleType><<<grid_x, block_x>>>(score, full_size, half_size);
		checkCudaErrors(cudaGetLastError());

		full_size = half_size;
		half_size = (full_size - 1) / 2 + 1;
	}

	checkCudaErrors(cudaDeviceSynchronize());

	MatrixDevice<eleType> dgrad(1, 6, valid_points_num, gradients), dhess(6, 6, valid_points_num, hessians);
	MatrixHost<eleType> hgrad(1, 6), hhess(6, 6);

	hgrad.moveToHost(dgrad);
	hhess.moveToHost(dhess);

	for (int i = 0; i < 6; i++) {
		score_gradient(i) = hgrad(i);
	}

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			hessian(i, j) = hhess(i, j);
		}
	}

	eleType score_inc;

	checkCudaErrors(cudaMemcpy(&score_inc, score, sizeof(eleType), cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaFree(gradients));
	checkCudaErrors(cudaFree(hessians));
	checkCudaErrors(cudaFree(point_hessians));
	checkCudaErrors(cudaFree(point_gradients));
	checkCudaErrors(cudaFree(score));

	checkCudaErrors(cudaFree(tmp_hessian));

	checkCudaErrors(cudaFree(e_x_cov_x));
	checkCudaErrors(cudaFree(cov_dxd_pi));

	if (valid_points != NULL)
		checkCudaErrors(cudaFree(valid_points));

	if (voxel_id != NULL)
		checkCudaErrors(cudaFree(voxel_id));

	if (starting_voxel_id != NULL)
		checkCudaErrors(cudaFree(starting_voxel_id));

	return score_inc;
}

template <typename eleType>
void GNormalDistributionsTransform<eleType>::computeAngleDerivatives(MatrixHost<eleType> pose, bool compute_hessian)
{
	eleType cx, cy, cz, sx, sy, sz;

	if (fabs(pose(3)) < 10e-5) {
		cx = 1.0;
		sx = 0.0;
	} else {
		cx = cos(pose(3));
		sx = sin(pose(3));
	}

	if (fabs(pose(4)) < 10e-5) {
		cy = 1.0;
		sy = 0.0;
	} else {
		cy = cos(pose(4));
		sy = sin(pose(4));
	}

	if (fabs(pose(5)) < 10e-5) {
		cz = 1.0;
		sz = 0.0;
	} else {
		cz = cos(pose(5));
		sz = sin(pose(5));
	}


	j_ang_(0) = -sx * sz + cx * sy * cz;
	j_ang_(1) = -sx * cz - cx * sy * sz;
	j_ang_(2) = -cx * cy;

	j_ang_(3) = cx * sz + sx * sy * cz;
	j_ang_(4) = cx * cz - sx * sy * sz;
	j_ang_(5) = -sx * cy;

	j_ang_(6) = -sy * cz;
	j_ang_(7) = sy * sz;
	j_ang_(8) = cy;

	j_ang_(9) = sx * cy * cz;
	j_ang_(10) = -sx * cy * sz;
	j_ang_(11) = sx * sy;

	j_ang_(12) = -cx * cy * cz;
	j_ang_(13) = cx * cy * sz;
	j_ang_(14) = -cx * sy;

	j_ang_(15) = -cy * sz;
	j_ang_(16) = -cy * cz;
	j_ang_(17) = 0;

	j_ang_(18) = cx * cz - sx * sy * sz;
	j_ang_(19) = -cx * sz - sx * sy * cz;
	j_ang_(20) = 0;

	j_ang_(21) = sx * cz + cx * sy * sz;
	j_ang_(22) = cx * sy * cz - sx * sz;
	j_ang_(23) = 0;

	j_ang_.moveToGpu(dj_ang_);

	if (compute_hessian) {

		h_ang_(0) = -cx * sz - sx * sy * cz;
		h_ang_(1) = -cx * cz + sx * sy * sz;
		h_ang_(2) = sx * cy;

		h_ang_(3) = -sx * sz + cx * sy * cz;
		h_ang_(4) = -cx * sy * sz - sx * cz;
		h_ang_(5) = -cx * cy;

		h_ang_(6) = cx * cy * cz;
		h_ang_(7) = -cx * cy * sz;
		h_ang_(8) = cx * sy;

		h_ang_(9) = sx * cy * cz;
		h_ang_(10) = -sx * cy * sz;
		h_ang_(11) = sx * sy;

		h_ang_(12) = -sx * cz - cx * sy * sz;
		h_ang_(13) = sx * sz - cx * sy * cz;
		h_ang_(14) = 0;

		h_ang_(15) = cx * cz - sx * sy * sz;
		h_ang_(16) = -sx * sy * cz - cx * sz;
		h_ang_(17) = 0;

		h_ang_(18) = -cy * cz;
		h_ang_(19) = cy * sz;
		h_ang_(20) = sy;

		h_ang_(21) = -sx * sy * cz;
		h_ang_(22) = sx * sy * sz;
		h_ang_(23) = sx * cy;

		h_ang_(24) = cx * sy * cz;
		h_ang_(25) = -cx * sy * sz;
		h_ang_(26) = -cx * cy;

		h_ang_(27) = sy * sz;
		h_ang_(28) = sy * cz;
		h_ang_(29) = 0;

		h_ang_(30) = -sx * cy * sz;
		h_ang_(31) = -sx * cy * cz;
		h_ang_(32) = 0;

		h_ang_(33) = cx * cy * sz;
		h_ang_(34) = cx * cy * cz;
		h_ang_(35) = 0;

		h_ang_(36) = -cy * cz;
		h_ang_(37) = cy * sz;
		h_ang_(38) = 0;

		h_ang_(39) = -cx * sz - sx * sy * cz;
		h_ang_(40) = -cx * cz + sx * sy * sz;
		h_ang_(41) = 0;

		h_ang_(42) = -sx * sz + cx * sy * cz;
		h_ang_(43) = -cx * sy * sz - sx * cz;
		h_ang_(44) = 0;

		h_ang_.moveToGpu(dh_ang_);
	}

}

template <typename eleType>
__global__ void gpuTransform(float *in_x, float *in_y, float *in_z,
										float *trans_x, float *trans_y, float *trans_z,
										int point_num, MatrixDevice<eleType> transform)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	float x, y, z;

	for (int i = idx; i < point_num; i += stride) {
		x = in_x[i];
		y = in_y[i];
		z = in_z[i];
		trans_x[i] = transform(0, 0) * x + transform(0, 1) * y + transform(0, 2) * z + transform(0, 3);
		trans_y[i] = transform(1, 0) * x + transform(1, 1) * y + transform(1, 2) * z + transform(1, 3);
		trans_z[i] = transform(2, 0) * x + transform(2, 1) * y + transform(2, 2) * z + transform(2, 3);
	}
}

template <typename eleType>
void GNormalDistributionsTransform<eleType>::transformPointCloud(float *in_x, float *in_y, float *in_z,
														float *trans_x, float *trans_y, float *trans_z,
														int points_number, Eigen::Matrix<float, 4, 4> transform)
{
	Eigen::Transform<float, 3, Eigen::Affine> t(transform);

	MatrixHost<eleType> htrans(3, 4);
	MatrixDevice<eleType> dtrans(3, 4);

	dtrans.memAlloc();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			htrans(i, j) = t(i, j);
		}
	}

	htrans.moveToGpu(dtrans);

	if (points_number > 0) {
		int block_x = (points_number <= BLOCK_SIZE_X) ? points_number : BLOCK_SIZE_X;
		int grid_x = (points_number - 1) / block_x + 1;

		gpuTransform<eleType><<<grid_x, block_x >>>(in_x, in_y, in_z, trans_x, trans_y, trans_z, points_number, dtrans);
		checkCudaErrors(cudaGetLastError());
		checkCudaErrors(cudaDeviceSynchronize());
	}

	dtrans.memFree();
}

template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::computeStepLengthMT(const Eigen::Matrix<eleType, 6, 1> &x, Eigen::Matrix<eleType, 6, 1> &step_dir,
															eleType step_init, eleType step_max, eleType step_min, eleType &score,
															Eigen::Matrix<eleType, 6, 1> &score_gradient, Eigen::Matrix<eleType, 6, 6> &hessian,
															float *trans_x, float *trans_y, float *trans_z, int points_num)
{
	eleType phi_0 = -score;
	eleType d_phi_0 = -(score_gradient.dot(step_dir));

	Eigen::Matrix<eleType, 6, 1> x_t;

	if (d_phi_0 >= 0) {
		if (d_phi_0 == 0)
			return 0;
		else {
			d_phi_0 *= -1;
			step_dir *= -1;
		}
	}

	int max_step_iterations = 10;
	int step_iterations = 0;


	eleType mu = 1.e-4;
	eleType nu = 0.9;
	eleType a_l = 0, a_u = 0;

	eleType f_l = auxilaryFunction_PsiMT(a_l, phi_0, phi_0, d_phi_0, mu);
	eleType g_l = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

	eleType f_u = auxilaryFunction_PsiMT(a_u, phi_0, phi_0, d_phi_0, mu);
	eleType g_u = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

	bool interval_converged = (step_max - step_min) > 0, open_interval = true;

	eleType a_t = step_init;
	a_t = std::min(a_t, step_max);
	a_t = std::max(a_t, step_min);

	x_t = x + step_dir * a_t;

	Eigen::Translation<float, 3> translation(static_cast<float>(x_t(0)), static_cast<float>(x_t(1)), static_cast<float>(x_t(2)));
	Eigen::AngleAxis<float> tmp1(static_cast<float>(x_t(3)), Eigen::Vector3f::UnitX());
	Eigen::AngleAxis<float> tmp2(static_cast<float>(x_t(4)), Eigen::Vector3f::UnitY());
	Eigen::AngleAxis<float> tmp3(static_cast<float>(x_t(5)), Eigen::Vector3f::UnitZ());
	Eigen::AngleAxis<float> tmp4(tmp1 * tmp2 * tmp3);

	final_transformation_ = (translation * tmp4).matrix();

	transformPointCloud(x_, y_, z_, trans_x, trans_y, trans_z, points_num, final_transformation_);

	score = computeDerivatives(score_gradient, hessian, trans_x, trans_y, trans_z, points_num, x_t);

	eleType phi_t = -score;
	eleType d_phi_t = -(score_gradient.dot(step_dir));
	eleType psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
	eleType d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

	while (!interval_converged && step_iterations < max_step_iterations && !(psi_t <= 0 && d_phi_t <= -nu * d_phi_0)) {
		if (open_interval) {
			a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
		} else {
			a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
		}

		a_t = (a_t < step_max) ? a_t : step_max;
		a_t = (a_t > step_min) ? a_t : step_min;

		x_t = x + step_dir * a_t;

		translation = Eigen::Translation<float, 3>(static_cast<float>(x_t(0)), static_cast<float>(x_t(1)), static_cast<float>(x_t(2)));
		tmp1 = Eigen::AngleAxis<float>(static_cast<float>(x_t(3)), Eigen::Vector3f::UnitX());
		tmp2 = Eigen::AngleAxis<float>(static_cast<float>(x_t(4)), Eigen::Vector3f::UnitY());
		tmp3 = Eigen::AngleAxis<float>(static_cast<float>(x_t(5)), Eigen::Vector3f::UnitZ());
		tmp4 = tmp1 * tmp2 * tmp3;

		final_transformation_ = (translation * tmp4).matrix();

		transformPointCloud(x_, y_, z_, trans_x, trans_y, trans_z, points_num, final_transformation_);

		score = computeDerivatives(score_gradient, hessian, trans_x, trans_y, trans_z, points_num, x_t, false);

		phi_t -= score;
		d_phi_t -= (score_gradient.dot(step_dir));
		psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
		d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

		if (open_interval && (psi_t <= 0 && d_psi_t >= 0)) {
			open_interval = false;

			f_l += phi_0 - mu * d_phi_0 * a_l;
			g_l += mu * d_phi_0;

			f_u += phi_0 - mu * d_phi_0 * a_u;
			g_u += mu * d_phi_0;
		}

		if (open_interval) {
			interval_converged = updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
		} else {
			interval_converged = updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
		}
		step_iterations++;
	}

	if (step_iterations) {
		computeHessian(hessian, trans_x, trans_y, trans_z, points_num, x_t);
	}

	real_iterations_ += step_iterations;

	return a_t;
}


//Copied from ndt.hpp
template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::trialValueSelectionMT (eleType a_l, eleType f_l, eleType g_l,
															eleType a_u, eleType f_u, eleType g_u,
															eleType a_t, eleType f_t, eleType g_t)
{
	// Case 1 in Trial Value Selection [More, Thuente 1994]
	if (f_t > f_l) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		eleType z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		eleType w = std::sqrt (z * z - g_t * g_l);
		// Equation 2.4.56 [Sun, Yuan 2006]
		eleType a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

		// Calculate the minimizer of the quadratic that interpolates f_l, f_t and g_l
		// Equation 2.4.2 [Sun, Yuan 2006]
		eleType a_q = a_l - 0.5 * (a_l - a_t) * g_l / (g_l - (f_l - f_t) / (a_l - a_t));

		if (std::fabs (a_c - a_l) < std::fabs (a_q - a_l))
		  return (a_c);
		else
		  return (0.5 * (a_q + a_c));
	}
	// Case 2 in Trial Value Selection [More, Thuente 1994]
	else if (g_t * g_l < 0) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		eleType z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		eleType w = std::sqrt (z * z - g_t * g_l);
		// Equation 2.4.56 [Sun, Yuan 2006]
		eleType a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

		// Calculate the minimizer of the quadratic that interpolates f_l, g_l and g_t
		// Equation 2.4.5 [Sun, Yuan 2006]
		eleType a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

		if (std::fabs (a_c - a_t) >= std::fabs (a_s - a_t))
		  return (a_c);
		else
		  return (a_s);
	}
	// Case 3 in Trial Value Selection [More, Thuente 1994]
	else if (std::fabs (g_t) <= std::fabs (g_l)) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		eleType z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		eleType w = std::sqrt (z * z - g_t * g_l);
		eleType a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

		// Calculate the minimizer of the quadratic that interpolates g_l and g_t
		// Equation 2.4.5 [Sun, Yuan 2006]
		eleType a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

		eleType a_t_next;

		if (std::fabs (a_c - a_t) < std::fabs (a_s - a_t))
		  a_t_next = a_c;
		else
		  a_t_next = a_s;

		if (a_t > a_l)
			return (a_t + 0.66 * (a_u - a_t) < a_t_next) ? a_t + 0.66 * (a_u - a_t) : a_t_next;
		  //return (std::min (a_t + 0.66 * (a_u - a_t), a_t_next));
		else
			return (a_t + 0.66 * (a_u - a_t) > a_t_next) ? a_t + 0.66 * (a_u - a_t) : a_t_next;
		  //return (std::max (a_t + 0.66 * (a_u - a_t), a_t_next));
	}
	// Case 4 in Trial Value Selection [More, Thuente 1994]
	else {
		// Calculate the minimizer of the cubic that interpolates f_u, f_t, g_u and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		eleType z = 3 * (f_t - f_u) / (a_t - a_u) - g_t - g_u;
		eleType w = std::sqrt (z * z - g_t * g_u);
		// Equation 2.4.56 [Sun, Yuan 2006]
		return (a_u + (a_t - a_u) * (w - g_u - z) / (g_t - g_u + 2 * w));
	}
}

//Copied from ndt.hpp
template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::updateIntervalMT (eleType &a_l, eleType &f_l, eleType &g_l,
														eleType &a_u, eleType &f_u, eleType &g_u,
														eleType a_t, eleType f_t, eleType g_t)
{
  // Case U1 in Update Algorithm and Case a in Modified Update Algorithm [More, Thuente 1994]
	if (f_t > f_l) {
		a_u = a_t;
		f_u = f_t;
		g_u = g_t;
		return (false);
	}
	// Case U2 in Update Algorithm and Case b in Modified Update Algorithm [More, Thuente 1994]
	else if (g_t * (a_l - a_t) > 0) {
		a_l = a_t;
		f_l = f_t;
		g_l = g_t;
		return (false);
	}
	// Case U3 in Update Algorithm and Case c in Modified Update Algorithm [More, Thuente 1994]
	else if (g_t * (a_l - a_t) < 0) {
		a_u = a_l;
		f_u = f_l;
		g_u = g_l;

		a_l = a_t;
		f_l = f_t;
		g_l = g_t;
		return (false);
	}
	// Interval Converged
	else
		return (true);
}

template <typename eleType>
void GNormalDistributionsTransform<eleType>::computeHessian(Eigen::Matrix<eleType, 6, 6> &hessian, float *trans_x, float *trans_y, float *trans_z, int points_num, Eigen::Matrix<eleType, 6, 1> &p)
{
	int *valid_points, *voxel_id, *starting_voxel_id;
	int valid_voxel_num, valid_points_num;
	//Radius Search
	voxel_grid_.radiusSearch(trans_x, trans_y, trans_z, points_num, resolution_, INT_MAX, &valid_points, &starting_voxel_id, &voxel_id, &valid_voxel_num, &valid_points_num);

	eleType *centroid = voxel_grid_.getCentroidList();
	eleType *covariance = voxel_grid_.getCovarianceList();
	eleType *inverse_covariance = voxel_grid_.getInverseCovarianceList();
	int *points_per_voxel = voxel_grid_.getPointsPerVoxelList();
	int voxel_num = voxel_grid_.getVoxelNum();

	if (valid_points_num <= 0)
		return;

	//Update score gradient and hessian matrix
	eleType *hessians, *point_gradients, *point_hessians;

	checkCudaErrors(cudaMalloc(&hessians, sizeof(eleType) * valid_points_num * 6 * 6));

	checkCudaErrors(cudaMalloc(&point_gradients, sizeof(eleType) * valid_points_num * 3 * 6));

	checkCudaErrors(cudaMalloc(&point_hessians, sizeof(eleType) * valid_points_num * 18 * 6));

	checkCudaErrors(cudaMemset(hessians, 0, sizeof(eleType) * valid_points_num * 6 * 6));
	checkCudaErrors(cudaMemset(point_gradients, 0, sizeof(eleType) * valid_points_num * 3 * 6));
	checkCudaErrors(cudaMemset(point_hessians, 0, sizeof(eleType) * valid_points_num * 18 * 6));

	int block_x = (valid_points_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : valid_points_num;
	int grid_x = (valid_points_num - 1) / block_x + 1;
	dim3 grid;

	computePointGradients0<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dj_ang_.buffer(),
												point_gradients,
												point_gradients + valid_points_num * 7,
												point_gradients + valid_points_num * 14,
												point_gradients + valid_points_num * 9,
												point_gradients + valid_points_num * 15,
												point_gradients + valid_points_num * 4,
												point_gradients + valid_points_num * 10);
	checkCudaErrors(cudaGetLastError());

	computePointGradients1<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dj_ang_.buffer(),
												point_gradients + valid_points_num * 16,
												point_gradients + valid_points_num * 5,
												point_gradients + valid_points_num * 11,
												point_gradients + valid_points_num * 17);
	checkCudaErrors(cudaGetLastError());


	computePointHessian0<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_.buffer(),
												point_hessians + valid_points_num * 57, point_hessians + valid_points_num * 63, point_hessians + valid_points_num * 69,
												point_hessians + valid_points_num * 75, point_hessians + valid_points_num * 58, point_hessians + valid_points_num * 81,
												point_hessians + valid_points_num * 64, point_hessians + valid_points_num * 87, point_hessians + valid_points_num * 70,
												point_hessians + valid_points_num * 93, point_hessians + valid_points_num * 59, point_hessians + valid_points_num * 99,
												point_hessians + valid_points_num * 65, point_hessians + valid_points_num * 105, point_hessians + valid_points_num * 71);
	checkCudaErrors(cudaGetLastError());

	computePointHessian1<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_.buffer(),
												point_hessians + valid_points_num * 76, point_hessians + valid_points_num * 82, point_hessians + valid_points_num * 88,
												point_hessians + valid_points_num * 94, point_hessians + valid_points_num * 77, point_hessians + valid_points_num * 100,
												point_hessians + valid_points_num * 83, point_hessians + valid_points_num * 106, point_hessians + valid_points_num * 89);
	checkCudaErrors(cudaGetLastError());

	computePointHessian2<eleType><<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_.buffer(),
												point_hessians + valid_points_num * 95, point_hessians + valid_points_num * 101, point_hessians + valid_points_num * 107);
	checkCudaErrors(cudaGetLastError());

	eleType *tmp_hessian;

	checkCudaErrors(cudaMalloc(&tmp_hessian, sizeof(eleType) * valid_voxel_num * 6));

	eleType *e_x_cov_x;

	checkCudaErrors(cudaMalloc(&e_x_cov_x, sizeof(eleType) * valid_voxel_num));

	eleType *cov_dxd_pi;

	checkCudaErrors(cudaMalloc(&cov_dxd_pi, sizeof(eleType) * valid_voxel_num * 3 * 6));

	computeExCovX<eleType><<<grid_x, block_x>>>(trans_x, trans_y, trans_z, valid_points,
										starting_voxel_id, voxel_id, valid_points_num,
										centroid, centroid + voxel_num, centroid + 2 * voxel_num,
										gauss_d1_, gauss_d2_,
										e_x_cov_x,
										inverse_covariance, inverse_covariance + voxel_num, inverse_covariance + 2 * voxel_num,
										inverse_covariance + 3 * voxel_num, inverse_covariance + 4 * voxel_num, inverse_covariance + 5 * voxel_num,
										inverse_covariance + 6 * voxel_num, inverse_covariance + 7 * voxel_num, inverse_covariance + 8 * voxel_num);

	checkCudaErrors(cudaGetLastError());

	grid.x = grid_x;
	grid.y = 3;
	grid.z = 6;
	computeCovDxdPi<eleType><<<grid, block_x>>>(valid_points, starting_voxel_id, voxel_id, valid_points_num,
											inverse_covariance, voxel_num,
											gauss_d1_, gauss_d2_, point_gradients,
											cov_dxd_pi, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());

	int block_x2 = (valid_voxel_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : valid_voxel_num;
	int grid_x2 = (valid_voxel_num - 1) / block_x2 + 1;


	updateExCovX<eleType><<<grid_x2, block_x2>>>(e_x_cov_x, gauss_d2_, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());

	grid.y = 6;
	grid.z = 1;

	computeHessianListS0<eleType><<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												centroid, centroid + voxel_num, centroid + 2 * voxel_num,
												inverse_covariance, inverse_covariance + voxel_num, inverse_covariance + 2 * voxel_num,
												inverse_covariance + 3 * voxel_num, inverse_covariance + 4 * voxel_num, inverse_covariance + 5 * voxel_num,
												inverse_covariance + 6 * voxel_num, inverse_covariance + 7 * voxel_num, inverse_covariance + 8 * voxel_num,
												point_gradients,
												tmp_hessian, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());

	grid.z = 6;

	computeHessianListS1<eleType><<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												centroid, centroid + voxel_num, centroid + 2 * voxel_num,
												gauss_d1_, gauss_d2_, hessians,
												e_x_cov_x, tmp_hessian, cov_dxd_pi,
												point_gradients,
												valid_voxel_num);
	checkCudaErrors(cudaGetLastError());

	computeHessianListS2<eleType><<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												centroid, centroid + voxel_num, centroid + 2 * voxel_num,
												gauss_d1_, e_x_cov_x,
												inverse_covariance, inverse_covariance + voxel_num, inverse_covariance + 2 * voxel_num,
												inverse_covariance + 3 * voxel_num, inverse_covariance + 4 * voxel_num, inverse_covariance + 5 * voxel_num,
												inverse_covariance + 6 * voxel_num, inverse_covariance + 7 * voxel_num, inverse_covariance + 8 * voxel_num,
												point_hessians, hessians, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());


	int full_size = valid_points_num;
	int half_size = (full_size - 1) / 2 + 1;

	while (full_size > 1) {
		block_x = (half_size > BLOCK_SIZE_X) ? BLOCK_SIZE_X : half_size;
		grid_x = (half_size - 1) / block_x + 1;

		grid.x = grid_x;
		grid.y = 6;
		grid.z = 6;
		matrixSum<eleType><<<grid_x, block_x>>>(hessians, full_size, half_size, 6, 6, valid_points_num);

		full_size = half_size;
		half_size = (full_size - 1) / 2 + 1;
	}

	checkCudaErrors(cudaDeviceSynchronize());

	MatrixDevice<eleType> dhessian(6, 6, valid_points_num, hessians);
	MatrixHost<eleType> hhessian(6, 6);

	hhessian.moveToHost(dhessian);

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			hessian(i, j) = hhessian(i, j);
		}
	}

	checkCudaErrors(cudaFree(hessians));
	checkCudaErrors(cudaFree(point_hessians));
	checkCudaErrors(cudaFree(point_gradients));

	checkCudaErrors(cudaFree(tmp_hessian));
	checkCudaErrors(cudaFree(e_x_cov_x));
	checkCudaErrors(cudaFree(cov_dxd_pi));

	if (valid_points != NULL) {
		checkCudaErrors(cudaFree(valid_points));
	}

	if (voxel_id != NULL) {
		checkCudaErrors(cudaFree(voxel_id));
	}

	if (starting_voxel_id != NULL) {
		checkCudaErrors(cudaFree(starting_voxel_id));
	}

	dhessian.memFree();
}

template <typename T>
__global__ void gpuSum(T *input, int size, int half_size)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = idx; i < half_size; i += stride) {
		if (i + half_size < size) {
			input[i] += (half_size < size) ? input[i + half_size] : 0;
		}
	}
}

template <typename eleType>
eleType GNormalDistributionsTransform<eleType>::getFitnessScore(eleType max_range)
{
	eleType fitness_score = 0.0;

	float *trans_x, *trans_y, *trans_z;

	checkCudaErrors(cudaMalloc(&trans_x, sizeof(float) * points_number_));
	checkCudaErrors(cudaMalloc(&trans_y, sizeof(float) * points_number_));
	checkCudaErrors(cudaMalloc(&trans_z, sizeof(float) * points_number_));

	transformPointCloud(x_, y_, z_, trans_x, trans_y, trans_z, points_number_, final_transformation_);

	int *valid_distance;

	checkCudaErrors(cudaMalloc(&valid_distance, sizeof(int) * points_number_));

	eleType *min_distance;

	checkCudaErrors(cudaMalloc(&min_distance, sizeof(eleType) * points_number_));

	voxel_grid_.nearestNeighborSearch(trans_x, trans_y, trans_z, points_number_, valid_distance, min_distance, max_range);

	int size = points_number_;
	int half_size;

	while (size > 1) {
		half_size = (size - 1) / 2 + 1;

		int block_x = (half_size > BLOCK_SIZE_X) ? BLOCK_SIZE_X : half_size;
		int grid_x = (half_size - 1) / block_x + 1;

		gpuSum<eleType><<<grid_x, block_x>>>(min_distance, size, half_size);
		checkCudaErrors(cudaGetLastError());

		gpuSum<int><<<grid_x, block_x>>>(valid_distance, size, half_size);
		checkCudaErrors(cudaGetLastError());

		size = half_size;
	}

	checkCudaErrors(cudaDeviceSynchronize());

	int nr;

	checkCudaErrors(cudaMemcpy(&nr, valid_distance, sizeof(int), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&fitness_score, min_distance, sizeof(eleType), cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaFree(trans_x));
	checkCudaErrors(cudaFree(trans_y));
	checkCudaErrors(cudaFree(trans_z));
	checkCudaErrors(cudaFree(valid_distance));
	checkCudaErrors(cudaFree(min_distance));

	if (nr > 0)
		return (fitness_score / nr);

	return DBL_MAX;
}

template class GNormalDistributionsTransform<float>;
template class GNormalDistributionsTransform<double>;
}
