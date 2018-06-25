#ifndef GPU_NDT_H_
#define GPU_NDT_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include "Registration.h"
#include "common.h"
#include "VoxelGrid.h"
#include "Eigen/Geometry"

namespace gpu {
template <typename eleType = float>
class GNormalDistributionsTransform: public GRegistration<eleType> {
public:
	GNormalDistributionsTransform();

	GNormalDistributionsTransform(const GNormalDistributionsTransform &other);

	void setStepSize(eleType step_size);

	void setResolution(float resolution);

	void setOutlierRatio(eleType olr);

	eleType getStepSize() const;

	float getResolution() const;

	eleType getOutlierRatio() const;

	eleType getTransformationProbability() const;

	int getRealIterations();

	/* Set the input map points */
	void setInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
	void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

	/* Compute and get fitness score */
	eleType getFitnessScore(eleType max_range = DBL_MAX);

	~GNormalDistributionsTransform();

protected:
	void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess);
	eleType computeDerivatives(Eigen::Matrix<eleType, 6, 1> &score_gradient, Eigen::Matrix<eleType, 6, 6> &hessian,
								float *trans_x, float *trans_y, float *trans_z,
								int points_num, Eigen::Matrix<eleType, 6, 1> pose, bool compute_hessian = true);


	using GRegistration<eleType>::transformation_epsilon_;
	using GRegistration<eleType>::max_iterations_;

	//Original scanned point clouds
	using GRegistration<eleType>::x_;
	using GRegistration<eleType>::y_;
	using GRegistration<eleType>::z_;
	using GRegistration<eleType>::points_number_;

	//Transformed point clouds
	using GRegistration<eleType>::trans_x_;
	using GRegistration<eleType>::trans_y_;
	using GRegistration<eleType>::trans_z_;

	using GRegistration<eleType>::converged_;
	using GRegistration<eleType>::nr_iterations_;

	using GRegistration<eleType>::final_transformation_;
	using GRegistration<eleType>::transformation_;
	using GRegistration<eleType>::previous_transformation_;

	using GRegistration<eleType>::target_cloud_updated_;

	// Reference map point
	using GRegistration<eleType>::target_x_;
	using GRegistration<eleType>::target_y_;
	using GRegistration<eleType>::target_z_;
	using GRegistration<eleType>::target_points_number_;

	using GRegistration<eleType>::is_copied_;

private:
	//Copied from ndt.h
    eleType auxilaryFunction_PsiMT (eleType a, eleType f_a, eleType f_0, eleType g_0, eleType mu = 1.e-4);

    //Copied from ndt.h
    eleType auxilaryFunction_dPsiMT (eleType g_a, eleType g_0, eleType mu = 1.e-4);

    eleType updateIntervalMT (eleType &a_l, eleType &f_l, eleType &g_l,
								eleType &a_u, eleType &f_u, eleType &g_u,
								eleType a_t, eleType f_t, eleType g_t);

    eleType trialValueSelectionMT (eleType a_l, eleType f_l, eleType g_l,
									eleType a_u, eleType f_u, eleType g_u,
									eleType a_t, eleType f_t, eleType g_t);

	void transformPointCloud(float *in_x, float *in_y, float *in_z,
								float *out_x, float *out_y, float *out_z,
								int points_number, Eigen::Matrix<float, 4, 4> transform);

	void computeAngleDerivatives(MatrixHost<eleType> pose, bool compute_hessian = true);

	eleType computeStepLengthMT(const Eigen::Matrix<eleType, 6, 1> &x, Eigen::Matrix<eleType, 6, 1> &step_dir,
								eleType step_init, eleType step_max, eleType step_min, eleType &score,
								Eigen::Matrix<eleType, 6, 1> &score_gradient, Eigen::Matrix<eleType, 6, 6> &hessian,
								float *out_x, float *out_y, float *out_z, int points_num);

	void computeHessian(Eigen::Matrix<eleType, 6, 6> &hessian, float *trans_x, float *trans_y, float *trans_z, int points_num, Eigen::Matrix<eleType, 6, 1> &p);


	eleType gauss_d1_, gauss_d2_;
	eleType outlier_ratio_;
	//MatrixHost j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;
	MatrixHost<eleType> j_ang_;

	//MatrixHost h_ang_a2_, h_ang_a3_, h_ang_b2_, h_ang_b3_, h_ang_c2_, h_ang_c3_, h_ang_d1_, h_ang_d2_, h_ang_d3_,
	//			h_ang_e1_, h_ang_e2_, h_ang_e3_, h_ang_f1_, h_ang_f2_, h_ang_f3_;
	MatrixHost<eleType> h_ang_;


	//MatrixDevice dj_ang_a_, dj_ang_b_, dj_ang_c_, dj_ang_d_, dj_ang_e_, dj_ang_f_, dj_ang_g_, dj_ang_h_;
	MatrixDevice<eleType> dj_ang_;


	//MatrixDevice dh_ang_a2_, dh_ang_a3_, dh_ang_b2_, dh_ang_b3_, dh_ang_c2_, dh_ang_c3_, dh_ang_d1_, dh_ang_d2_, dh_ang_d3_,
	//			dh_ang_e1_, dh_ang_e2_, dh_ang_e3_, dh_ang_f1_, dh_ang_f2_, dh_ang_f3_;
	MatrixDevice<eleType> dh_ang_;

	eleType step_size_;
	float resolution_;
	eleType trans_probability_;

	int real_iterations_;


	GVoxelGrid<eleType> voxel_grid_;
};

}


#endif
