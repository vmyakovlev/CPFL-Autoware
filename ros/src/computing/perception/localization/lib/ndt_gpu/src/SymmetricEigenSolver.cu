#include "ndt_gpu/common.h"
#include "ndt_gpu/debug.h"
#include "ndt_gpu/SymmetricEigenSolver.h"

namespace gpu {

template <typename eleType>
SymmetricEigensolver3x3<eleType>::SymmetricEigensolver3x3(int offset)
{
	offset_ = offset;

	checkCudaErrors(cudaMalloc(&buffer_, sizeof(eleType) * 18 * offset_));
	checkCudaErrors(cudaMalloc(&maxAbsElement_, sizeof(eleType) * offset_));
	checkCudaErrors(cudaMalloc(&norm_, sizeof(eleType) * offset_));
	checkCudaErrors(cudaMalloc(&i02_, sizeof(int) * 2 * offset_));

	eigenvectors_ = NULL;
	eigenvalues_ = NULL;
	input_matrices_ = NULL;

	is_copied_ = false;
}

template <typename eleType>
void SymmetricEigensolver3x3<eleType>::setInputMatrices(eleType *input_matrices)
{
	input_matrices_ = input_matrices;
}

template <typename eleType>
void SymmetricEigensolver3x3<eleType>::setEigenvectors(eleType *eigenvectors)
{
	eigenvectors_ = eigenvectors;
}

template <typename eleType>
void SymmetricEigensolver3x3<eleType>::setEigenvalues(eleType *eigenvalues)
{
	eigenvalues_ = eigenvalues;
}

template <typename eleType>
eleType* SymmetricEigensolver3x3<eleType>::getBuffer() const
{
	return buffer_;
}

template <typename eleType>
void SymmetricEigensolver3x3<eleType>::memFree()
{
	if (!is_copied_) {
		if (buffer_ != NULL) {
			checkCudaErrors(cudaFree(buffer_));
			buffer_ = NULL;
		}

		if (maxAbsElement_ != NULL) {
			checkCudaErrors(cudaFree(maxAbsElement_));
			maxAbsElement_ = NULL;
		}

		if (norm_ != NULL) {
			checkCudaErrors(cudaFree(norm_));
			norm_ = NULL;
		}

		if (i02_ != NULL) {
			checkCudaErrors(cudaFree(i02_));
			i02_ = NULL;
		}
	}
}

template class SymmetricEigensolver3x3<float>;
template class SymmetricEigensolver3x3<double>;
}
