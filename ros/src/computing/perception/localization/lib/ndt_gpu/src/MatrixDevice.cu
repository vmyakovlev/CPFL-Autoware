#include "ndt_gpu/common.h"
#include "ndt_gpu/debug.h"
#include "ndt_gpu/MatrixDevice.h"

namespace gpu {
template <typename eleType>
MatrixDevice<eleType>::MatrixDevice(int rows, int cols) {
	rows_ = rows;
	cols_ = cols;
	offset_ = 1;
	fr_ = true;
	buffer_ = NULL;
}

template <typename eleType>
void MatrixDevice<eleType>::memAlloc()
{
	if (buffer_ != NULL && fr_) {
		checkCudaErrors(cudaFree(buffer_));
		buffer_ = NULL;
	}

	checkCudaErrors(cudaMalloc(&buffer_, sizeof(eleType) * rows_ * cols_ * offset_));
	checkCudaErrors(cudaMemset(buffer_, 0, sizeof(eleType) * rows_ * cols_ * offset_));
	checkCudaErrors(cudaDeviceSynchronize());
	fr_ = true;
}

template <typename eleType>
void MatrixDevice<eleType>::memFree()
{
	if (fr_) {
		if (buffer_ != NULL) {
			checkCudaErrors(cudaFree(buffer_));
			buffer_ = NULL;
		}
	}
}

template class MatrixDevice<float>;
template class MatrixDevice<double>;

}
