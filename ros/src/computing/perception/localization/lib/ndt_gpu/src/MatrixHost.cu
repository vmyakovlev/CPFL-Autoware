#include "ndt_gpu/common.h"
#include "ndt_gpu/debug.h"
#include "ndt_gpu/MatrixHost.h"
#include <iostream>

namespace gpu {

template <typename eleType>
MatrixHost<eleType>::MatrixHost()
{
	fr_ = false;
}

template <typename eleType>
MatrixHost<eleType>::MatrixHost(int rows, int cols) {
	rows_ = rows;
	cols_ = cols;
	offset_ = 1;

	buffer_ = (eleType*)malloc(sizeof(eleType) * rows_ * cols_ * offset_);
	memset(buffer_, 0, sizeof(eleType) * rows_ * cols_ * offset_);
	fr_ = true;
}

template <typename eleType>
MatrixHost<eleType>::MatrixHost(int rows, int cols, int offset, eleType *buffer)
{
	rows_ = rows;
	cols_ = cols;
	offset_ = offset;
	buffer_ = buffer;
	fr_ = false;
}

template <typename eleType>
MatrixHost<eleType>::MatrixHost(const MatrixHost<eleType>& other) {
	rows_ = other.rows_;
	cols_ = other.cols_;
	offset_ = other.offset_;
	fr_ = other.fr_;

	if (fr_) {
		buffer_ = (eleType*)malloc(sizeof(eleType) * rows_ * cols_ * offset_);
		memcpy(buffer_, other.buffer_, sizeof(eleType) * rows_ * cols_ * offset_);
	} else {
		buffer_ = other.buffer_;
	}
}

template <typename eleType>
__global__ void copyMatrixDevToDev(MatrixDevice<eleType> input, MatrixDevice<eleType> output) {
	int row = threadIdx.x;
	int col = threadIdx.y;
	int rows_num = input.rows();
	int cols_num = input.cols();

	if (row < rows_num && col < cols_num)
		output(row, col) = input(row, col);
}

template <typename eleType>
bool MatrixHost<eleType>::moveToGpu(MatrixDevice<eleType> output) {
	if (rows_ != output.rows() || cols_ != output.cols())
		return false;

	if (offset_ == output.offset()) {
		checkCudaErrors(cudaMemcpy(output.buffer(), buffer_, sizeof(eleType) * rows_ * cols_ * offset_, cudaMemcpyHostToDevice));
		return true;
	}
	else {
		eleType *tmp;

		checkCudaErrors(cudaMalloc(&tmp, sizeof(eleType) * rows_ * cols_ * offset_));
		checkCudaErrors(cudaMemcpy(tmp, buffer_, sizeof(eleType) * rows_ * cols_ * offset_, cudaMemcpyHostToDevice));

		MatrixDevice<eleType> tmp_output(rows_, cols_, offset_, tmp);

		dim3 block_x(rows_, cols_, 1);
		dim3 grid_x(1, 1, 1);

		copyMatrixDevToDev<eleType><<<grid_x, block_x>>>(tmp_output, output);
		checkCudaErrors(cudaDeviceSynchronize());

		checkCudaErrors(cudaFree(tmp));

		return true;
	}
}

template <typename eleType>
bool MatrixHost<eleType>::moveToHost(MatrixDevice<eleType> input) {
	if (rows_ != input.rows() || cols_ != input.cols())
		return false;

	if (offset_ == input.offset()) {
		checkCudaErrors(cudaMemcpy(buffer_, input.buffer(), sizeof(eleType) * rows_ * cols_ * offset_, cudaMemcpyDeviceToHost));
		return true;
	}
	else {
		eleType *tmp;

		checkCudaErrors(cudaMalloc(&tmp, sizeof(eleType) * rows_ * cols_ * offset_));

		MatrixDevice<eleType> tmp_output(rows_, cols_, offset_, tmp);

		dim3 block_x(rows_, cols_, 1);
		dim3 grid_x(1, 1, 1);

		copyMatrixDevToDev<eleType><<<grid_x, block_x>>>(input, tmp_output);
		checkCudaErrors(cudaDeviceSynchronize());

		checkCudaErrors(cudaMemcpy(buffer_, tmp, sizeof(eleType) * rows_ * cols_ * offset_, cudaMemcpyDeviceToHost));
		checkCudaErrors(cudaFree(tmp));

		return true;
	}
}

template <typename eleType>
MatrixHost<eleType> &MatrixHost<eleType>::operator=(const MatrixHost<eleType> &other)
{
	rows_ = other.rows_;
	cols_ = other.cols_;
	offset_ = other.offset_;
	fr_ = other.fr_;

	if (fr_) {
		buffer_ = (eleType*)malloc(sizeof(eleType) * rows_ * cols_ * offset_);
		memcpy(buffer_, other.buffer_, sizeof(eleType) * rows_ * cols_ * offset_);
	} else {
		buffer_ = other.buffer_;
	}

	return *this;
}

template <typename eleType>
void MatrixHost<eleType>::debug()
{
	for (int i = 0; i < rows_; i++) {
		for (int j = 0; j < cols_; j++) {
			std::cout << buffer_[(i * cols_ + j) * offset_] << " ";
		}

		std::cout << std::endl;
	}

	std::cout << std::endl;
}

template <typename eleType>
MatrixHost<eleType>::~MatrixHost()
{
	if (fr_)
		free(buffer_);
}

template class MatrixHost<float>;
template class MatrixHost<double>;

}
