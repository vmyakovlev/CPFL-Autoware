#ifndef GMATRIX_H_
#define GMATRIX_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include "common.h"
#include <float.h>

namespace gpu {

template <typename eleType = float>
class Matrix {
public:
	CUDAH Matrix();

	CUDAH Matrix(int rows, int cols, int offset, eleType *buffer);

	CUDAH int rows() const;

	CUDAH int cols() const;

	CUDAH int offset() const;

	CUDAH eleType *buffer() const;

	CUDAH void setRows(int rows);
	CUDAH void setCols(int cols);
	CUDAH void setOffset(int offset);
	CUDAH void setBuffer(eleType *buffer);
	CUDAH void setCellVal(int row, int col, eleType val);

	CUDAH void copy(Matrix &output);

	//Need to fix. Only reducing rows is OK now.
	CUDAH void resize(int rows, int cols);

	CUDAH eleType *cellAddr(int row, int col);

	CUDAH eleType *cellAddr(int index);

	//Assignment operator
	CUDAH void operator=(const Matrix<eleType> input);

	CUDAH eleType& operator()(int row, int col);

	CUDAH void set(int row, int col, eleType val);

	CUDAH eleType& operator()(int index);

	CUDAH eleType at(int row, int col) const;

	CUDAH bool operator*=(eleType val);

	CUDAH bool operator/=(eleType val);

	CUDAH bool transpose(Matrix<eleType> &output);

	//Only applicable for 3x3 matrix or below
	CUDAH bool inverse(Matrix<eleType> &output);

	CUDAH Matrix<eleType> col(int index);

	CUDAH Matrix<eleType> row(int index);

protected:
	eleType *buffer_;
	int rows_, cols_, offset_;
};


template <typename eleType>
CUDAH Matrix<eleType>::Matrix() {
	buffer_ = NULL;
	rows_ = cols_ = offset_ = 0;
}

template <typename eleType>
CUDAH Matrix<eleType>::Matrix(int rows, int cols, int offset, eleType *buffer) {
	rows_ = rows;
	cols_ = cols;
	offset_ = offset;
	buffer_ = buffer;
}

template <typename eleType>
CUDAH int Matrix<eleType>::rows() const {
	return rows_;
}

template <typename eleType>
CUDAH int Matrix<eleType>::cols() const {
	return cols_;
}

template <typename eleType>
CUDAH int Matrix<eleType>::offset() const {
	return offset_;
}

template <typename eleType>
CUDAH eleType *Matrix<eleType>::buffer() const {
	return buffer_;
}

template <typename eleType>
CUDAH void Matrix<eleType>::setRows(int rows) { rows_ = rows; }

template <typename eleType>
CUDAH void Matrix<eleType>::setCols(int cols) { cols_ = cols; }

template <typename eleType>
CUDAH void Matrix<eleType>::setOffset(int offset) { offset_ = offset; }

template <typename eleType>
CUDAH void Matrix<eleType>::setBuffer(eleType *buffer) { buffer_ = buffer; }

template <typename eleType>
CUDAH void Matrix<eleType>::setCellVal(int row, int col, eleType val) {
	buffer_[(row * cols_ + col) * offset_] = val;
}

template <typename eleType>
CUDAH void Matrix<eleType>::copy(Matrix &output) {
	for (int i = 0; i < rows_; i++) {
		for (int j = 0; j < cols_; j++) {
			output(i, j) = buffer_[(i * cols_ + j) * offset_];
		}
	}
}

//Need to fix. Only reducing rows is OK now.
template <typename eleType>
CUDAH void Matrix<eleType>::resize(int rows, int cols) {
	rows_ = rows;
	cols_ = cols;
}

template <typename eleType>
CUDAH eleType *Matrix<eleType>::cellAddr(int row, int col) {
	if (row >= rows_ || col >= cols_ || row < 0 || col < 0)
		return NULL;

	return buffer_ + (row * cols_ + col) * offset_;
}

template <typename eleType>
CUDAH eleType *Matrix<eleType>::cellAddr(int index) {
	if (rows_ == 1 && index >= 0 && index < cols_) {
			return buffer_ + index * offset_;
	}
	else if (cols_ == 1 && index >= 0 && index < rows_) {
			return buffer_ + index * offset_;
	}

	return NULL;
}

//Assignment operator
template <typename eleType>
CUDAH void Matrix<eleType>::operator=(const Matrix<eleType> input) {
	rows_ = input.rows_;
	cols_ = input.cols_;
	offset_ = input.offset_;
	buffer_ = input.buffer_;
}

template <typename eleType>
CUDAH eleType& Matrix<eleType>::operator()(int row, int col) {
	return buffer_[(row * cols_ + col) * offset_];
}

template <typename eleType>
CUDAH void Matrix<eleType>::set(int row, int col, eleType val) {
	buffer_[(row * cols_ + col) * offset_] = val;
}

template <typename eleType>
CUDAH eleType& Matrix<eleType>::operator()(int index) {
	return buffer_[index * offset_];
}

template <typename eleType>
CUDAH eleType Matrix<eleType>::at(int row, int col) const {
	return buffer_[(row * cols_ + col) * offset_];
}

template <typename eleType>
CUDAH bool Matrix<eleType>::operator*=(eleType val) {
	for (int i = 0; i < rows_; i++) {
		for (int j = 0; j < cols_; j++) {
			buffer_[(i * cols_ + j) * offset_] *= val;
		}
	}

	return true;
}

template <typename eleType>
CUDAH bool Matrix<eleType>::operator/=(eleType val) {
	if (val == 0)
		return false;

	for (int i = 0; i < rows_ * cols_; i++) {
			buffer_[i * offset_] /= val;
	}

	return true;
}

template <typename eleType>
CUDAH bool Matrix<eleType>::transpose(Matrix<eleType> &output) {
	if (rows_ != output.cols_ || cols_ != output.rows_)
		return false;

	for (int i = 0; i < rows_; i++) {
		for (int j = 0; j < cols_; j++) {
			output(j, i) = buffer_[(i * cols_ + j) * offset_];
		}
	}

	return true;
}

//Only applicable for 3x3 matrix or below
template <typename eleType>
CUDAH bool Matrix<eleType>::inverse(Matrix<eleType> &output) {
	if (rows_ != cols_ || rows_ == 0 || cols_ == 0)
		return false;

	if (rows_ == 1) {
		if (buffer_[0] != 0)
			output(0, 0) = 1 / buffer_[0];
		else
			return false;
	}

	if (rows_ == 2) {
		eleType det = at(0, 0) * at(1, 1) - at(0, 1) * at(1, 0);

		if (det != 0) {
			output(0, 0) = at(1, 1) / det;
			output(0, 1) = - at(0, 1) / det;

			output(1, 0) = - at(1, 0) / det;
			output(1, 1) = at(0, 0) / det;
		} else
			return false;
	}

	if (rows_ == 3) {
		eleType det = at(0, 0) * at(1, 1) * at(2, 2) + at(0, 1) * at(1, 2) * at(2, 0) + at(1, 0) * at (2, 1) * at(0, 2)
						- at(0, 2) * at(1, 1) * at(2, 0) - at(0, 1) * at(1, 0) * at(2, 2) - at(0, 0) * at(1, 2) * at(2, 1);
		eleType idet = 1.0 / det;

		if (det != 0) {
			output(0, 0) = (at(1, 1) * at(2, 2) - at(1, 2) * at(2, 1)) * idet;
			output(0, 1) = - (at(0, 1) * at(2, 2) - at(0, 2) * at(2, 1)) * idet;
			output(0, 2) = (at(0, 1) * at(1, 2) - at(0, 2) * at(1, 1)) * idet;

			output(1, 0) = - (at(1, 0) * at(2, 2) - at(1, 2) * at(2, 0)) * idet;
			output(1, 1) = (at(0, 0) * at(2, 2) - at(0, 2) * at(2, 0)) * idet;
			output(1, 2) = - (at(0, 0) * at(1, 2) - at(0, 2) * at(1, 0)) * idet;

			output(2, 0) = (at(1, 0) * at(2, 1) - at(1, 1) * at(2, 0)) * idet;
			output(2, 1) = - (at(0, 0) * at(2, 1) - at(0, 1) * at(2, 0)) * idet;
			output(2, 2) = (at(0, 0) * at(1, 1) - at(0, 1) * at(1, 0)) * idet;
		} else
			return false;
	}

	return true;
}

template <typename eleType>
CUDAH Matrix<eleType> Matrix<eleType>::col(int index) {
	return Matrix<eleType>(rows_, 1, offset_ * cols_, buffer_ + index * offset_);
}

template <typename eleType>
CUDAH Matrix<eleType> Matrix<eleType>::row(int index) {
	return Matrix<eleType>(1, cols_, offset_, buffer_ + index * cols_ * offset_);
}

}

#endif
