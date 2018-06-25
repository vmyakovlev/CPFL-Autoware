#ifndef MATRIX_DEVICE_H_
#define MATRIX_DEVICE_H_

#include "Matrix.h"

namespace gpu {
template <typename eleType = float>
class MatrixDevice : public Matrix<eleType> {
public:
	CUDAH MatrixDevice();

	MatrixDevice(int rows, int cols);

	CUDAH MatrixDevice(int rows, int cols, int offset, eleType *buffer);

	CUDAH bool isEmpty();

	CUDAH MatrixDevice<eleType> col(int index);

	CUDAH MatrixDevice<eleType> row(int index);

	CUDAH void setBuffer(eleType *buffer);

	void memAlloc();

	void memFree();
protected:
	using Matrix<eleType>::buffer_;
	using Matrix<eleType>::rows_;
	using Matrix<eleType>::cols_;
	using Matrix<eleType>::offset_;

private:
	bool fr_;
};

template <typename eleType>
CUDAH MatrixDevice<eleType>::MatrixDevice()
{
	rows_ = cols_ = offset_ = 0;
	buffer_ = NULL;
	fr_ = true;
}

template <typename eleType>
CUDAH MatrixDevice<eleType>::MatrixDevice(int rows, int cols, int offset, eleType *buffer)
{
	rows_ = rows;
	cols_ = cols;
	offset_ = offset;
	buffer_ = buffer;
	fr_ = false;
}

template <typename eleType>
CUDAH bool MatrixDevice<eleType>::isEmpty()
{
	return (rows_ == 0 || cols_ == 0 || buffer_ == NULL);
}

template <typename eleType>
CUDAH MatrixDevice<eleType> MatrixDevice<eleType>::col(int index)
{
	return MatrixDevice<eleType>(rows_, 1, offset_ * cols_, buffer_ + index * offset_);
}

template <typename eleType>
CUDAH MatrixDevice<eleType> MatrixDevice<eleType>::row(int index)
{
	return MatrixDevice<eleType>(1, cols_, offset_, buffer_ + index * cols_ * offset_);
}

template <typename eleType>
CUDAH void MatrixDevice<eleType>::setBuffer(eleType *buffer)
{
	buffer_ = buffer;
}

}

#endif

