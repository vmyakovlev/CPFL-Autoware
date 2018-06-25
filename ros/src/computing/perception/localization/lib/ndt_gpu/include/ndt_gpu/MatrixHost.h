#ifndef MATRIX_HOST_H_
#define MATRIX_HOST_H_

#include "Matrix.h"
#include "MatrixDevice.h"

namespace gpu {
template <typename eleType = float>
class MatrixHost : public Matrix<eleType> {
public:
	MatrixHost();
	MatrixHost(int rows, int cols);
	MatrixHost(int rows, int cols, int offset, eleType *buffer);
	MatrixHost(const MatrixHost<eleType>& other);

	bool moveToGpu(MatrixDevice<eleType> output);
	bool moveToHost(MatrixDevice<eleType> input);

	MatrixHost<eleType> &operator=(const MatrixHost<eleType> &other);

	void debug();

	~MatrixHost();
protected:
	using Matrix<eleType>::buffer_;
	using Matrix<eleType>::rows_;
	using Matrix<eleType>::cols_;
	using Matrix<eleType>::offset_;

private:
	bool fr_;
};

}

#endif
