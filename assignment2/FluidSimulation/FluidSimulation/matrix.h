#pragma once

#include <vector>
#include <iostream>
#include "linearSolver.h"

using namespace std;

class matrix : public implicitMatrix
{

public:

	int m_rows;
	int m_cols;

	double** m_data;

	matrix(int row, int col);
	
	void release();

	int getRows() { return m_rows; };
	int getCols() { return m_cols; };


	double getValue(int row, int col) { return m_data[row][col]; };
	void setValue(int row, int col, double value) { m_data[row][col] = value; };
	double* getData() { return *m_data; };
	void setData(double* data) { m_data = &data; };

	void printMatrix();

	void matVecMult(double x[], double r[]) override;
	//void matTransVecMult(double x[], double r[]) override;

	friend matrix operator-(const matrix lhs, const matrix rhs);
	friend matrix operator+(const matrix lhs, const matrix rhs);

	friend matrix operator*(const matrix lhs, const float rhs);
	friend matrix operator*(const matrix lhs, const matrix rhs);

	~matrix(void);

};

