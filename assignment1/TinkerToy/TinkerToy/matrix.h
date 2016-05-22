#pragma once

#include <vector>
#include "linearSolver.h"

using namespace std;

class matrix : public implicitMatrix
{
	int m_rows;
	int m_cols;

	double** m_data;


public:


	matrix(int size);
	matrix(int row, int col);
	double getValue(int row, int col) { return m_data[col][row]; };
	double setValue(int row, int col, double value) { m_data[col][row] = value; };

	void matVecMult(double x[], double r[]) override;
	//void matTransVecMult(double x[], double r[]) override;

	matrix& operator+ (const matrix& rhs);
	matrix& operator* (const matrix& rhs);

	matrix& operator+= (const matrix& rhs);
	matrix& operator*= (const matrix& rhs);
};

