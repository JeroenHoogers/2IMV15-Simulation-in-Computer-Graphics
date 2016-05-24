#include "matrix.h"

matrix::matrix(int rows, int cols)
{
	m_rows = rows;
	m_cols = cols;

	m_data = new double*[rows];

	for (int i = 0; i < rows; i++)
	{
		m_data[i] = new double[cols];
		for (int j = 0; j < cols; j++)
		{
			m_data[i][j] = 0;
		}
	}
}

matrix::~matrix(void)
{ 
	//for (int i = 0; i < m_rows; i++)
	//{
	//	delete[] m_data[i];
	//}

	//delete[] m_data;
	//free(m_data);
}

void matrix::release()
{
	//delete[] m_data;

	for (int i = 0; i < m_rows; i++)
	{
		delete[] m_data[i];
	}

	delete[] m_data;
}

void matrix::matVecMult(double x[], double r[])
{
	for (int i = 0; i < m_rows; i++) 
	{
		double val = 0;

		for (int j = 0; j < m_cols; j++) 
		{
			//double dat = m_data[i][j];
			val += m_data[i][j] * x[j];
		}

		r[i] = val;
	}
}

void matrix::printMatrix()
{
	for (int i = 0; i < m_rows; i++)
	{
		for (int j = 0; j < m_cols; j++)
		{
			cout << m_data[i][j] << " ";
		}
		cout << endl;
	}
}
//
//matrix& matrix::operator+ (const matrix& rhs)
//{
//	matrix out = *this;
//	return (out += rhs);
//}
// 
//matrix& matrix::operator- ( const matrix& rhs)
//{
//	matrix out = *this;
//	return (out -= rhs);
//}

matrix operator-(matrix lhs, const matrix rhs)
{
	matrix out = matrix(lhs.m_rows, lhs.m_cols);

	for (int i = 0; i < lhs.m_rows; i++)
	{
		for (int j = 0; j < lhs.m_cols; j++)
		{
			out.m_data[i][j] = lhs.m_data[i][j] - rhs.m_data[i][j];
		}
	}

	return out;
}


matrix operator+(matrix lhs, const matrix rhs)
{
	matrix out = matrix(lhs.m_rows, lhs.m_cols);

	for (int i = 0; i < lhs.m_rows; i++)
	{
		for (int j = 0; j < lhs.m_cols; j++)
		{
			out.m_data[i][j] = lhs.m_data[i][j] + rhs.m_data[i][j];
		}
	}

	return out;
}


matrix operator*(matrix lhs, const matrix rhs)
{
	matrix out = matrix(lhs.m_rows, rhs.m_cols);
	
	for (int i = 0; i < lhs.m_rows; ++i)
	{
		for (int j = 0; j < rhs.m_cols; ++j)
		{
			for (int k = 0; k < lhs.m_cols; ++k)
			{
				out.m_data[i][j] += (lhs.m_data[i][k] * rhs.m_data[k][j]);
			}
		}
	}

	return out;
}

matrix operator*(matrix lhs, const float rhs)
{
	matrix out = matrix(lhs.m_rows, lhs.m_cols);

	for (int i = 0; i < lhs.m_rows; ++i)
	{
		for (int k = 0; k < lhs.m_cols; ++k)
		{
			out.m_data[i][k] = lhs.m_data[i][k] *= rhs;
		}
	}

	return out;
}
