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

void matrix::matVecMult(double x[], double r[])
{
	for (int i = 0; i < m_rows; i++) 
	{
		r[i] = 0;
		for (int j = 0; j < m_cols; j++) 
		{
			double dat = m_data[i][j];
			r[i] += m_data[i][j] * x[j];
		}
	}
}

matrix& matrix::operator+ (const matrix& rhs)
{
	return (*this += rhs);
}
 
matrix& matrix::operator- ( const matrix& rhs)
{
	return (*this -= rhs);
}

matrix& matrix::operator* (const matrix& rhs)
{
	return (*this *= rhs);
}

matrix& matrix::operator* (const float& rhs)
{
	return (*this *= rhs);
}


matrix& matrix::operator+= (const matrix& rhs)
{
	for (int i = 0; i < m_rows; i++)
	{
		for (int j = 0; j < m_cols; j++)
		{
			m_data[i][j] += rhs.m_data[i][j];
		}
	}

	return *this;
}

matrix& matrix::operator-= (const matrix& rhs)
{
	for (int i = 0; i < m_rows; i++)
	{
		for (int j = 0; j < m_cols; j++)
		{
			m_data[i][j] -= rhs.m_data[i][j];
		}
	}

	return *this;
}


matrix& matrix::operator*= (const matrix& rhs)
{
	matrix out = matrix(m_rows, rhs.m_cols);

	for (int i = 0; i < m_rows; ++i)
	{
		for (int j = 0; j < rhs.m_cols; ++j)
		{
			for (int k = 0; k < m_cols; ++k)
			{
				out.m_data[i][j] += (m_data[i][k] * rhs.m_data[k][i]);
			}
		}
	}

	*this = out;
	return *this;
}

matrix& matrix::operator*= (const float& rhs)
{
	for (int i = 0; i < m_rows; ++i)
	{
		for (int k = 0; k < m_cols; ++k)
		{
			m_data[i][k] *= rhs;
		}
	}

	return *this;
}


//
//void matrix::matTransVecMult(double x[], double r[])
//{
//
//}

