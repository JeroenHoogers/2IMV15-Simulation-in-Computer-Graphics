#include "matrix.h"

matrix::matrix(int rows, int cols)
{
	m_rows = rows;
	m_cols = cols;

	m_data = new double*[cols];

	for (int i = 0; i < cols; i++)
	{
		m_data[i] = new double[rows];
	}
}

void matrix::matVecMult(double x[], double r[])
{
	for (int i = 0; i < m_cols; i++) 
	{
		r[i] = 0;
		for (int j = 0; j < m_rows; j++) 
		{
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
	if (m_cols == rhs.m_rows)
	{
		for (int i = 0; i < rhs.m_rows; ++i)
		{
			for (int k = 0; k < m_cols; ++k)
			{
				m_data[i][k] *= rhs.m_data[k][i];
			}
		}
	}

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

