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

	if (lhs.m_rows != rhs.m_rows || lhs.m_cols != rhs.m_cols) {
		
		printf("Matrix substract: matrix dimentions are different");
		return out;
	}

	for (int i = 0; i < lhs.m_rows; i++)
	{
		for (int j = 0; j < lhs.m_cols; j++)
		{
			out.m_data[i][j] = lhs.m_data[i][j] - rhs.m_data[i][j];
		}
	}

	//int minRows = lhs.mRows < rhs.mRows ? lhs.mRows : rhs.mRows;
	//int minCols = lhs.mCols < rhs.mCols ? lhs.mCols : rhs.mCols;
	//Matrix output = Matrix::Matrix(minRows, minCols);

	//if (lhs.mRows != rhs.mRows || lhs.mCols != rhs.mCols) {
	//	printf("ERROR: Matrix subtract operator, matrix has no equal size");
	//	return output;
	//}

	//for (int i = 0; i < minRows; i++)
	//{
	//	for (int j = 0; j < minCols; j++)
	//	{
	//		output.mData[i][j] = lhs.mData[i][j] - rhs.mData[i][j];
	//	}
	//}

	//return output;

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

//	if (lhs.m_cols == rhs.m_rows)
//	{
//		for (int i = 0; i < rhs.m_rows; ++i)
//		{
//			for (int k = 0; k < lhs.m_cols; ++k)
//			{
//				p[i][k] *= T.p[k][i];
//			}
//		}
//	}
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

//
//matrix& matrix::operator* (const matrix& rhs)
//{
//	matrix out = *this;
//	return (out *= rhs);
//}

//matrix& matrix::operator* (const float& rhs)
//{
//	matrix out = *this;
//	return (out *= rhs);
//}

//
//matrix& matrix::operator+= (const matrix& rhs)
//{
//	for (int i = 0; i < m_rows; i++)
//	{
//		for (int j = 0; j < m_cols; j++)
//		{
//			m_data[i][j] += rhs.m_data[i][j];
//		}
//	}
//
//	return *this;
//}
//
//matrix& matrix::operator-= (const matrix& rhs)
//{
//	for (int i = 0; i < m_rows; i++)
//	{
//		for (int j = 0; j < m_cols; j++)
//		{
//			m_data[i][j] -= rhs.m_data[i][j];
//		}
//	}
//
//	return *this;
//}
//
//
//matrix& matrix::operator*= (const matrix& rhs)
//{
//	matrix out = matrix(m_rows, rhs.m_cols);
//
//	if (m_cols == rhs.m_rows)
//	{
//		for (int i = 0; i < rhs.m_rows; ++i)
//		{
//			for (int k = 0; k < m_cols; ++k)
//			{
//				m_data[i][k] *= rhs.m_data[k][i];
//			}
//		}
//	}
//	//*this = out;
//	return *this;
//}
//
//matrix& matrix::operator*= (const float& rhs)
//{
//	for (int i = 0; i < m_rows; ++i)
//	{
//		for (int k = 0; k < m_cols; ++k)
//		{
//			m_data[i][k] *= rhs;
//		}
//	}
//
//	return *this;
//}


//
//void matrix::matTransVecMult(double x[], double r[])
//{
//
//}

