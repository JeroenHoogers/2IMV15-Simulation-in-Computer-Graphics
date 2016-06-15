#include "FluidContainer.h"
#include "GLUtil.h"

using namespace std;

FluidContainer::FluidContainer(float radius, float sceneWidth, float sceneHeight)
{
	_cellSize = radius;

	_sceneWidth = sceneWidth;
	_sceneHeight = sceneHeight;

	m_GridRows = floorf(sceneHeight / _cellSize);
	m_GridCols = floorf(sceneWidth / _cellSize);

	_gridCounters = new int*[m_GridRows];
	m_GridColors = new float*[m_GridRows];
	_gridCells = new vector<int>*[m_GridRows];

	m_CenterPoints = new Vec2f*[m_GridRows];

	for (int i = 0; i < m_GridRows; i++)
	{
		_gridCounters[i] = new int[m_GridCols];
		m_GridColors[i] = new float[m_GridCols];
		_gridCells[i] = new vector<int>[m_GridCols];

		m_CenterPoints[i] = new Vec2f[m_GridCols];


		for (int j = 0; j < m_GridCols; j++)
		{
			_gridCounters[i][j] = 0;
			m_GridColors[i][j] = 0;
			_gridCells[i][j] = vector<int>();

			// calculate centerpoints
			m_CenterPoints[i][j] = Vec2f(
				(i * _cellSize) + (_cellSize / 2) - 1,
				(j * _cellSize) + (_cellSize / 2) - 1);

		}
	}
}

FluidContainer::~FluidContainer()
{
	//// Release memory
	//for (int i = 0; i < _gridRows; i++)
	//{
	//	//for (int j = 0; j < _gridSize; j++)
	//	//{
	//	//	delete[] _gridCells[i][j];
	//	//}

	//	delete[] _gridCounters[i];
	//	delete[] _gridCells[i];
	//	delete[] _gridColors[i];
	//	delete[] _centerPoints[i];
	//}

	//delete[] _gridCounters;
	//delete[] _gridCells;
	//delete[] _gridColors;
	//delete[] _centerPoints;
}

void FluidContainer::UpdateGrid(vector<Particle*> particles)
{
	ClearGrid();

	for (int i = 0; i < particles.size(); i++)
	{

		int x = floorf((particles[i]->m_Position[0] / _cellSize) + (m_GridCols / 2.0f));
		int y = floorf((particles[i]->m_Position[1] / _cellSize) + (m_GridRows / 2.0f));

		// Check if this cell is within the grid
		if (!(x < 0 || y < 0 || x >= m_GridCols || y >= m_GridCols))
		{
			_gridCounters[x][y]++;
			_gridCells[x][y].push_back(i);
		}
	}
}

vector<int> FluidContainer::FindNeighbours(Vec2f position)
{
	vector<int> ids = vector<int>();

	//float width = _sceneWidth / _cellSize;
	int centerX = floorf((position[0] / _cellSize) + (m_GridCols / 2.0f));
	int centerY = floorf((position[1] / _cellSize) + (m_GridRows / 2.0f));

	int x = 0;
	int y = 0;

	for (int i = 0; i < 9; i++)
	{
		x = centerX + (i % 3) - 1;
		y = centerY + int(i / 3) - 1;

		// Check if this cell is within the grid
		if (!(x < 0 || y < 0 || x >= m_GridCols || y >= m_GridCols))
		{
			//// Check if this cell contains particles
			if (_gridCounters[x][y] > 0)
			{
				// Add particle ids
				for (int j = 0; j < _gridCells[x][y].size(); j++)
				{
					ids.push_back(_gridCells[x][y].at(j));
				}
			}
			//ids.insert(ids.end(), _gridCells[x][y].begin(), _gridCells[x][y].end());
		}
	}

	return ids;
}


vector<int> FluidContainer::FindInCell(Vec2f position)
{
	vector<int> ids = vector<int>();

	//float width = _sceneWidth / _cellSize;
	int x = floorf((position[0] / _cellSize) + (m_GridCols / 2.0f));
	int y = floorf((position[1] / _cellSize) + (m_GridRows / 2.0f));

	// Check if this cell is within the grid
	if (!(x < 0 || y < 0 || x >= m_GridCols || y >= m_GridCols))
	{
		// Check if this cell contains particles
		if (_gridCounters[x][y] > 0)
		{
			// Add particle ids
			for (int j = 0; j < _gridCells[x][y].size(); j++)
			{
				ids.push_back(_gridCells[x][y].at(j));
			}
		}
	}

	return ids;
}

void FluidContainer::setColor(Vec2f position, float color)
{
	//float width = _sceneWidth / _cellSize;
	int x = floorf((position[0] / _cellSize) + (m_GridCols / 2.0f));
	int y = floorf((position[1] / _cellSize) + (m_GridRows / 2.0f));

	// Check if this cell is within the grid
	if (!(x < 0 || y < 0 || x >= m_GridCols || y >= m_GridCols))
	{
		m_GridColors[x][y] = color;
	}
}


void FluidContainer::ClearGrid()
{
	for (int i = 0; i < m_GridRows; i++)
	{
		for (int j = 0; j < m_GridCols; j++)
		{
			_gridCounters[i][j] = 0;
			_gridCells[i][j] = vector<int>();
		}
	}
}

void FluidContainer::draw()
{
	glColor4f(1, 1, 1, 0.1);

	//glLineWidth(5.0);
	glPointSize(2.0);
	glBegin(GL_POINTS);

	//glTranslatef(-1, -1, 0);
	for (int i = 0; i < m_GridRows; i++)
	{
		//glVertex2f(-1, i * _cellSize - 1);
		//glVertex2f(_sceneWidth - 1, i * _cellSize - 1);

		for (int j = 0; j < m_GridCols; j++)
		{
			//if(_gridCounters[i][j] > 0)
			//{ 
				
				//glColor4f(1, 0.4, 0.4, _gridColors[i][j] / 3.0f);
				
				glVertex2f(m_CenterPoints[i][j][0], m_CenterPoints[i][j][1]);
				//glVertex2f(i * _cellSize - 1, (j + 1) * _cellSize - 1);
				//glVertex2f((i + 1) * _cellSize - 1, j * _cellSize - 1);
				//glVertex2f((i + 1) * _cellSize - 1, (j + 1) * _cellSize - 1);
				//glVertex2f(i * _cellSize - 1, j * _cellSize - 1);
			//}
		}
	}
	glColor4f(1, 1, 1, 0.1);
	for (int j = 0; j < m_GridCols; j++)
	{
		//glVertex2f(j * _cellSize - 1, -1);
		//glVertex2f(j * _cellSize - 1, _sceneHeight);
	}

	//glTranslatef(2, 2, 0);
	glEnd();
}
