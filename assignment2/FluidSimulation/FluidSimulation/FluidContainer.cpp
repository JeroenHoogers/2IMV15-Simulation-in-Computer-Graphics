#include "FluidContainer.h"
#include "GLUtil.h"
#include <algorithm>

using namespace std;

FluidContainer::FluidContainer(float radius, float sceneWidth, float sceneHeight)
{
	_cellSize = radius;

	_sceneWidth = sceneWidth;
	_sceneHeight = sceneHeight;

	m_GridRows = floorf(sceneHeight / _cellSize);
	m_GridCols = floorf(sceneWidth / _cellSize);

	_gridCounters = new int[m_GridRows * m_GridCols];
	//m_GridColors = new float[m_GridRows];
	_gridCells = new vector<int>[m_GridRows * m_GridCols];
	m_Neighbours = new vector<int>[m_GridRows * m_GridCols];

	m_CenterPoints = new Vec2f[m_GridRows * m_GridCols];

	for (int i = 0; i < m_GridRows * m_GridCols; i++)
	{
		//_gridCounters[i] = new int[m_GridCols];
		//m_GridColors[i] = new float[m_GridCols];
		//_gridCells[i] = new vector<int>[m_GridCols];

		//m_CenterPoints[i] = new Vec2f[m_GridCols];
		_gridCounters[i] = 0;
		_gridCells[i] = vector<int>();
		m_Neighbours[i] = vector<int>(90, -1);


		//for (int j = 0; j < m_GridCols; j++)
		//{
		//	_gridCounters[i][j] = 0;
		//	m_GridColors[i][j] = 0;
		//	//_gridCells[i][j] = vector<int>();

		// calculate centerpoints
		m_CenterPoints[i] = Vec2f(
			((i % m_GridCols) * _cellSize) + (_cellSize / 2) - 1,
			(int(i / m_GridCols) * _cellSize) + (_cellSize / 2) - 1);

		//}
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

#include <time.h>

void FluidContainer::UpdateGrid(vector<Particle*> particles)
{
	//ClearGrid();
	//time_t begin_time = clock();
	for (int i = 0; i < particles.size(); i++)
	{
		int id = CalcHash(particles[i]->m_Position);
		
		// Check if this particle moved to another cell since last timestep
		if (id != particles[i]->m_GridId)
		{
			if (particles[i]->m_GridId != -1)
			{
				// Remove from old cell
				_gridCells[particles[i]->m_GridId].erase(
					remove(_gridCells[particles[i]->m_GridId].begin(), _gridCells[particles[i]->m_GridId].end(), i), _gridCells[particles[i]->m_GridId].end()
					);
				_gridCounters[particles[i]->m_GridId]--;
			}

			particles[i]->m_GridId = id;

			if (!(id < 0 || id >= m_GridRows * m_GridCols))
			{
				// Add to new cell
				_gridCounters[id]++;
				_gridCells[id].push_back(i);
			}
			else
			{
				// Particle went off grid
				particles[i]->m_GridId = -1;
			}
		}
	}
	
	//time_t begin_time2 = clock();
	// Find neighbours for every cell w/ particle count > 0
	for (int i = 0; i < m_GridRows * m_GridCols; i++)
	{
		if (_gridCounters[i] > 0)
		{
			FindNeighbours(i);
		}
	}
	//std::cout << float(clock() - begin_time2) / CLOCKS_PER_SEC << std::endl;
}


int FluidContainer::CalcHash(Vec2f position)
{
	//float width = _sceneWidth / _cellSize;
	//int centerX = floorf((position[0] / _cellSize) + (m_GridCols / 2.0f));
	//int centerY = floorf((position[1] / _cellSize) + (m_GridRows / 2.0f));
	return (
		int((position[0] / _cellSize) + (m_GridCols / 2.0f)) +					// x
		int((position[1] / _cellSize) + (m_GridRows / 2.0f)) * m_GridCols		// y
	);
}

void FluidContainer::FindNeighbours(int id)
{
	m_Neighbours[id].clear();
	//int id = CalcHash(position);
	//float width = _sceneWidth / _cellSize;

	//int x = 0;
	//int y = 0;

	// move to left cell
	int nid = id - (m_GridCols + 1);
	for (int i = 0; i < 9; i++)
	{
		if (i != 0)
		{
			if (i % 3 == 0)
				nid += (m_GridCols - 2); // move y value and go back 2 places
			else
				nid++;
		}

		// Check if this cell is within the grid
		if (!(nid < 0 || nid >= m_GridRows * m_GridCols))
		{
			// Add particles
			for (int j = 0; j < _gridCells[nid].size(); j++)
			{
				m_Neighbours[id].push_back(_gridCells[nid].at(j));
				//ids.push_back(_gridCells[id].at(j));
			}
			//m_Neighbours[id].insert(m_Neighbours[id].end(), _gridCells[nid].begin(), _gridCells[nid].end());
		}
	}
	//return m_Neighbours[id];
	// return ids;
}

void FluidContainer::setColor(Vec2f position, float color)
{
	//float width = _sceneWidth / _cellSize;
	int x = floorf((position[0] / _cellSize) + (m_GridCols / 2.0f));
	int y = floorf((position[1] / _cellSize) + (m_GridRows / 2.0f));

	// Check if this cell is within the grid
	if (!(x < 0 || y < 0 || x >= m_GridCols || y >= m_GridCols))
	{
		//m_GridColors[x][y] = color;
	}
}


void FluidContainer::ClearGrid()
{
	for (int i = 0; i < m_GridRows * m_GridCols; i++)
	{
		if (_gridCounters[i] != 0)
		{
			//_gridCounters[i] = 0;
			//_gridCells[i] = vector<int>();
			m_Neighbours[i] = vector<int>();
		}
	}
}

void FluidContainer::draw()
{
	// Draw boundaries
	glLineWidth(4);
	glColor3f(0.3, 0.3, 0.3);
	glBegin(GL_LINE_LOOP);

	glVertex2f(-0.95f, -0.95f);
	glVertex2f(-0.95f, 0.94f);
	glVertex2f(0.94f, 0.94f);
	glVertex2f(0.94f, -0.95f);

	glEnd();
	glLineWidth(1);

	//glColor4f(1, 1, 1, 0.1);

	////glLineWidth(5.0);
	//glPointSize(6.0);


	////glTranslatef(-1, -1, 0);
	//for (int i = 0; i < m_GridRows * m_GridCols; i++)
	//{
	//	//glBegin(GL_LINES);
	//	//glVertex2f(-1, i * _cellSize - 1);
	//	//glVertex2f(_sceneWidth - 1, i * _cellSize - 1);
	//	//glEnd();

	//	glBegin(GL_POINTS);

	//	//if(_gridCounters[i] > 0)
	//	//{ 
	//	glColor4f(1, 1, 1, 0.1f + (_gridCounters[i] ));
	//			
	//	glVertex2f(m_CenterPoints[i][0], m_CenterPoints[i][1]);
	//	//}
	//	glEnd();
	//}
	//glBegin(GL_LINES);
	//glColor4f(1, 1, 1, 0.1);
	////for (int j = 0; j < m_GridCols; j++)
	////{
	////	glVertex2f(j * _cellSize - 1, -1);
	////	glVertex2f(j * _cellSize - 1, _sceneHeight);
	////}

	////glTranslatef(2, 2, 0);
	//glEnd();
}
