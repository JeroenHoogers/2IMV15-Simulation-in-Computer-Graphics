#include "FluidContainer.h"

using namespace std;

FluidContainer::FluidContainer(float radius, float sceneWidth, float sceneHeight)
{
	_cellSize = radius;

	_sceneWidth = sceneWidth;
	_sceneHeight = sceneHeight;

	_gridRows = floorf(sceneHeight / _cellSize);
	_gridCols = floorf(sceneWidth / _cellSize);

	_gridCounters = new int*[_gridRows];
	_gridCells = new vector<int>*[_gridRows];

	for (int i = 0; i < _gridRows; i++)
	{
		_gridCounters[i] = new int[_gridCols];
		_gridCells[i] = new vector<int>[_gridCols];

		for (int j = 0; j < _gridCols; j++)
		{
			_gridCounters[i][j] = 0;
			_gridCells[i][j] = vector<int>();
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
	//}

	//delete[] _gridCounters;
	//delete[] _gridCells;
}

void FluidContainer::UpdateGrid(vector<Particle*> particles)
{
	ClearGrid();

	for (int i = 0; i < particles.size(); i++)
	{

		int x = floorf((particles[i]->m_Position[0] / _cellSize) + (_gridCols / 2.0f));
		int y = floorf((particles[i]->m_Position[1] / _cellSize) + (_gridRows / 2.0f));

		// Check if this cell is within the grid
		if (!(x < 0 || y < 0 || x >= _gridCols || y >= _gridCols))
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
	int centerX = floorf((position[0] / _cellSize) + (_gridCols / 2.0f));
	int centerY = floorf((position[1] / _cellSize) + (_gridRows / 2.0f));

	int x = 0;
	int y = 0;

	for (int i = 0; i < 9; i++)
	{
		x = centerX + (i % 3) - 1;
		y = centerY + int(i / 3) - 1;

		// Check if this cell is within the grid
		if (!(x < 0 || y < 0 || x >= _gridCols || y >= _gridCols))
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

			//ids.insert(ids.end(), _gridCells[x][y].begin(), _gridCells[x][y].begin());
		}
	}

	return ids;
}

void FluidContainer::ClearGrid()
{
	for (int i = 0; i < _gridRows; i++)
	{
		for (int j = 0; j < _gridCols; j++)
		{
			_gridCounters[i][j] = 0;
			_gridCells[i][j] = vector<int>();
		}
	}
}