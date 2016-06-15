#pragma once
#include <vector>
#include "Particle.h"

class FluidContainer
{
private:

	float _sceneWidth;
	float _sceneHeight;

	float _cellSize;
	int** _gridCounters;

	vector<int>** _gridCells;
	
public:

	FluidContainer(float radius, float sceneWidth, float sceneHeight);
	~FluidContainer();

	void UpdateGrid(vector<Particle*> particles);
	vector<int> FindNeighbours(Vec2f position);
	vector<int> FindInCell(Vec2f position);

	int m_GridRows;
	int m_GridCols;

	float** m_GridColors;
	Vec2f** m_CenterPoints;

	void setColor(Vec2f point, float color);

	void ClearGrid();

	void draw();
};

