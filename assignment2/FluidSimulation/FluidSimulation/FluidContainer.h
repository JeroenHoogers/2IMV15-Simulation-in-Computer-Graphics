#pragma once
#include <vector>
#include "Particle.h"

class FluidContainer
{
private:
	int _gridRows;
	int _gridCols;
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

	void ClearGrid();


};

