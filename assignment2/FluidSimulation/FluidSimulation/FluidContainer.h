#pragma once
#include <vector>
#include <unordered_map>
#include "Particle.h"

class FluidContainer
{
private:

	float _sceneWidth;
	float _sceneHeight;

	float _cellSize;
	int* _gridCounters;

	vector<int>* _gridCells;

	unordered_multimap<int, int> _gridMap;
	

public:

	vector<int>* m_Neighbours;

	FluidContainer(float radius, float sceneWidth, float sceneHeight);
	~FluidContainer();

	void UpdateGrid(vector<Particle*> particles);
	void FindNeighbours(int id);

	int CalcHash(Vec2f position);

	int m_GridRows;
	int m_GridCols;

	//float** m_GridColors;
	//Vec2f** m_CenterPoints;

	void setColor(Vec2f point, float color);

	void ClearGrid();

	void draw();
};

