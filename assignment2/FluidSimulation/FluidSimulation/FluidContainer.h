#pragma once
#include <vector>
#include <unordered_map>
#include "Particle.h"
#include "GLUtil.h"

class FluidContainer
{
private:

	float _sceneWidth;
	float _sceneHeight;

	float _cellSize;
	int* _gridCounters;

	// Rendering variables
	float _radius;
	int _textureSize;

	GLuint _fbo;
	GLuint _fluidTexture;
	GLuint _particleTexture;

	float _threshold;

	vector<int>* _gridCells;

public:

	vector<Particle*> m_BoundaryParticles;

	vector<int>* m_Neighbours;

	FluidContainer(float radius, float sceneWidth, float sceneHeight);
	~FluidContainer();

	void UpdateGrid(vector<Particle*> particles);
	void FindNeighbours(int id);

	void SetupFluidRendering();
	void RenderFluid(vector<Particle*> particles, float radius);
	void RenderFluidToTexture(vector<Particle*> particles, float radius);
	float* GenerateParticleTexture(int particleSize);

	int CalcHash(Vec2f position);

	int m_GridRows;
	int m_GridCols;

	//float** m_GridColors;
	Vec2f* m_CenterPoints;

	void setColor(Vec2f point, float color);

	void ClearGrid();

	void draw();
};

