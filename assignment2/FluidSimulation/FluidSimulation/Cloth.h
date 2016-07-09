#pragma once

#include "Particle.h"
#include <vector>

class Cloth
{
public:
	Cloth();
	~Cloth();
private:
	vector<Particle> m_ClothVertices;
	vector<Particle> m_GhostParticles;

	float getBoundaryLength();
};

