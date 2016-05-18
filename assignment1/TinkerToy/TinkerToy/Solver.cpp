#include "Particle.h"
#include "Force.h";

#include <vector>

#define DAMP 0.1f
#define RAND (((rand()%2000)/1000.f)-1.f)

void simulation_step(std::vector<Particle*> pVector, std::vector<Force*> forces, float dt)
{
	int ii, size = pVector.size();
	
	for(ii=0; ii < size; ii++)
	{
		// Set new position
		pVector[ii]->m_Position += dt * pVector[ii]->m_Velocity;
		
		
		pVector[ii]->m_Velocity += dt * pVector[ii]->m_Force / pVector[ii]->m_Mass;// +Vec2f(RAND, RAND) * 0.005f;
	}

	size = forces.size();
	for (ii = 0; ii < size; ii++)
	{
		// Apply force
		forces[ii]->apply();
	}
}

