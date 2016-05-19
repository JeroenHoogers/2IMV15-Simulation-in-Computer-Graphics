#include "Particle.h"
#include "IForce.h";

#include <vector>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)

void solveEuler(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IForce*> constraints, float dt);
void solveMidpoint(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IForce*> constraints, float dt);

void simulation_step(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IForce*> constraints, float dt)
{
	solveMidpoint(pVector, forces, constraints, dt);
	//solveEuler(pVector, forces, constraints, dt);
}

void solveEuler(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IForce*> constraints, float dt)
{
	// Apply forces
	for (int i = 0; i < forces.size(); i++)
	{
		// Apply force
		forces[i]->apply();
	}

	// TODO: solve constraints
	//for (int i = 0; i < constraints.size(); i++)
	//{
	//	// Apply force
	//	//constraints[i]->apply();
	//}


	// Loop particles
	for (int i = 0; i < pVector.size(); i++)
	{
		// Set new position
		pVector[i]->m_Position += dt * pVector[i]->m_Velocity;

		pVector[i]->m_Velocity += dt * (pVector[i]->m_Force / pVector[i]->m_Mass);

		// +Vec2f(RAND, RAND) * 0.005f;

		Vec2f veloc = pVector[i]->m_Velocity;
		Vec2f force = pVector[i]->m_Force;

		// reset force
		pVector[i]->m_Force = Vec2f(0, 0);
	}
}



void solveMidpoint(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IForce*> constraints, float dt)
{
	std::vector<Vec2f> positions = std::vector<Vec2f>();

	// Reset forces
	for (int i = 0; i < pVector.size(); i++)
	{
		// reset force
		pVector[i]->m_Force = Vec2f(0, 0);
		positions.push_back(pVector[i]->m_Position);
	}

	// Apply forces
	for (int i = 0; i < forces.size(); i++)
	{
		// Apply force
		forces[i]->apply();
	}

	for (int i = 0; i < pVector.size(); i++)
	{
		// reset force
		pVector[i]->m_Velocity += pVector[i]->m_Force * dt;
		pVector[i]->m_Position += (dt / 2) * pVector[i]->m_Velocity;

	}

	// Reset forces
	for (int i = 0; i < pVector.size(); i++)
	{
		// reset force
		pVector[i]->m_Force = Vec2f(0, 0);
	}

	// Apply forces
	for (int i = 0; i < forces.size(); i++)
	{
		// Apply force
		forces[i]->apply();
	}

	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity += pVector[i]->m_Force * dt;
		pVector[i]->m_Position = dt * pVector[i]->m_Velocity + positions[i];
	}
}

