#include "Particle.h"
#include "IForce.h";
#include "IConstraint.h"

#include <vector>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)


void solveEuler(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IConstraint*> constraints, float dt);
void solveMidpoint(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IConstraint*> constraints, float dt);
void solveRungeKutta(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IConstraint*> constraints, float dt);

void applyForces(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IConstraint*> constraints);

void simulation_step(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IConstraint*> constraints, float dt)
{
	//solveMidpoint(pVector, forces, constraints, dt);
	solveEuler(pVector, forces, constraints, dt);
}

//--------------------------------------------------------------
// Reset and apply forces to particles
//--------------------------------------------------------------
void applyForces(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IConstraint*> constraints)
{
	// Reset forces
	for (int i = 0; i < pVector.size(); i++)
	{
		// reset force
		pVector[i]->m_Force = Vec2f(0, 0);
	}

	// Apply forces
	//int size = forces.size();
	for (int i = 0; i < forces.size(); i++)
	{
		// Apply force
		forces[i]->apply();
	}

	// TODO: solve constraints

}

//--------------------------------------------------------------
// Solve using Euler's scheme
//--------------------------------------------------------------
void solveEuler(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IConstraint*> constraints, float dt)
{
	applyForces(pVector, forces, constraints);

	// Loop particles
	for (int i = 1; i < pVector.size(); i++)
	{
		// Set new position
		pVector[i]->m_Position += dt * pVector[i]->m_Velocity;
		pVector[i]->m_Velocity += dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		

		// +Vec2f(RAND, RAND) * 0.005f;
	}
}

//--------------------------------------------------------------
// Solve using the midpoint scheme
//--------------------------------------------------------------
void solveMidpoint(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IConstraint*> constraints, float dt)
{
	std::vector<Vec2f> startPositions = std::vector<Vec2f>();
	std::vector<Vec2f> startVelocities = std::vector<Vec2f>();

	
	for (int i = 0; i < pVector.size(); i++)
	{
		// Store starting positions
		startPositions.push_back(pVector[i]->m_Position);
		startVelocities.push_back(pVector[i]->m_Velocity);
	}

	// Apply forces
	applyForces(pVector, forces, constraints);

	for (int i = 0; i < pVector.size(); i++)
	{
		// Update velocity
		pVector[i]->m_Velocity += pVector[i]->m_Force * dt;

		// Position particles halfway of the next timestep
		pVector[i]->m_Position += pVector[i]->m_Velocity * (dt / 2);
	}

	// Apply forces
	applyForces(pVector, forces, constraints);

	for (int i = 0; i < pVector.size(); i++)
	{
		pVector[i]->m_Velocity = startVelocities[i] + (pVector[i]->m_Force / pVector[i]->m_Mass) * dt;
		pVector[i]->m_Position = startPositions[i] + pVector[i]->m_Velocity * dt;
	}
}

//--------------------------------------------------------------
// Solve using Runge Kutta 4
//--------------------------------------------------------------
void solveRungeKutta(std::vector<Particle*> pVector, std::vector<IForce*> forces, std::vector<IConstraint*> constraints, float dt)
{
	std::vector<Vec2f> startPositions = std::vector<Vec2f>();
	std::vector<Vec2f> startVelocities = std::vector<Vec2f>();
}
