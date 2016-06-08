#include "Particle.h"
#include "IForce.h"

#include <vector>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)


void solveEuler(std::vector<Particle*> pVector, std::vector<IForce*> forces, float dt);
void solveMidpoint(std::vector<Particle*> pVector, std::vector<IForce*> forces, float dt);
void solveRungeKutta(std::vector<Particle*> pVector, std::vector<IForce*> forces, float dt);

void applyForces(std::vector<Particle*> pVector, std::vector<IForce*> forces);

void calculateFluidDynamics(std::vector<Particle*>);

void simulation_step(std::vector<Particle*> pVector, std::vector<IForce*> forces, float dt, int method)
{
	switch (method)
	{
		case 0:
		default:
			solveEuler(pVector, forces, dt);
			break;
		case 1:
			solveMidpoint(pVector, forces, dt);
			break;
		case 2:
			solveRungeKutta(pVector, forces, dt);
			break;
	}
}

//--------------------------------------------------------------
// Reset and apply forces to particles
//--------------------------------------------------------------
void applyForces(std::vector<Particle*> pVector, std::vector<IForce*> forces)
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


}


//--------------------------------------------------------------
// Calculate fluid values
//--------------------------------------------------------------
void calculateFluidDynamics(std::vector<Particle*> pVector)
{
	// Calculate parameters
	for (int i = 0; i < pVector.size(); i++)
	{
		// reset force
		pVector[i]->m_Force = Vec2f(0, 0);

		// Assume A_j to be 1;
		pVector[i]->m_Density = 0;		// Rho_j
		pVector[i]->m_Quantity = 1;		// A_j
		for (int j = 0; j < pVector.size(); j++)
		{
			pVector[i]->m_Density += pVector[j]->getDensityAt(pVector[i]->m_Position);
		}
	}
}

//--------------------------------------------------------------
// Solve using Euler's scheme
//--------------------------------------------------------------
void solveEuler(std::vector<Particle*> pVector, std::vector<IForce*> forces, float dt)
{
	applyForces(pVector, forces);

	// Loop particles
	for (int i = 0; i < pVector.size(); i++)
	{
		// Set new position
		if (!pVector[i]->m_isFixed)
		{
			pVector[i]->m_Position += dt * pVector[i]->m_Velocity;
			pVector[i]->m_Velocity += dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		}
		

		// +Vec2f(RAND, RAND) * 0.005f;
	}
}

//--------------------------------------------------------------
// Solve using the midpoint scheme
//--------------------------------------------------------------
void solveMidpoint(std::vector<Particle*> pVector, std::vector<IForce*> forces, float dt)
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
	applyForces(pVector, forces);

	for (int i = 0; i < pVector.size(); i++)
	{
		if (!pVector[i]->m_isFixed)
		{
			// Update velocity
			pVector[i]->m_Velocity += (pVector[i]->m_Force / pVector[i]->m_Mass) * dt;

			// Position particles halfway of the next timestep
			pVector[i]->m_Position += pVector[i]->m_Velocity * (dt / 2);
		}
	}

	// Apply forces
	applyForces(pVector, forces);

	for (int i = 0; i < pVector.size(); i++)
	{
		if (!pVector[i]->m_isFixed)
		{
			pVector[i]->m_Velocity = startVelocities[i] + (pVector[i]->m_Force / pVector[i]->m_Mass) * dt;
			pVector[i]->m_Position = startPositions[i] + pVector[i]->m_Velocity * dt;
		}
	}
}

//--------------------------------------------------------------
// Solve using Runge Kutta 4
//--------------------------------------------------------------
void solveRungeKutta(std::vector<Particle*> pVector, std::vector<IForce*> forces, float dt)
{
	std::vector<Vec2f> startPositions = std::vector<Vec2f>();
	std::vector<Vec2f> startVelocities = std::vector<Vec2f>();
	std::vector<Vec2f> k1 = std::vector<Vec2f>();
	std::vector<Vec2f> k2 = std::vector<Vec2f>();
	std::vector<Vec2f> k3 = std::vector<Vec2f>();
	std::vector<Vec2f> k4 = std::vector<Vec2f>();
	std::vector<Vec2f> vel1 = std::vector<Vec2f>();
	std::vector<Vec2f> vel2 = std::vector<Vec2f>();
	std::vector<Vec2f> vel3 = std::vector<Vec2f>();
	std::vector<Vec2f> vel4 = std::vector<Vec2f>();


	for (int i = 0; i < pVector.size(); i++)
	{
		// Store starting positions
		startPositions.push_back(pVector[i]->m_Position);
		startVelocities.push_back(pVector[i]->m_Velocity);
		k1.push_back(Vec2f(0, 0));
		k2.push_back(Vec2f(0, 0));
		k3.push_back(Vec2f(0, 0));
		k4.push_back(Vec2f(0, 0));
		vel1.push_back(Vec2f(0, 0));
		vel2.push_back(Vec2f(0, 0));
		vel3.push_back(Vec2f(0, 0));
		vel4.push_back(Vec2f(0, 0));
	}

	// Apply forces
	applyForces(pVector, forces);

	for (int i = 0; i < pVector.size(); i++)
	{
		if (!pVector[i]->m_isFixed)
		{
			// Update velocity
			k1[i] = pVector[i]->m_Force / pVector[i]->m_Mass;
			vel1[i] = pVector[i]->m_Velocity;

			// Position particles on the beginning of the next timestep
			pVector[i]->m_Velocity += k1[i] * (dt / 2);
			pVector[i]->m_Position += vel1[i] * (dt / 2);
		}
	}

	// Apply forces
	applyForces(pVector, forces);

	for (int i = 0; i < pVector.size(); i++)
	{
		if (!pVector[i]->m_isFixed)
		{

			k2[i] = pVector[i]->m_Force / pVector[i]->m_Mass;
			vel2[i] = pVector[i]->m_Velocity;

			// Position particles on the midpoint of the next timestep
			pVector[i]->m_Velocity += k2[i] * (dt / 2);
			pVector[i]->m_Position += vel2[i] * (dt / 2);
		}
	}

	// Apply forces
	applyForces(pVector, forces);

	for (int i = 0; i < pVector.size(); i++)
	{
		if (!pVector[i]->m_isFixed)
		{
			k3[i] = pVector[i]->m_Force / pVector[i]->m_Mass;
			vel3[i] = pVector[i]->m_Velocity;

			// Position particles on the midpoint of the next timestep
			pVector[i]->m_Velocity += k3[i] * dt;
			pVector[i]->m_Position += vel3[i] * dt;
		}
	}

	// Apply forces
	applyForces(pVector, forces);

	for (int i = 0; i < pVector.size(); i++)
	{
		if (!pVector[i]->m_isFixed)
		{
			k4[i] = pVector[i]->m_Force / pVector[i]->m_Mass;
			vel4[i] = pVector[i]->m_Velocity;

			// Position particles on the end of the next timestep
			pVector[i]->m_Velocity = startVelocities[i] + dt / 6.0f * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
			pVector[i]->m_Position = startPositions[i] + dt / 6.0f * (vel1[i] + 2.0f * vel2[i] + 2.0f * vel3[i] + vel4[i]);
		}
	}
}
