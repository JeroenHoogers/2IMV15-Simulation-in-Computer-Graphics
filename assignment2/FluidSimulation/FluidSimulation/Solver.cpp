#include "Particle.h"
#include "IForce.h"
#include "FluidContainer.h"
#include "KernelFunctions.h"

#include <vector>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)


void solveEuler(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt);
void solveMidpoint(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt);
void solveRungeKutta(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt);

void applyForces(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces);

void calculateFluidDynamics(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float kd);

void simulation_step(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt, int method)
{
	switch (method)
	{
		case 0:
		default:
			solveEuler(pVector, fluidContainer, forces, dt);
			break;
		case 1:
			solveMidpoint(pVector, fluidContainer, forces, dt);
			break;
		case 2:
			solveRungeKutta(pVector, fluidContainer, forces, dt);
			break;
	}
}

//--------------------------------------------------------------
// Reset and apply forces to particles
//--------------------------------------------------------------
void applyForces(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces)
{
	// Reset forces
	for (int i = 0; i < pVector.size(); i++)
	{
		// reset force
		pVector[i]->m_Force = Vec2f(0, 0);
	}


	calculateFluidDynamics(pVector, fluidContainer, 1.0f);

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
void calculateFluidDynamics(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float kd)
{
	float dist = 0;
	 kd = 10.0f;		// Stiffness (higher = less compressable)	
	float mu = 8; // Viscosity Coefficient (lower = thicker fluids)
	float restDensity = 5.0f;

	// Updat the spatial hashing grid 
	fluidContainer->UpdateGrid(pVector);

	// Calculate particle densities, pressures and quantities
	for (int i = 0; i < pVector.size(); i++)
	{
		// TODO: iterating over cells might be faster since the neighbours will be the same for all particles within a cell
		// reset force
		//pVector[i]->m_Force = Vec2f(0, 0);

		// Assume A_j to be 1;
		pVector[i]->m_Density = 0;		// Rho_j
		pVector[i]->m_Quantity = 1;		// A_j
		pVector[i]->m_Pressure = 0;

		vector<int> neighbours = fluidContainer->FindNeighbours(pVector[i]->m_Position);

		int j = 0;

		//for (int j = 0; j < pVector.size(); j++)
		//{
		for (int k = 0; k < neighbours.size(); k++)
		{
			j = neighbours[k];

			// m * W(|r-r_j|,h)
			dist = pVector[j]->distTo(pVector[i]->m_Position);

			//pVector[i]->m_Density += pVector[j]->m_Mass * pVector[j]->getW(dist);
			pVector[i]->m_Density += pVector[j]->m_Mass * Kernels::getWPoly6(dist, pVector[j]->m_Radius);
		}
		
		// P_i = k(rho_i - restDensity_i)
		pVector[i]->m_Pressure = kd * (pVector[i]->m_Density - restDensity);
	}


	Vec2f pressureForce = 0;
	Vec2f viscocityForce = 0;
	float scalar = 0;


	dist = 0;
	// Calculate pressure force
	for (int i = 0; i < pVector.size(); i++)
	{
		pressureForce = 0;
		viscocityForce = 0;

		vector<int> neighbours = fluidContainer->FindNeighbours(pVector[i]->m_Position);

		int j = 0;

		//for (int j = 0; j < pVector.size(); j++)
		//{
		for (int k = 0; k < neighbours.size(); k++)
		{
			j = neighbours[k];
			// m_j * (p_i + p_j / 2 * rho_j) * WGrad(|r-r_j|,h)
			dist = pVector[j]->distTo(pVector[i]->m_Position);
			scalar = 0; 

			// calculate pressure force
			float scalar = (pVector[i]->m_Pressure + pVector[j]->m_Pressure) / (2 * pVector[j]->m_Density);
			pressureForce += pVector[j]->m_Mass * scalar * Kernels::getWGradSpiky(pVector[i]->m_Position - pVector[j]->m_Position, pVector[j]->m_Radius);
			//pressureForce += pVector[j]->m_Mass * scalar * pVector[j]->getWGrad(pVector[i]->m_Position - pVector[j]->m_Position);

			// calculate viscocity force
			Vec2f vscalar = (pVector[j]->m_Velocity - pVector[i]->m_Velocity) / (pVector[j]->m_Density);
			
			//viscocityForce += pVector[j]->m_Mass * vscalar *  pVector[j]->getWLaplacian(dist);
			viscocityForce += pVector[j]->m_Mass * vscalar * Kernels::getWViscosityLaplace(dist, pVector[j]->m_Radius);
		}

		// Add forces to the accumulator
		pVector[i]->m_Force += -pressureForce * 0.001f;

		pVector[i]->m_Force += mu * viscocityForce;
	}
}

//--------------------------------------------------------------
// Solve using Euler's scheme
//--------------------------------------------------------------
void solveEuler(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt)
{
	applyForces(pVector, fluidContainer, forces);

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
void solveMidpoint(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt)
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
	applyForces(pVector, fluidContainer, forces);

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
	applyForces(pVector, fluidContainer, forces);

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
void solveRungeKutta(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt)
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
	applyForces(pVector, fluidContainer, forces);

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
	applyForces(pVector, fluidContainer, forces);

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
	applyForces(pVector, fluidContainer, forces);

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
	applyForces(pVector, fluidContainer, forces);

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
