#include "Particle.h"
#include "IForce.h"
#include "FluidContainer.h"
#include "KernelFunctions.h"
#include "RigidBody.h"

#include <vector>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)


void solveEuler(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt);
void solveMidpoint(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt);
void solveRungeKutta(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces, float dt);

void applyForces(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<IForce*> forces);

void calculateFluidDynamics(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float kd);
void calculateColorField(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float radius);

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
// Calculate rigid body values
//--------------------------------------------------------------
void calculateRigidDynamics(std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces)
{

}


//--------------------------------------------------------------
// Calculate fluid values
//--------------------------------------------------------------
void calculateFluidDynamics(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float kd)
{
	float radius = pVector[0]->m_Radius;
	float dist = 0.0f;
	kd = 0.0141f;					// Stiffness (higher = less compressable)	
	float mu = 0.42f;				// Viscosity Coefficient (lower = thicker fluids)
	float restDensity = 15.0f;
	float sigma = 1.0f;				// Surface tension

	// Update the spatial hashing grid 
	fluidContainer->UpdateGrid(pVector);
	
	// Calculate particle densities, pressures and quantities
	for (int i = 0; i < pVector.size(); i++)
	{
		// TODO: iterating over cells might be faster since the neighbours will be the same for all particles within a cell
		// reset force
		//pVector[i]->m_Force = Vec2f(0, 0);

		// Assume A_j to be 1;
		pVector[i]->m_Radius = radius;
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
			pVector[i]->m_Density += pVector[j]->m_Mass * Kernels::getWPoly6(pVector[i]->m_Position - pVector[j]->m_Position, radius);
		}
		
		// P_i = k(rho_i - restDensity_i)
		//pVector[i]->m_Pressure = kd * (pow(pVector[i]->m_Density / restDensity, 7) -1.0f);

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
		Vec2f vscalar;
		float scalar;

		//for (int j = 0; j < pVector.size(); j++)
		//{
		for (int k = 0; k < neighbours.size(); k++)
		{
			j = neighbours[k];
			// m_j * (p_i + p_j / 2 * rho_j) * WGrad(|r-r_j|,h)
			dist = pVector[j]->distTo(pVector[i]->m_Position);

			// calculate pressure force
			scalar = (pVector[i]->m_Pressure + pVector[j]->m_Pressure) / (2.0f * pVector[j]->m_Density);
			pressureForce += pVector[j]->m_Mass * scalar * Kernels::getWGradSpiky(pVector[i]->m_Position - pVector[j]->m_Position, radius);

			// calculate viscocity force
			vscalar = (pVector[j]->m_Velocity - pVector[i]->m_Velocity) / (pVector[j]->m_Density);
			
			//viscocityForce += pVector[j]->m_Mass * vscalar *  pVector[j]->getWLaplacian(dist);
			viscocityForce += pVector[j]->m_Mass * vscalar * Kernels::getWViscosityLaplace(pVector[i]->m_Position - pVector[j]->m_Position, radius);
		}

		// Add forces to the accumulator
		pVector[i]->m_Force += -pressureForce;

		pVector[i]->m_Force += mu * viscocityForce;

		pVector[i]->m_Force += Vec2f(0.0f, -0.000981) * pVector[i]->m_Density;			// Gravity


		// Calculate surface tension
		//calculateColorField(pVector, fluidContainer, radius);
	}
}



void calculateColorField(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float radius)
{
	float sigma = 0.0f;

	float color = 0.0f;
	Vec2f colorGrad = Vec2f(0, 0);
	float colorLaplacian = 0.0f;

	float surfaceThreshold = 0.0f;

	Vec2f rDiff = Vec2f(0, 0);
	float scalar = 0.0f;



	// for each cell in fluidcontainer.
	for (int x = 0; x < fluidContainer->m_GridRows; x++)
	{
		for (int y = 0; y < fluidContainer->m_GridCols; y++)
		{
			Vec2f r = fluidContainer->m_CenterPoints[x][y];
			
			sigma = 0.0f;

			color = 0.0f;
			colorGrad = Vec2f(0, 0);
			colorLaplacian = 0.0f;

			surfaceThreshold = 0.0f;

			rDiff = Vec2f(0, 0);
			scalar = 0.0f;

			
			int j = 0;

			// Bottleneck
			//vector<int> neighbours = fluidContainer->FindInCell(r);

			//for (int k = 0; k < neighbours.size(); k++)
			//{
			//	j = neighbours[k];

			//	rDiff = r - pVector[j]->m_Position;

			//	// calculate color force
			//	scalar = 1.0f / pVector[j]->m_Density;

			//	color += pVector[j]->m_Mass * scalar * Kernels::getWPoly6(rDiff, radius);
			//		colorGrad += pVector[j]->m_Mass * scalar * Kernels::getWGradPoly6(rDiff, radius);
			//		colorLaplacian += pVector[j]->m_Mass * scalar * Kernels::getWLaplacePoly6(rDiff, radius);
			//}

			//Vec2f n = colorGrad;							//  n
			//float nLen = sqrt(n[0] * n[0] + n[1] * n[1]);	// |n|

			//float kappa = -colorLaplacian / nLen;

			//Vec2f surfaceTraction = sigma * kappa * (n / nLen);

			//// surfaceForce = Vec2f(0, 0);
			//if (nLen > surfaceThreshold)
			//{
			//	// apply force
			//	//Vec2f surfaceForce = ;

			//}

			fluidContainer->m_GridColors[x][y] = 0.2f;
		}
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
			// Symplectic Euler
			pVector[i]->m_Velocity += dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
			pVector[i]->m_Position += dt * pVector[i]->m_Velocity;

			// Explicit euler
			//pVector[i]->m_Position += dt * pVector[i]->m_Velocity;
			//pVector[i]->m_Velocity += dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
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
