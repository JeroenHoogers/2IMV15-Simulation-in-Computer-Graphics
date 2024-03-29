#include "Particle.h"
#include "FluidContainer.h"
#include "KernelFunctions.h"
#include "RigidBody.h"
#include "Box.h"
#include "Utility.h"

#include <vector>
#include <time.h>
#include <thread>

using namespace std;


void calculateFluidDynamics(vector<Particle*> pVector, FluidContainer* fluidContainer, vector<RigidBody*> rigidBodies);

void calculateBoundaryVolumes(vector<Particle*> pVector, FluidContainer* fluidContainer, vector<RigidBody*> rigidBodies, float radius);
void calculateFluidDensities(vector<Particle*> pVector, FluidContainer* fluidContainer, float radius, float kd, float restDensity, int start, int end);
void calculateFluidForces(vector<Particle*> pVector, FluidContainer* fluidContainer, float radius, float mu, int start, int end);
void calculateRigidForces(FluidContainer* fluidContainer, vector<RigidBody*> rigidBodies);
void calculateSurfaceTension(vector<Particle*> pVector, FluidContainer* fluidContainer, float radius);

void renderFluid(vector<Particle*> pVector, float radius);

//-----------------------------------------------------------------------------------------------------------------//
void calculateFluidDynamics(vector<Particle*> pVector, FluidContainer* fluidContainer, vector<RigidBody*> rigidBodies)
{
	if (pVector.size() < 1)
		return;

	//------------------------
	// Fluid parameters
	//------------------------
	float radius = pVector[0]->m_Radius;

	//float kd = 0.005f;					// Stiffness (higher = less compressable)	
	//float mu = 1.5f;					// Viscosity Coefficient (higher = thicker fluids)
	//float restDensity = 210.0f;			// Rest density


	/// dist = 0.04
	float kd = 0.005f;					// Stiffness (higher = less compressable)	
	float mu = 2.0f;					// Viscosity Coefficient (higher = thicker fluids)
	float restDensity = 270.0f;			// Rest density

	int n = pVector.size();
	int nrThreads = 4;
	int start = 0;
	int end = 0;

	vector<thread> threads;

	// Update ghost particles of rigid body objects
	for (int i = 0; i < rigidBodies.size(); i++)
	{
		rigidBodies[i]->updateGhostParticles();
	}

	// Update the spatial hashing grid 
	fluidContainer->UpdateGrid(pVector);

	// Calculate Boundary Volumes
	calculateBoundaryVolumes(pVector, fluidContainer, rigidBodies, radius);

	// Calculate Fluid Densities
	for (int i = 0; i < nrThreads; i++)
	{
		start = i * (n / nrThreads);
		end = (i == nrThreads - 1) ? n : (i + 1) * (n / nrThreads);

		threads.push_back(thread(calculateFluidDensities, pVector, fluidContainer, radius, kd, restDensity, start, end));
	}

	for (int i = 0; i < nrThreads; i++)
		threads[i].join();

	threads.clear();


	//calculateFluidDensities(pVector, fluidContainer, radius, kd, restDensity, 0, n);

	// Calculate Fluid Forces
	for (int i = 0; i < nrThreads; i++)
	{
		start = i * (n / nrThreads);
		end = (i == nrThreads - 1) ? n : (i + 1) * (n / nrThreads);

		threads.push_back(thread(calculateFluidForces, pVector, fluidContainer, radius, mu, start, end));
	}

	for (int i = 0; i < nrThreads; i++)
		threads[i].join();

	threads.clear();

	// Calculate fluid -> rigid body forces
	calculateRigidForces(fluidContainer, rigidBodies);


	//calculateFluidForces(pVector, fluidContainer, radius, mu, 0, n);

	// Calculate surface tension
	//calculateSurfaceTension(pVector, fluidContainer, radius);
}


//-----------------------------------------------------------------------------------------------------------------//
void calculateBoundaryVolumes(vector<Particle*> pVector, FluidContainer* fluidContainer, vector<RigidBody*> rigidBodies, float radius)
{
	Particle pi = Particle(Vec2f(0, 0), 1.0f);
	Particle pj = Particle(Vec2f(0, 0), 1.0f);

	//// Calculate boundary particle boundary volumes
	//for (int b = 0; b < rigidBodies.size(); b++)
	//{
	//	for (int i = 0; i < rigidBodies[b]->m_GhostParticles.size(); i++)
	//	{
	//		// Calculate only boundary particle volumes in this step
	//		pi = *rigidBodies[b]->m_GhostParticles[i];
	//		if (pi.m_GridId != -1 && !pi.m_isBoundary)
	//		{
	//			float volume = 0;
	//			bool active = false;

	//			for (int j = 0; j < fluidContainer->m_Neighbours[pi.m_GridId].size(); j++)
	//			{
	//				pj = *pVector[fluidContainer->m_Neighbours[pi.m_GridId][j]];

	//				if (pj.m_isBoundary)
	//					volume += Kernels::getWPoly6(pi.m_Position - pj.m_Position, radius);
	//				else
	//					active = true;	// boundary particle has fluid neighbour(s) so we activate it.
	//			}

	//			// V_b_i = 1 / W_ik
	//			rigidBodies[b]->m_GhostParticles[i]->m_Volume = 1.0f / volume;
	//			rigidBodies[b]->m_GhostParticles[i]->m_isActive = active;
	//		}
	//	}
	//}

	for (int i = 0; i < pVector.size(); i++)
	{
		// Calculate only boundary particle volumes in this step
		pi = *pVector[i];
		if (pi.m_GridId != -1 && pi.m_isBoundary)
		{
			float volume = 0;
			bool active = false;

			for (int j = 0; j < fluidContainer->m_Neighbours[pi.m_GridId].size(); j++)
			{
				pj = *pVector[fluidContainer->m_Neighbours[pi.m_GridId][j]];

				if (pj.m_isBoundary)
					volume += Kernels::getWPoly6(pi.m_Position - pj.m_Position, radius);
				else
					active = true;	// boundary particle has fluid neighbour(s) so we activate it.
			}

			// V_b_i = 1 / W_ik
			pVector[i]->m_Volume = 1.0f / volume;
			pVector[i]->m_isActive = active;
		}
	}
}

//-----------------------------------------------------------------------------------------------------------------//
void calculateFluidDensities(vector<Particle*> pVector, FluidContainer* fluidContainer, float radius, float kd, float restDensity, int start, int end)
{
	Particle pi = Particle(Vec2f(0, 0), 1.0f);
	Particle pj = Particle(Vec2f(0, 0), 1.0f);

	// Calculate particle densities, pressures and quantities
	for (int i = start; i < end; i++)
	{
		pi = *pVector[i];

		// Reset previous variables
		float density = 0;
		float pressure = 0;

		if (pi.m_GridId != -1)// && (!pi.m_isBoundary || (pi.m_isBoundary && pi.m_isActive))) // TODO: Handle boundaries differently
		{
			for (int j = 0; j < fluidContainer->m_Neighbours[pi.m_GridId].size(); j++)
			{
				pj = *pVector[fluidContainer->m_Neighbours[pi.m_GridId][j]];

				// m * W(r-r_j,h)
				Vec2f posDiff = pi.m_Position - pj.m_Position;

				// Calculate fluid -> fluid density
				density += (!pj.m_isBoundary) * pj.m_Mass * Kernels::getWPoly6(posDiff, radius);

				// Calculate fluid -> boundary density
				density += pj.m_isBoundary * (restDensity * pj.m_Volume) * Kernels::getWPoly6(posDiff, radius);
			}
		}

		// P_i = k(rho_i - restDensity_i)
		float gamma = 1.3f;
		pVector[i]->m_Density = density;

		// 
		//pVector[i]->m_Pressure = kd * (density - restDensity);

		// Tait equation
		pVector[i]->m_Pressure = kd * restDensity * (pow((density / restDensity), gamma) - 1);
	}
}

//-----------------------------------------------------------------------------------------------------------------//
void calculateFluidForces(vector<Particle*> pVector, FluidContainer* fluidContainer, float radius, float mu, int start, int end)
{
	Particle pi = Particle(Vec2f(0, 0), 1.0f);
	Particle pj = Particle(Vec2f(0, 0), 1.0f);

	// Calculate forces
	for (int i = start; i < end; i++)
	{
		pi = *pVector[i];

		Vec2f pressureForce = 0;
		Vec2f viscocityForce = 0;

		//pVector[i]->m_Color = 0;

		radius = pi.m_Radius;

		Vec2f vscalar;
		float scalar = 0;


		if (pi.m_GridId != -1)
		{
			for (int j = 0; j < fluidContainer->m_Neighbours[pi.m_GridId].size(); j++)
			{
				pj = *pVector[fluidContainer->m_Neighbours[pi.m_GridId][j]];

				// m_j * (p_i + p_j / 2 * rho_j) * WGrad(|r-r_j|,h)
				Vec2f posDiff = pi.m_Position - pj.m_Position;

				// calculate pressure force
				scalar = (pi.m_Pressure + pj.m_Pressure) / (2.0f * pj.m_Density);
				pressureForce += pj.m_Mass * scalar * Kernels::getWGradSpiky(posDiff, radius);

				// calculate viscocity force
				vscalar = (pj.m_Velocity - pi.m_Velocity) / (pj.m_Density);
				/*	if (pVector[j]->m_isFixed)
				vscalar *= 0;*/

				viscocityForce += pj.m_Mass * vscalar * Kernels::getWViscosityLaplace(posDiff, radius);
			}
		}

		// Add forces to the accumulator
		pVector[i]->m_Force += -pressureForce;

		pVector[i]->m_Force += mu * viscocityForce;

		// Add gravity
		//if(!pVector[i]->m_isBoundary)
		pVector[i]->m_Force += Vec2f(0.0f, -0.000981) * pi.m_Density;			// Gravity
	}
}


//-----------------------------------------------------------------------------------------------------------------//
void calculateRigidForces(FluidContainer* fluidContainer, vector<RigidBody*> rigidBodies)
{
	Particle pi = Particle(Vec2f(0, 0), 1.0f);

	// Calculate forces
	for (int b = 0; b < rigidBodies.size(); b++)
	{
		Vec2f totalBoundaryForce = 0;
		float totalTorque = 0;

		for (int i = 0; i < rigidBodies[b]->m_GhostParticles.size(); i++)
		{
			pi = *rigidBodies[b]->m_GhostParticles[i];

			if (pi.m_isActive)
			{
				// Calculate total force and torque applied to the rigid body
				totalBoundaryForce += pi.m_Force;
				totalTorque += Util::crossProduct((pi.m_Position - rigidBodies[b]->m_Position), pi.m_Force);
			}
		}
		rigidBodies[b]->m_Force += totalBoundaryForce * 0.01f;
		rigidBodies[b]->m_Force += Vec2f(0.0f, -0.981) * rigidBodies[b]->m_Mass;
		rigidBodies[b]->m_Torque += totalTorque;
	}
}


//-----------------------------------------------------------------------------------------------------------------//
void calculateSurfaceTension(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float radius)
{
	// TODO: Use color field to identify surfaces
	float color = 0.0f;
	Vec2f colorGrad = Vec2f(0, 0);
	float colorLaplacian = 0.0f;

	float sigma = 0.01f;

	float surfaceThreshold = 0.0f;

	Vec2f rDiff = Vec2f(0, 0);
	float scalar = 0.0f;

	Particle pi = Particle(Vec2f(0, 0), 1.0f);
	Particle pj = Particle(Vec2f(0, 0), 1.0f);


	//// for each cell in fluidcontainer.
	//for (int x = 0; x < fluidContainer->m_GridRows; x++)
	//{
	//	for (int y = 0; y < fluidContainer->m_GridCols; y++)
	//	{
	for (int i = 0; i < pVector.size(); i++)
	{
		pi = *pVector[i];
		color = 0.0f;
		colorGrad = Vec2f(0, 0);
		colorLaplacian = 0.0f;

		surfaceThreshold = 0.01f;

		rDiff = Vec2f(0, 0);
		scalar = 0.0f;

		if (pi.m_GridId != -1)
		{
			// Calculate surface tension
			for (int j = 0; j < fluidContainer->m_Neighbours[pi.m_GridId].size(); j++)
			{
				pj = *pVector[fluidContainer->m_Neighbours[pi.m_GridId][j]];

				rDiff = pi.m_Position - pj.m_Position;

				// calculate color force
				scalar = 1.0f / pVector[j]->m_Density;

				color += pj.m_Mass * scalar * Kernels::getWPoly6(rDiff, radius);
				colorGrad += pj.m_Mass * scalar * Kernels::getWGradPoly6(rDiff, radius);
				colorLaplacian += pj.m_Mass * scalar * Kernels::getWLaplacePoly6(rDiff, radius);
			}
		}

		Vec2f n = colorGrad;							//  n
		float nLen = sqrt(n[0] * n[0] + n[1] * n[1]);	// |n|

		float kappa = -colorLaplacian / nLen;

		Vec2f surfaceForce = sigma * kappa * (n / nLen);

		// surfaceForce = Vec2f(0, 0);
		if (nLen > surfaceThreshold)
		{
			// apply force
			pVector[i]->m_Force += surfaceForce;
		}

		//fluidContainer->m_GridColors[x][y] = 0.2f;
	}
}

//-----------------------------------------------------------------------------------------------------------------//
void calculateRender(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float radius)
{

}
