#include "Particle.h"
#include "IForce.h"
#include "FluidContainer.h"
#include "KernelFunctions.h"
#include "RigidBody.h"
#include "Box.h"
#include "Utility.h"

#include <vector>
#include <time.h>

#include <algorithm>

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)


void solveEuler(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt);
void solveLeapFrog(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt);
void solveMidpoint(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt);
void solveRungeKutta(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt);

void applyForces(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces);

void calculateFluidDynamics(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float kd);
void calculateColorField(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float radius);
void calculateRigidDynamics(std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces);

struct Pair
{
	RigidBody *A;
	RigidBody *B;
	//Vec2f contactP;
	float distance;
};

void simulation_step(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt, int method)
{
	switch (method)
	{
	case 0:
	default:
		solveEuler(pVector, fluidContainer, rigidBodies, forces, dt);
		break;
	case 1:
		solveMidpoint(pVector, fluidContainer, rigidBodies, forces, dt);
		break;
	case 2:
		solveLeapFrog(pVector, fluidContainer, rigidBodies, forces, dt);
		//solveRungeKutta(pVector, fluidContainer, forces, dt);
		break;
	}
}

//--------------------------------------------------------------
// Reset and apply forces to particles
//--------------------------------------------------------------
void applyForces(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces)
{
	// Reset forces
	for (int i = 0; i < pVector.size(); i++)
	{
		// reset force
		pVector[i]->m_Force = Vec2f(0, 0);
	}

	for (int i = 0; i < rigidBodies.size(); i++)
	{
		rigidBodies[i]->m_Force = Vec2f(0, 0);
		rigidBodies[i]->m_Torque = 0;
		rigidBodies[i]->m_ImpactPoints.clear();
	}

	calculateFluidDynamics(pVector, fluidContainer, 1.0f);
	calculateRigidDynamics(rigidBodies, forces);
	// Apply forces
	//int size = forces.size();
	for (int i = 0; i < forces.size(); i++)
	{
		// Apply force
		forces[i]->apply();
	}
}

bool SortPairs(Pair* lhs, Pair* rhs)
{
	if (lhs->A < rhs->A)
		return true;

	if (lhs->A == rhs->A)
		return lhs->B < rhs->B;

	return false;
}

//--------------------------------------------------------------
// Calculate rigid body values
//--------------------------------------------------------------
void calculateRigidDynamics(std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces)
{
	//Broad phase collision finding
	vector<Pair*> pairs = vector<Pair*>();
	vector<Pair*> uniquePairs = vector<Pair*>();

	for (int i = 0; i != rigidBodies.size(); i++)
	{
		for (int j = 0; j != rigidBodies.size(); j++)
		{
			RigidBody *A = rigidBodies[i];
			RigidBody *B = rigidBodies[j];

			// Continue on self check
			if (A == B || (A->m_isFixed && B->m_isFixed))
				continue;

			//Vec2f contact = A->BroadPhase(B);
			//if (contact[0] != 0 && contact[1] != 0)
			//{
			Pair* p = new Pair();
			p->A = A;
			p->B = B;
			//p->contactP = contact;
			pairs.push_back(p);
			//}
		}
	}
	sort(pairs.begin(), pairs.end(), SortPairs);

	//Get unique pairs
	/*{
		int i = 0;
		while (i < pairs.size())
		{
			Pair *pair = pairs[i];
			uniquePairs.push_back(pair);

			++i;

			while (i < pairs.size())
			{
				Pair *potential_dup = pairs[i];
				if (pair->A != potential_dup->B || pair->B != potential_dup->A)
					break;
				++i;
			}
		}
	}*/
	//uniquePairs = pairs;
	//pairs = vector<Pair*>();
	//Narrow phase collisions
	for (int i = 0; i < pairs.size(); i++)
	{
		Vec2f newforce = pairs[i]->B->CollisionCheck(pairs[i]->A, pairs[i]->B->m_Velocity);
		//uniquePairs[i]->A->m_Force -= newforce * 0.5f; 
		pairs[i]->B->m_Force += newforce;
		pairs[i]->A->m_Force -= newforce;
		pairs[i]->B->m_Velocity = 0;

		Vec2f relativeVelo = pairs[i]->B->m_Velocity - pairs[i]->A->m_Velocity;

		/*Vec2f normal = Vec2f(0, 0);
		int size = pairs[i]->A->m_Vertices.size();
		float lowestDist = 1.0f;
		for (int j = 0; j < size; j++)
		{
			Vec2f currentV = pairs[i]->A->m_Vertices[j % size]->m_Position + pairs[i]->A->m_Position;
			Vec2f nextV = pairs[i]->A->m_Vertices[(j + 1) % size]->m_Position + pairs[i]->A->m_Position;
			Vec2f currentNorm = Vec2f((nextV[1] - currentV[1]), -(nextV[0] - currentV[0]));

			float dist = abs(((nextV[1] - currentV[1])*pairs[i]->contactP[0]) - ((nextV[0] - currentV[0]) * pairs[i]->contactP[1]) + nextV[0] * currentV[1] - nextV[1] * currentV[0]);
			dist = dist / sqrt(pow(nextV[1] - currentV[1], 2) + pow(nextV[0] - currentV[0], 2));
			if (dist < lowestDist)
			{
				lowestDist = dist;
				normal = currentNorm;
			}
		}*/
		Vec2f normal = Util::normalise(newforce);
		Vec2f t = relativeVelo - ((relativeVelo * normal) * normal);
		t = Util::normalise(t);
		// Calculate relative velocity in terms of the normal direction
		float velAlongNormal = (relativeVelo * normal);

		// Do not resolve if velocities are separating
		if (velAlongNormal > 0)
			return;

		// Calculate restitution
		float e = 1;

		// Calculate impulse scalar
		float j = -(1 + e) * velAlongNormal;
		j /= 1 / pairs[i]->A->m_Mass
			+ 1 / pairs[i]->B->m_Mass;
		// Apply impulse
		Vec2f impulse = j * normal;
		pairs[i]->A->m_Force -= 1 / pairs[i]->A->m_Mass * impulse;
		pairs[i]->B->m_Force += 1 / pairs[i]->B->m_Mass * impulse;
		pairs[i]->A->m_Position -= normal * pairs[i]->distance;
		pairs[i]->A->m_Position += normal * pairs[i]->distance;

		j = -(1 + e) * (relativeVelo * t);
		j /= 1 / pairs[i]->A->m_Mass
			+ 1 / pairs[i]->B->m_Mass
			+ (pow(Util::crossProduct(pairs[i]->A->getRadiusVec(pairs[i]->contactP), t), 2) / pairs[i]->A->m_Inertia)
			+ (pow(Util::crossProduct(pairs[i]->B->getRadiusVec(pairs[i]->contactP), t), 2) / pairs[i]->B->m_Inertia);


		pairs[i]->A->m_Force += 1 / pairs[i]->A->m_Mass * impulse;
		pairs[i]->B->m_Force += 1 / pairs[i]->B->m_Mass * impulse;
		pairs[i]->A->m_Torque += 1 / pairs[i]->A->m_Inertia * Util::crossProduct(pairs[i]->A->getRadiusVec(pairs[i]->contactP), impulse);
		pairs[i]->B->m_Torque += 1 / pairs[i]->B->m_Inertia * Util::crossProduct(pairs[i]->B->getRadiusVec(pairs[i]->contactP), impulse);

	}
	//for (int i = 0; i < uniquePairs.size(); i++)
	//{
	//	RigidBody *A = uniquePairs[i]->A;
	//	RigidBody *B = uniquePairs[i]->B;

	//	// Continue on self check
	//	if (A == B)
	//		continue;

	//	float lowestDist = 2.00f;
	//	Vec2f collisionPoint = Vec2f(0, 0);
	//	/*for (int i = 0; i < A->m_Vertices.size(); i++)
	//	{
	//		for (int j = 0; j < B->m_Vertices.size(); j++)
	//		{
	//			Vec2f currentV = A->m_Vertices[i %  A->m_Vertices.size()]->m_Position + A->m_Position;
	//			Vec2f nextV = A->m_Vertices[(i + 1) % A->m_Vertices.size()]->m_Position + A->m_Position;
	//			Vec2f currentNorm = Vec2f((nextV[1] - currentV[1]), -(nextV[0] - currentV[0]));
	//			currentNorm = Util::normalise(currentNorm);
	//			float dist = abs(((nextV[1] - currentV[1])* (B->m_Vertices[j]->m_Position[0] + B->m_Position[0])) - ((nextV[0] - currentV[0])*(B->m_Vertices[j]->m_Position[1] + B->m_Position[1])) + nextV[0] * currentV[1] - nextV[1] * currentV[0]);
	//			dist = dist / sqrt(pow(nextV[1] - currentV[1], 2) + pow(nextV[0] - currentV[0], 2));
	//			if (dist < lowestDist && ((nextV[0] - currentV[0])*((B->m_Vertices[j]->m_Position[1] + B->m_Position[1]) - currentV[1]) - (nextV[1] - currentV[1])*((B->m_Vertices[j]->m_Position[0] + B->m_Position[0]) - currentV[0])) > 0)
	//			{
	//				lowestDist = dist;
	//				Vec2f alterdist = (currentNorm * dist);
	//				collisionPoint = B->m_Vertices[j]->m_Position + B->m_Position + alterdist;
	//				B->m_ImpactPoints.push_back(B->m_Vertices[j]->m_Position);
	//			}
	//		}
	//	}*/
	//	for (int i = 0; i < B->m_Vertices.size(); i++)
	//	{
	//		bool isInside = true;
	//		for (int j = 0; j < A->m_Vertices.size(); j++)
	//		{
	//			Vec2f currentV = A->m_Vertices[j]->m_Position + A->m_Position;
	//			Vec2f nextV = A->m_Vertices[(j + 1) % A->m_Vertices.size()]->m_Position + A->m_Position;
	//			//Vec2f currentNorm = Vec2f((nextV[1] - currentV[1]), -(nextV[0] - currentV[0]));
	//			//currentNorm = Util::normalise(currentNorm);
	//			//float dist = abs(((A->m_NormalPositions[i][1]) * (B->m_Vertices[j]->m_Position[0] + B->m_Position[0]))
	//			//	- ((nextV[0] - currentV[0])*(B->m_Vertices[j]->m_Position[1] + B->m_Position[1]))
	//			//	+ nextV[0] * currentV[1] - nextV[1] * currentV[0]);
	//			//dist = dist / sqrt(pow(nextV[1] - currentV[1], 2) + pow(nextV[0] - currentV[0], 2));
	//			float distance = Util::distancePointToLine(currentV, nextV, (B->m_Vertices[i]->m_Position + B->m_Position));
	//			if (distance >= 0)//(dist < lowestDist && ((nextV[0] - currentV[0])*((B->m_Vertices[j]->m_Position[1] + B->m_Position[1]) - currentV[1]) - (nextV[1] - currentV[1])*((B->m_Vertices[j]->m_Position[0] + B->m_Position[0]) - currentV[0])) > 0)
	//			{
	//				isInside = false;
	//				break;
	//				//lowestDist = dist;
	//				//Vec2f alterdist = (A->m_Normals[i] * distance);
	//			}
	//		}
	//		if (isInside)
	//		{
	//			collisionPoint = B->m_Vertices[i]->m_Position + B->m_Position;// +alterdist;
	//			B->m_ImpactPoints.push_back(B->m_Vertices[i]->m_Position);
	//		}

	//	}

	//	if (collisionPoint[0] != 0 && collisionPoint[1] != 0)
	//	{
	//		Pair* p = new Pair();
	//		p->A = A;
	//		p->B = B;
	//		p->contactP = collisionPoint;
	//		p->distance = lowestDist;
	//		pairs.push_back(p);
	//	}
	//}
	//sort(pairs.begin(), pairs.end(), SortPairs);

	//return;
	////for all Collisions
	//for (int i = 0; i < pairs.size(); i++)
	//{

	//	//Collision with wall object
	//	if (pairs[i]->B->m_isFixed)
	//	{
	//		if (pairs[i]->A->m_isFixed)
	//			continue;

	//		float bounceFactor = 0.65f;

	//		Vec2f n = Vec2f(0, 0);
	//		int size = pairs[i]->B->m_Vertices.size();
	//		float lowestDist = 1.0f;
	//		for (int j = 0; j < size; j++)
	//		{
	//			Vec2f currentV = pairs[i]->B->m_Vertices[j % size]->m_Position + pairs[i]->B->m_Position;
	//			Vec2f nextV = pairs[i]->B->m_Vertices[(j + 1) % size]->m_Position + pairs[i]->B->m_Position;
	//			Vec2f currentNorm = Vec2f((nextV[1] - currentV[1]), -(nextV[0] - currentV[0]));

	//			float dist = abs(((nextV[1] - currentV[1])*pairs[i]->contactP[0]) - ((nextV[0] - currentV[0]) * pairs[i]->contactP[1]) + nextV[0] * currentV[1] - nextV[1] * currentV[0]);
	//			dist = dist / sqrt(pow(nextV[1] - currentV[1], 2) + pow(nextV[0] - currentV[0], 2));
	//			if (dist < lowestDist)
	//			{
	//				lowestDist = dist;
	//				n = currentNorm;
	//			}
	//		}
	//		n = Util::normalise(n);
	//		Vec2f Vn = (n * pairs[i]->A->m_Velocity) * n;
	//		Vec2f Vt = pairs[i]->A->m_Velocity - Vn;
	//		pairs[i]->A->m_Velocity = Vt - bounceFactor * Vn;
	//		pairs[i]->A->m_Position += n * pairs[i]->distance;
	//		continue;
	//	}
	//	else if (pairs[i]->A->m_isFixed)
	//	{
	//		float bounceFactor = 0.65f;

	//		Vec2f n = Vec2f(0, 0);
	//		int size = pairs[i]->A->m_Vertices.size();
	//		float lowestDist = 1.0f;
	//		for (int j = 0; j < size; j++)
	//		{
	//			Vec2f currentV = pairs[i]->A->m_Vertices[j % size]->m_Position + pairs[i]->A->m_Position;
	//			Vec2f nextV = pairs[i]->A->m_Vertices[(j + 1) % size]->m_Position + pairs[i]->A->m_Position;
	//			Vec2f currentNorm = Vec2f((nextV[1] - currentV[1]), -(nextV[0] - currentV[0]));

	//			float dist = abs(((nextV[1] - currentV[1])*pairs[i]->contactP[0]) - ((nextV[0] - currentV[0]) * pairs[i]->contactP[1]) + nextV[0] * currentV[1] - nextV[1] * currentV[0]);
	//			dist = dist / sqrt(pow(nextV[1] - currentV[1], 2) + pow(nextV[0] - currentV[0], 2));
	//			if (dist < lowestDist)
	//			{
	//				lowestDist = dist;
	//				n = currentNorm;
	//			}
	//		}
	//		n = Util::normalise(n);
	//		Vec2f Vn = (n * pairs[i]->B->m_Velocity) * n;
	//		Vec2f Vt = pairs[i]->B->m_Velocity - Vn;
	//		pairs[i]->B->m_Velocity = Vt - bounceFactor * Vn;
	//		pairs[i]->B->m_Position += n * pairs[i]->distance;
	//		continue;
	//	}

	//	//Resolve Collision

	//	Vec2f relativeVelo = pairs[i]->B->m_Velocity - pairs[i]->A->m_Velocity;

	//	Vec2f normal = Vec2f(0, 0);
	//	int size = pairs[i]->A->m_Vertices.size();
	//	float lowestDist = 1.0f;
	//	for (int j = 0; j < size; j++)
	//	{
	//		Vec2f currentV = pairs[i]->A->m_Vertices[j % size]->m_Position + pairs[i]->A->m_Position;
	//		Vec2f nextV = pairs[i]->A->m_Vertices[(j + 1) % size]->m_Position + pairs[i]->A->m_Position;
	//		Vec2f currentNorm = Vec2f((nextV[1] - currentV[1]), -(nextV[0] - currentV[0]));

	//		float dist = abs(((nextV[1] - currentV[1])*pairs[i]->contactP[0]) - ((nextV[0] - currentV[0]) * pairs[i]->contactP[1]) + nextV[0] * currentV[1] - nextV[1] * currentV[0]);
	//		dist = dist / sqrt(pow(nextV[1] - currentV[1], 2) + pow(nextV[0] - currentV[0], 2));
	//		if (dist < lowestDist)
	//		{
	//			lowestDist = dist;
	//			normal = currentNorm;
	//		}
	//	}
	//	normal = Util::normalise(normal);
	//	Vec2f t = relativeVelo - ((relativeVelo * normal) * normal);
	//	t = Util::normalise(t);
	//	// Calculate relative velocity in terms of the normal direction
	//	float velAlongNormal = (relativeVelo * normal);

	//	// Do not resolve if velocities are separating
	//	if (velAlongNormal > 0)
	//		return;

	//	// Calculate restitution
	//	float e = 1;

	//	// Calculate impulse scalar
	//	float j = -(1 + e) * velAlongNormal;
	//	j /= 1 / pairs[i]->A->m_Mass
	//		+ 1 / pairs[i]->B->m_Mass;
	//	// Apply impulse
	//	Vec2f impulse = j * normal;
	//	pairs[i]->A->m_Force -= 1 / pairs[i]->A->m_Mass * impulse;
	//	pairs[i]->B->m_Force += 1 / pairs[i]->B->m_Mass * impulse;
	//	pairs[i]->A->m_Position -= normal * pairs[i]->distance;
	//	pairs[i]->A->m_Position += normal * pairs[i]->distance;

	//	j = -(1 + e) * (relativeVelo * t);
	//	j /= 1 / pairs[i]->A->m_Mass
	//		+ 1 / pairs[i]->B->m_Mass
	//		+ (pow(Util::crossProduct(pairs[i]->A->getRadiusVec(pairs[i]->contactP), t), 2) / pairs[i]->A->m_Inertia)
	//		+ (pow(Util::crossProduct(pairs[i]->B->getRadiusVec(pairs[i]->contactP), t), 2) / pairs[i]->B->m_Inertia);


	//	pairs[i]->A->m_Force += 1 / pairs[i]->A->m_Mass * impulse;
	//	pairs[i]->B->m_Force += 1 / pairs[i]->B->m_Mass * impulse;
	//	pairs[i]->A->m_Torque += 1 / pairs[i]->A->m_Inertia * Util::crossProduct(pairs[i]->A->getRadiusVec(pairs[i]->contactP), impulse);
	//	pairs[i]->B->m_Torque += 1 / pairs[i]->B->m_Inertia * Util::crossProduct(pairs[i]->B->getRadiusVec(pairs[i]->contactP), impulse);

	//}
}


//--------------------------------------------------------------
// Calculate fluid values
//--------------------------------------------------------------
void calculateFluidDynamics(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float kd)
{
	if (pVector.size() < 1)
		return;
	float radius = pVector[0]->m_Radius;
	float dist = 0.0f;
	kd = 0.006f;					// Stiffness (higher = less compressable)	
	float mu = 0.5f;				// Viscosity Coefficient (lower = thicker fluids)
	float restDensity = 120.0f;

	//float dist = 0.0f;
	//kd = 0.0045f;					// Stiffness (higher = less compressable)	
	//float mu = 0.056f;				// Viscosity Coefficient (lower = thicker fluids)
	//float restDensity = 203.0f;

	// Update the spatial hashing grid 
	//const clock_t begin_time = clock();
	fluidContainer->UpdateGrid(pVector);
	//std::cout << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;


	// Calculate particle densities, pressures and quantities
	for (int i = 0; i < pVector.size(); i++)
	{
		// TODO: iterating over cells might be faster since the neighbours will be the same for all particles within a cell
		// reset force
		//pVector[i]->m_Force = Vec2f(0, 0);

		pVector[i]->m_Density = 0;		// Rho_j
		pVector[i]->m_Quantity = 1;		// A_j
		pVector[i]->m_Pressure = 0;

		radius = pVector[i]->m_Radius;

		//vector<int> neighbours = fluidContainer->FindNeighbours(pVector[i]->m_GridId);

		int j = 0;

		//for (int j = 0; j < pVector.size(); j++)
		//{
		if (pVector[i]->m_GridId != -1)
		{
			for (int k = 0; k < fluidContainer->m_Neighbours[pVector[i]->m_GridId].size(); k++)
			{
				j = fluidContainer->m_Neighbours[pVector[i]->m_GridId][k];

				// m * W(|r-r_j|,h)
				dist = pVector[j]->distTo(pVector[i]->m_Position);

				//pVector[i]->m_Density += pVector[j]->m_Mass * pVector[j]->getW(dist);

				//if(pVector[j]->m_isFixed)	// Decrease influence radius for solid objects
				//	pVector[i]->m_Density += pVector[j]->m_Mass * Kernels::getWPoly6(pVector[i]->m_Position - pVector[j]->m_Position, radius * 0.1);
				//else
				pVector[i]->m_Density += pVector[j]->m_Mass * Kernels::getWPoly6(pVector[i]->m_Position - pVector[j]->m_Position, radius);
			}
		}
		// P_i = k(rho_i - restDensity_i)
		//pVector[i]->m_Pressure = kd * (pow(pVector[i]->m_Density / restDensity, 7) -1.0f);
		pVector[i]->m_Pressure = kd * (pVector[i]->m_Density - restDensity);

	}

	Vec2f pressureForce = 0;
	Vec2f viscocityForce = 0;
	float scalar = 0;

	dist = 0;

	float color = 0;

	// Calculate pressure force
	for (int i = 0; i < pVector.size(); i++)
	{
		pressureForce = 0;
		viscocityForce = 0;

		pVector[i]->m_Color = 0;

		radius = pVector[i]->m_Radius;

		//vector<int> neighbours = fluidContainer->FindNeighbours(pVector[i]->m_GridId);
		int j = 0;
		Vec2f vscalar;
		scalar = 0;

		if (pVector[i]->m_GridId != -1)
		{
			for (int k = 0; k < fluidContainer->m_Neighbours[pVector[i]->m_GridId].size(); k++)
			{
				j = fluidContainer->m_Neighbours[pVector[i]->m_GridId][k];
				// m_j * (p_i + p_j / 2 * rho_j) * WGrad(|r-r_j|,h)
				dist = pVector[j]->distTo(pVector[i]->m_Position);

				Vec2f posDiff = pVector[i]->m_Position - pVector[j]->m_Position;
				// calculate pressure force
				scalar = (pVector[i]->m_Pressure + pVector[j]->m_Pressure) / (2.0f * pVector[j]->m_Density);
				pressureForce += pVector[j]->m_Mass * scalar * Kernels::getWGradSpiky(posDiff, radius);




				// calculate viscocity force


				vscalar = (pVector[j]->m_Velocity - pVector[i]->m_Velocity) / (pVector[j]->m_Density);
				/*	if (pVector[j]->m_isFixed)
				vscalar *= 0;*/

				viscocityForce += pVector[j]->m_Mass * vscalar * Kernels::getWViscosityLaplace(posDiff, radius);



			}
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
	//// TODO: Use color field to identify surfaces
	//
	//float color = 0.0f;
	//Vec2f colorGrad = Vec2f(0, 0);
	//float colorLaplacian = 0.0f;

	//float sigma = 0.01f;

	//float surfaceThreshold = 0.0f;

	//Vec2f rDiff = Vec2f(0, 0);
	//float scalar = 0.0f;



	////// for each cell in fluidcontainer.
	////for (int x = 0; x < fluidContainer->m_GridRows; x++)
	////{
	////	for (int y = 0; y < fluidContainer->m_GridCols; y++)
	////	{
	//for (int i = 0; i < pVector.size(); i++)
	//{

	//	color = 0.0f;
	//	colorGrad = Vec2f(0, 0);
	//	colorLaplacian = 0.0f;

	//	surfaceThreshold = 0.1f;

	//	rDiff = Vec2f(0, 0);
	//	scalar = 0.0f;

	//		
	//	int j = 0;

	//	vector<int> neighbours = fluidContainer->FindNeighbours(pVector[i]->m_GridId);

	//	// Calculate surface tension
	//	for (int k = 0; k < fluidContainer->m_Neighbours[pVector[i]->m_GridId].size(); k++)
	//	{
	//		j = fluidContainer->m_Neighbours[pVector[i]->m_GridId][k];

	//		rDiff = pVector[i]->m_Position - pVector[j]->m_Position;

	//		// calculate color force
	//		scalar = 1.0f / pVector[j]->m_Density;

	//		color += pVector[j]->m_Mass * scalar * Kernels::getWPoly6(rDiff, radius);
	//		colorGrad += pVector[j]->m_Mass * scalar * Kernels::getWGradPoly6(rDiff, radius);
	//		colorLaplacian += pVector[j]->m_Mass * scalar * Kernels::getWLaplacePoly6(rDiff, radius);
	//	}

	//	Vec2f n = colorGrad;							//  n
	//	float nLen = sqrt(n[0] * n[0] + n[1] * n[1]);	// |n|

	//	float kappa = -colorLaplacian / nLen;

	//	Vec2f surfaceForce = sigma * kappa * (n / nLen);

	//	// surfaceForce = Vec2f(0, 0);
	//	if (nLen > surfaceThreshold)
	//	{
	//		// apply force
	//		pVector[i]->m_Force += surfaceForce;
	//	}

	//	//fluidContainer->m_GridColors[x][y] = 0.2f;
	//}
}

//--------------------------------------------------------------
// Solve using Euler's scheme
//--------------------------------------------------------------
void solveEuler(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt)
{
	applyForces(pVector, fluidContainer, rigidBodies, forces);

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
	// Loop rigid Bodies
	for (int i = 0; i < rigidBodies.size(); i++)
	{
		if (rigidBodies[i]->m_isFixed)
			continue;
		rigidBodies[i]->m_Velocity += rigidBodies[i]->m_Force * (1.0f / rigidBodies[i]->m_Mass) * dt;
		rigidBodies[i]->m_AngularVelocity += rigidBodies[i]->m_Torque * (1.0f / rigidBodies[i]->m_Inertia) * dt;
		rigidBodies[i]->m_Position += rigidBodies[i]->m_Velocity * dt;
		rigidBodies[i]->m_Orientation += rigidBodies[i]->m_AngularVelocity * dt;
		rigidBodies[i]->updateRotation(rigidBodies[i]->m_Orientation);

		for (int j = 0; j < rigidBodies[i]->m_Vertices.size(); j++)
		{
			matrix currentVertex = matrix(2, 1);
			currentVertex.setValue(0, 0, rigidBodies[i]->m_Vertices[j]->m_Position[0]);
			currentVertex.setValue(1, 0, rigidBodies[i]->m_Vertices[j]->m_Position[1]);
			matrix result = *rigidBodies[i]->m_Rotation * currentVertex;
			rigidBodies[i]->m_Vertices[j]->m_Position = Vec2f(result.getValue(0, 0), result.getValue(1, 0));
		}
		rigidBodies[i]->updateGhostParticles();
	}
}

//--------------------------------------------------------------
// Solve using the Leapfrog scheme
//--------------------------------------------------------------
void solveLeapFrog(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt)
{
	//std::vector<Vec2f> startPositions = std::vector<Vec2f>();
	//std::vector<Vec2f> startVelocities = std::vector<Vec2f>();

	//for (int i = 0; i < pVector.size(); i++)
	//{
	//	// Store starting positions
	//	startPositions.push_back(pVector[i]->m_Position);
	//	startVelocities.push_back(pVector[i]->m_Velocity);
	//}

	applyForces(pVector, fluidContainer, rigidBodies, forces);

	for (int i = 0; i < pVector.size(); i++)
	{
		// Update velocity
		pVector[i]->m_Velocity -= 0.5f * (pVector[i]->m_Force / pVector[i]->m_Mass) * dt;
	}

	applyForces(pVector, fluidContainer, rigidBodies, forces);

	for (int i = 0; i < pVector.size(); i++)
	{
		// Update velocity
		pVector[i]->m_Velocity += (pVector[i]->m_Force / pVector[i]->m_Mass) * dt;

		pVector[i]->m_Position += dt * pVector[i]->m_Velocity;
	}
}

//--------------------------------------------------------------
// Solve using the midpoint scheme
//--------------------------------------------------------------
void solveMidpoint(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt)
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
	applyForces(pVector, fluidContainer, rigidBodies, forces);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces);

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
void solveRungeKutta(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt)
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
	applyForces(pVector, fluidContainer, rigidBodies, forces);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces);

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