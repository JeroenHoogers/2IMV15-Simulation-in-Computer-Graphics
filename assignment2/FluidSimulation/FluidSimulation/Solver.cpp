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

extern void calculateFluidDynamics(std::vector<Particle*> pVector, FluidContainer* fluidContainer);
//void calculateColorField(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float radius);
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

	//time_t begin_time = clock();
	calculateFluidDynamics(pVector, fluidContainer);
	//std::cout << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;
	
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

		//j = -(1 + e) * (relativeVelo * t);
		//j /= 1 / pairs[i]->A->m_Mass
		//	+ 1 / pairs[i]->B->m_Mass
		//	+ (pow(Util::crossProduct(pairs[i]->A->getRadiusVec(pairs[i]->contactP), t), 2) / pairs[i]->A->m_Inertia)
		//	+ (pow(Util::crossProduct(pairs[i]->B->getRadiusVec(pairs[i]->contactP), t), 2) / pairs[i]->B->m_Inertia);


		//pairs[i]->A->m_Force += 1 / pairs[i]->A->m_Mass * impulse;
		//pairs[i]->B->m_Force += 1 / pairs[i]->B->m_Mass * impulse;
		//pairs[i]->A->m_Torque += 1 / pairs[i]->A->m_Inertia * Util::crossProduct(pairs[i]->A->getRadiusVec(pairs[i]->contactP), impulse);
		//pairs[i]->B->m_Torque += 1 / pairs[i]->B->m_Inertia * Util::crossProduct(pairs[i]->B->getRadiusVec(pairs[i]->contactP), impulse);

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