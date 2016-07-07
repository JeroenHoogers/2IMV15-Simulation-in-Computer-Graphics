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

void applyForces(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt);

extern void calculateFluidDynamics(std::vector<Particle*> pVector, FluidContainer* fluidContainer, vector<RigidBody*> rigidBodies);
//void calculateColorField(std::vector<Particle*> pVector, FluidContainer* fluidContainer, float radius);
void calculateRigidDynamics(std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt);

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
void applyForces(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt)
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
		rigidBodies[i]->m_Orientation = 0;
		rigidBodies[i]->m_ImpactPoints.clear();
	}

	//time_t begin_time = clock();
	calculateFluidDynamics(pVector, fluidContainer, rigidBodies);
	//std::cout << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;
	
	calculateRigidDynamics(rigidBodies, forces, dt);
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
void calculateRigidDynamics(std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt)
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

			Vec2f contact = A->BroadPhase(B);
			if (contact[0] != 0 && contact[1] != 0)
			{
				Pair* p = new Pair();
				p->A = A;
				p->B = B;
				pairs.push_back(p);
			}
		}
	}
	sort(pairs.begin(), pairs.end(), SortPairs);

	Box boxA = Box(Vec2f(0,0), 0, 0, 0);
	Box boxB = Box(Vec2f(0, 0), 0, 0, 0);
	for (int i = 0; i < pairs.size(); i++)
	{
		boxA = *(Box*)pairs[i]->A;
		boxB = *(Box*)pairs[i]->B;

		//Check for actual Collision
		Vec2f newforce = pairs[i]->B->CollisionCheck(pairs[i]->A, boxB.m_Velocity, dt);
		Vec2f normal = Util::normalise(newforce);

		//Only if the resulting force of both bodies would be 0 exit the loop.
		if ((boxA.m_Force[0] + newforce[0] == 0 && boxA.m_Force[1] + newforce[1] == 0)
			&& (boxB.m_Force[0] + newforce[0] == 0 && boxB.m_Force[1] + newforce[1] == 0))
			continue;

		//Determine contact points from both bodies.
		vector<Vec2f> contactPoints = pairs[i]->B->findImpactPoint(pairs[i]->A);
		vector<Vec2f> contactPointsB = pairs[i]->A->findImpactPoint(pairs[i]->B);
		contactPoints.insert(contactPoints.end(), contactPointsB.begin(), contactPointsB.end());

		//Apply linear forces to the bodies
		pairs[i]->A->m_Force -= newforce;
		pairs[i]->B->m_Force += newforce;

		//Correct positions to avoid penetration
		//Determine Y-axis penetration depth
		float minA = 0; float minB = 0; float maxA = 0; float maxB = 0;
		vector<float> resultA = pairs[i]->A->Project(Vec2f(0, 1), minA, maxA);
		vector<float> resultB = pairs[i]->B->Project(Vec2f(0, 1), minB, maxB);
		minA = resultA[0];
		maxA = resultA[1];
		minB = resultB[0];
		maxB = resultB[1];
		float penetrationY = abs(pairs[i]->A->DistInterval(minA, maxA, minB, maxB));

		//Determine X-axis penetration depth
		resultA = pairs[i]->A->Project(Vec2f(1, 0), minA, maxA);
		resultB = pairs[i]->B->Project(Vec2f(1, 0), minB, maxB);
		minA = resultA[0];
		maxA = resultA[1];
		minB = resultB[0];
		maxB = resultB[1];
		float penetrationX = abs(pairs[i]->A->DistInterval(minA, maxA, minB, maxB));

		//Get the least penetration
		float penetration = min(penetrationX, penetrationY);

		//Correct actual penetration
		const float perct = 0.1; // correction percentage
		const float allowpen = 0.01; // allowed penetration, to avoid stuttering at rest.
		float massInverseA = (1 / boxA.m_Mass);
		float massInverseB = (1 / boxB.m_Mass);
		Vec2f corr = max(penetration - allowpen, 0.0f) / (massInverseA + massInverseB) * perct * normal;
		if (boxA.m_isFixed)
		{
			pairs[i]->B->m_Position += 2.0f * (massInverseB * corr);
		}
		else if (boxB.m_isFixed)
		{
			pairs[i]->A->m_Position -= 2.0f * (massInverseA * corr);
		}
		else
		{
			pairs[i]->A->m_Position -= massInverseA * corr;
			pairs[i]->B->m_Position += massInverseB * corr;
		}

		//Loop all contact points to determine angular rotations
		for (int k = 0; k < contactPoints.size(); k++)
		{
			Vec2f rA = contactPoints[k] - boxA.m_Position;
			Vec2f rB = contactPoints[k] - boxB.m_Position;

			Vec2f pointVelA = boxA.m_Velocity + Util::crossProduct(boxA.m_AngularVelocity, rA);
			Vec2f pointVelB = boxB.m_Velocity + Util::crossProduct(boxB.m_AngularVelocity, rB);

			Vec2f relativeVelocity = pointVelB - pointVelA;
			Vec2f t = relativeVelocity - ((relativeVelocity * normal) * normal);
			t = Util::normalise(t);

			//Relative velocity in normal direction
			float velAlongNormal = (relativeVelocity * normal);

			// Do not resolve if velocities are separating
			if (velAlongNormal > 0)
				return;

			// Calculate restitution
			float e = 1;

			// Calculate impulse scalar
			float j = -(1 + e) * velAlongNormal;
			j /= 1 / boxA.m_Mass
				+ 1 / boxB.m_Mass;

			j += newforce*newforce;

			// Apply impulse
			Vec2f impulse = j * normal;

			float momentOfInertiaA = boxA.m_Mass * (rA * rA);
			float momentOfInertiaB = boxB.m_Mass * (rB * rB);
			if (!boxB.m_isFixed)
			{
				pairs[i]->B->m_Force += impulse;
				pairs[i]->B->m_Torque += (1.0f / momentOfInertiaB) * Util::crossProduct(contactPoints[k] - boxB.m_Position, impulse);
			}
			else if (!boxA.m_isFixed)
			{
				pairs[i]->A->m_Force -= impulse;
				pairs[i]->A->m_Torque -= ((1.0f / momentOfInertiaA) * Util::crossProduct(contactPoints[k] - boxA.m_Position, impulse));
			}
		}
	}
}

//--------------------------------------------------------------
// Solve using Euler's scheme
//--------------------------------------------------------------
void solveEuler(std::vector<Particle*> pVector, FluidContainer* fluidContainer, std::vector<RigidBody*> rigidBodies, std::vector<IForce*> forces, float dt)
{
	applyForces(pVector, fluidContainer, rigidBodies, forces, dt);

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
		{
			continue;
		}
		rigidBodies[i]->m_Velocity += rigidBodies[i]->m_Force * (1.0f / rigidBodies[i]->m_Mass) * dt;
  		rigidBodies[i]->m_AngularVelocity += rigidBodies[i]->m_Torque  * (1.0f / rigidBodies[i]->m_Inertia) * dt;
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
		//Drag force
		rigidBodies[i]->m_AngularVelocity *= 0.999f;
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

	applyForces(pVector, fluidContainer, rigidBodies, forces, dt);

	for (int i = 0; i < pVector.size(); i++)
	{
		// Update velocity
		pVector[i]->m_Velocity -= 0.5f * (pVector[i]->m_Force / pVector[i]->m_Mass) * dt;
	}

	applyForces(pVector, fluidContainer, rigidBodies, forces, dt);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces, dt);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces, dt);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces, dt);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces, dt);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces, dt);

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
	applyForces(pVector, fluidContainer, rigidBodies, forces, dt);

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