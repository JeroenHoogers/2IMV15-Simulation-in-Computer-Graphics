#include "ConstraintSolver.h"

using namespace std;

ConstraintSolver::ConstraintSolver()
{
}


void ConstraintSolver::Solve(std::vector<Particle*> pVector, std::vector<IConstraint*> constraints, float ks, float kd)
{
	// ks = strength
	// kd = damping

	if (constraints.empty())
		return;



	// Initialize particle matrices and vectors
	int n = pVector.size();
	int n2 = n * 2;

	vector<float> q = vector<float>(n2);			// Positions
	vector<float> qd = vector<float>(n2);			// Velocities
	vector<float> Q = vector<float>(n2);			// Force accumulators
	vector<float> M = vector<float>(n2 * n2);		// Mass
	vector<float> W = vector<float>(n2 * n2);		// M-inverse

	// Fill matrices
	for (int i = 0; i < n; i++)
	{
		int xy = i * 2;

		// Position and Velocity
		q[xy + 0] = pVector[i]->m_Position[0];
		q[xy + 1] = pVector[i]->m_Position[1];
		qd[xy + 0] = pVector[i]->m_Velocity[0];
		qd[xy + 1] = pVector[i]->m_Velocity[1];

		// Force accumulators
		Q[xy + 0] = pVector[i]->m_Force[0];
		Q[xy + 1] = pVector[i]->m_Force[1];

		// Mass
		M[n2 * (xy + 0) + (xy + 0)] = pVector[i]->m_Mass;
		M[n2 * (xy + 1) + (xy + 1)] = pVector[i]->m_Mass;
		W[n2 * (xy + 0) + (xy + 0)] = 1 / pVector[i]->m_Mass;
		W[n2 * (xy + 1) + (xy + 1)] = 1 / pVector[i]->m_Mass;
	}

	// Initialize constraint matrices and vectors
	int m = constraints.size();

	vector<float> C = vector<float>(m);												
	vector<float> Cd = vector<float>(m);											
	vector<vector<float>> J = vector<vector<float>>(m, vector<float>(n2));
	vector<vector<float>> Jd = vector<vector<float>>(m, vector<float>(n2));
	vector<vector<float>> JT = vector<vector<float>>(n2, vector<float>(m));

	// Fill matrices
	for (int i = 0; i < m; i++)
	{
		C[i] = constraints[i]->getC();
		Cd[i] = constraints[i]->getCd();

		vector<Vec2f> jacobian = constraints[i]->getJ();
		vector<Vec2f> jacobianDot = constraints[i]->getJd();

		// TODO: fix jacobian
	}

	// (JWJTLamba = -Jd * qd - JWQ -ks*C - kd*Cd)*
	vector<vector<float>> JW = vector<vector<float>>(m, vector<float>(n2));
	vector<vector<float>> JWJT = vector<vector<float>>(m, vector<float>(n2));


	

	
}