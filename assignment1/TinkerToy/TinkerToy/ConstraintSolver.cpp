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

	// Define sizes
	int n = pVector.size();
	int m = constraints.size();
	int n2 = n * 2;

	// Initialize particle matrices and vectors

	matrix q = matrix::matrix(n2, 1);			// Positions
	matrix qd = matrix::matrix(n2, 1);			// Velocities
	matrix Q = matrix::matrix(n2, 1);			// Force accumulators
	matrix M = matrix::matrix(n2, n2);			// Mass
	matrix W = matrix::matrix(n2, n2);			// M-inverse
	
	// Fill matrices / vectors
	for (int i = 0; i < n; i++)
	{
		int i2 = i * 2;

		// Set the particle index (to be used in the jacobian matrices)
		pVector[i]->m_index = i;

		// Position and Velocity
		q.setValue(i2, 0, pVector[i]->m_Position[0]);
		q.setValue(i2 + 1, 0, pVector[i]->m_Position[1]);
		qd.setValue(i2, 0, pVector[i]->m_Velocity[0]);
		qd.setValue(i2 + 1, 0, pVector[i]->m_Velocity[1]);

		// Force accumulators
		Q.setValue(i2, 0, pVector[i]->m_Force[0]);
		Q.setValue(i2 + 1, 0, pVector[i]->m_Force[1]);

		// Mass
		M.setValue(i2, i2, pVector[i]->m_Mass);
		M.setValue(i2 + 1, i2 + 1, pVector[i]->m_Mass);
		M.setValue(i2, i2, 1 / pVector[i]->m_Mass);
		M.setValue(i2 + 1, i2 + 1, 1 / pVector[i]->m_Mass);
	}

	// Initialize constraint matrices and vectors
	matrix C = matrix(m, 1);
	matrix Cd = matrix(m, 1);

	matrix J = matrix(m, n2);
	matrix Jd = matrix(m, n2);
	matrix JT = matrix(n2, m);

	// Fill matrices
	for (int i = 0; i < m; i++)
	{
		C.setValue(i, 0, constraints[i]->getC());
		Cd.setValue(i, 0, constraints[i]->getCd());

		vector<Vec2f> jacobian = constraints[i]->getJ();
		vector<Vec2f> jacobiand = constraints[i]->getJd();
		vector<Particle*> particles = constraints[i]->getParticles();

		// Calculate Jacobian matrices
		for (int j = 0; j < particles.size(); j++)
		{

			for (int dim = 0; dim < 2; dim++)
			{
				J.setValue(i, particles[j]->m_index * 2 + dim, jacobian[j][dim]);
				Jd.setValue(i, particles[j]->m_index * 2 + dim, jacobiand[j][dim]);

				// Transpose of J
				JT.setValue(particles[j]->m_index * 2 + dim, i, jacobian[j][dim]);
			}
		}
	}

	matrix JW = J * W;
	matrix JWJT = JW * JT;

	// JWJTLamba = -Jd * qd - JWQ -ks*C - kd*Cd 
	matrix Jdqd = Jd * qd;
	matrix JWQ = JW * Q;

	matrix Cks = C * ks;
	matrix Cdkd = Cd * kd;

	matrix JWJTLambda = Jdqd - JWQ - Cks - Cdkd;
	
	// Obtain lambda using the conjugate gradient solver
	double* lambda = new double[m];

	int stepSize = 100;
	ConjGrad(m, &JWJT, lambda, JWJTLambda.getData(), 1 / 1000, &stepSize);

	// convert lambda to a 1 column matrix (vector)
	matrix lambdaMat = matrix(m, 1);
	lambdaMat.setData(lambda);

	// Calculate the resulting force using lambda Qh = lambda * JT
	matrix Qh = JT * lambdaMat;

	// Finally apply the constraint forces to the particles
	for (int i = 0; i < n; i++)
	{
		pVector[i]->m_Force += Vec2f( Qh.getValue(i * 2, 1), 
									  Qh.getValue(i * 2 + 1, 1));
	}
	
	free(lambda);
}