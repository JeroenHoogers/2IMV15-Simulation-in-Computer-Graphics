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
		W.setValue(i2, i2, 1 / pVector[i]->m_Mass);
		W.setValue(i2 + 1, i2 + 1, 1 / pVector[i]->m_Mass);
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
		vector<Vec2f> jacobianD = constraints[i]->getJd();
		//int bla = jacobiand.size();
		vector<Particle*> particles = constraints[i]->getParticles();

		// Calculate Jacobian matrices
		for (int j = 0; j < particles.size(); j++)
		{

			for (int dim = 0; dim < 2; dim++)
			{
				int x = i;
				int y = particles[j]->m_index * 2 + dim;
				//double hoi = jacobian[j][dim];
				//double hoid = jacobiand[j][dim];
				cout << "jac: " << jacobian[j][dim] << endl;
				cout << "jacD: " << jacobianD[j][dim] << endl;

				J.setValue(i, particles[j]->m_index * 2 + dim, jacobian[j][dim]);
				Jd.setValue(i, particles[j]->m_index * 2 + dim, jacobianD[j][dim]);

				// Transpose of J
				JT.setValue(particles[j]->m_index * 2 + dim, i, jacobian[j][dim]);
			}
		}
	}
	
	matrix JW = J * W;
	matrix JWJT = JW * JT;

	// JWJTLamba = -Jd * qd - JWQ -ks*C - kd*Cd 
	//matrix Jdqd = matrix(n2, 1);
	//matrix JWQ = matrix(n2, 1);
	matrix Jdqd = Jd * qd;
	Jdqd = Jdqd * -1.0;
	matrix JWQ = JW * Q;


	cout << endl << "Q:" << endl;
	Q.printMatrix();

	cout << endl << "J:" << endl;
	J.printMatrix();

	cout << endl << "Jd:" << endl;
	Jd.printMatrix();

	cout << endl << "JT:" << endl;
	JT.printMatrix();


	cout << endl << "JW:" << endl;
	JW.printMatrix();

	cout << endl << "Jdqd:" << endl;
	Jdqd.printMatrix();

	cout << endl << "JWQ:" << endl;
	JWQ.printMatrix();

	//matrix Cks = matrix(m, 1);
	//matrix Cdkd = matrix(m, 1);
	matrix Cks = C * ks;
	matrix Cdkd = Cd * kd;

	matrix JWJTLambda = Jdqd - JWQ - Cks - Cdkd;
	
	// Obtain lambda using the conjugate gradient solver
	double* lambda = new double[m];
	for (int i = 0; i < m; i++)
	{
		lambda[i] = 0;
	}

	//int size = JWJTLambda.m_cols * JWJTLambda.m_rows;
	//double* jwjtlam = new double[size];
	//for (unsigned int i = 0; i < size; i++)
	//{
	//	jwjtlam[i] = JWJTLambda.getValue(i / JWJTLambda.m_rows, i % JWJTLambda.m_rows);
	//}
	
	int stepSize = 100;
	ConjGrad(m, &JWJT, lambda, JWJTLambda.m_data[0], 1.0 / 100.0, &stepSize);

	// convert lambda to a 1 column matrix (vector)
	double lam = lambda[0];
	matrix lambdaMat = matrix(m, 1);
	lambdaMat.setData(lambda);

	// Calculate the resulting force using lambda Qh = lambda * JT
	matrix Qh = JT * lambdaMat;

	// Finally apply the constraint forces to the particles
	for (int i = 0; i < n; i++)
	{
		Vec2f asdfsadf = Vec2f(Qh.getValue(i * 2, 1),
			Qh.getValue(i * 2 + 1, 1));
		pVector[i]->m_Force += Vec2f( Qh.getValue(i * 2, 1), 
									  Qh.getValue(i * 2 + 1, 1));
	}
	
	free(lambda);
}