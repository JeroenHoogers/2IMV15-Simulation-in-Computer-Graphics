#pragma once
#include <vector>
#include "Particle.h"
#include "IConstraint.h"
#include "linearSolver.h"
#include "matrix.h"

class ConstraintSolver
{
public:
	ConstraintSolver();

	static void Solve(std::vector<Particle*> pVector, std::vector<IConstraint*> constraints, float ks, float kd);
};

