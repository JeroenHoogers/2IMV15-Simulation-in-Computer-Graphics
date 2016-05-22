#pragma once
#include <vector>
#include "Particle.h"
#include "IConstraint.h"

class ConstraintSolver
{
public:
	ConstraintSolver();

	void Solve(std::vector<Particle*> pVector, std::vector<IConstraint*> constraints, float ks, float kd);
};

