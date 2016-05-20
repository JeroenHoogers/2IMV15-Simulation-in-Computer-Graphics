#pragma once
#include <vector>

#include "Particle.h"
#include "GLUtil.h"

class IConstraint
{
public:

	virtual void draw() = 0;

	virtual float getC() = 0;
	virtual float getCd() = 0;

	virtual float getJ() = 0;
	virtual float getJd() = 0;
};

