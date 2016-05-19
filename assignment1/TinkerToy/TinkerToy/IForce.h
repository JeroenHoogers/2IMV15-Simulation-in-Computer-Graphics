#pragma once
#include "Particle.h"
#include "GLUtil.h"

class IForce
{
public:

	virtual void draw() = 0;
	virtual void apply() = 0;
};

