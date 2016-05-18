#pragma once
#include "Particle.h"
#include "GLUtil.h"

class Force
{
public:
	Force();

	virtual void draw() = 0;
	virtual void apply() = 0;
};

