#pragma once

#include "IForce.h"

class GravityForce : public IForce
{
public:
	GravityForce(Particle* particle);

	void draw() override;
	void apply() override;
private:
	Particle * const m_particle;	// particle
	Vec2f const m_Gravity;			// Gravity constant

};

