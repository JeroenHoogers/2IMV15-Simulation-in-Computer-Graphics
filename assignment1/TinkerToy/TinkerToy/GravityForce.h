#pragma once

#include "Force.h"

class GravityForce : public Force
{
public:
	GravityForce(Particle* particle);

	void draw() override;
	void apply() override;
private:
	Particle * const m_particle;	// particle
	Vec2f const m_Gravity;			// Gravity constant

};

