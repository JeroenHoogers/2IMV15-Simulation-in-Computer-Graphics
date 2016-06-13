#pragma once

#include "IForce.h"

class WallForce : public IForce
{
public:
	WallForce(Particle* p);

	void draw() override;
	void apply() override;

private:
	Particle* const m_p;		// particle
	Vec2f const m_Gravity;		// gravity force
};
