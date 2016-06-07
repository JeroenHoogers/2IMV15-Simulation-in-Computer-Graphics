#pragma once

#include "IForce.h"

class DragForce : public IForce
{
public:
	DragForce(Particle* p);

	void draw() override;
	void apply() override;

private:
	Particle* const m_p;		// particle
	float const m_Drag;			// drag coefficient
};

