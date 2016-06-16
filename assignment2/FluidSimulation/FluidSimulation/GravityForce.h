#pragma once

#include "IForce.h"

class GravityForce : public IForce
{
public:
	GravityForce(IPhysicsObject* p);

	void draw() override;
	void apply() override;

private:
	IPhysicsObject* const m_p;		// particle
	Vec2f const m_Gravity;		// gravity force
};