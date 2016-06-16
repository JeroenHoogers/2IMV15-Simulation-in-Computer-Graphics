#pragma once

#include "IForce.h"

class DragForce : public IForce
{
public:
	DragForce(IPhysicsObject* p);

	void draw() override;
	void apply() override;

private:
	IPhysicsObject* const m_p;		// Physical object
	float const m_Drag;			// drag coefficient
};

