#pragma once

#include <gfx\vec2.h>

class IPhysicsObject
{
public:
	virtual float getMass() = 0;
	virtual Vec2f getVelocity() = 0;
	virtual void setVelocity(Vec2f) = 0;
	virtual Vec2f getPosition() = 0;
	virtual void setPosition(Vec2f) = 0;
	virtual void addForce(Vec2f) = 0;
};