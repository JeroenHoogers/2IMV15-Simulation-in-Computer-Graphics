#pragma once

#include <gfx/vec2.h>
#include "IPhysicsObject.h"

using namespace std;

class Particle : public IPhysicsObject
{
public:

	Particle(const Vec2f & ConstructPos, float mass = 1.0f, float radius = 0.05f, bool isFixed = false, bool isBoundary = false);
	virtual ~Particle(void);

	float getMass() override;
	Vec2f getVelocity() override;
	void setVelocity(Vec2f) override;
	Vec2f getPosition() override;
	void setPosition(Vec2f) override;
	void addForce(Vec2f) override;

	void reset();
	void draw(bool renderFluid = false);

	float distTo(Vec2f p);

	int m_index;

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;

	Vec2f m_LocalPosition;

	float m_Density;
	float m_Quantity;
	float m_Pressure;

	// h
	float m_Radius;

	float m_Color;

	float m_Mass;
	Vec2f m_Force;

	bool m_isFixed;
	bool m_isBoundary;
	bool m_isActive;

	float m_Volume;

	int m_GridId;
};
