#pragma once
#include "RigidBody.h"

class Box : public RigidBody
{
private:


public:
	float m_Width;
	float m_Height;

	Box(const Vec2f ConstPosition, float mass, float height, float width);
	void calculateInertia() override;
	~Box();
};

