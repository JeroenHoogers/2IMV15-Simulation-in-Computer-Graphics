#pragma once
#include "RigidBody.h"

class Box : public RigidBody
{
private:


public:
	float m_Width;
	float m_Height;

	Box(const Vec2f ConstPosition, float density, float width, float height, bool isFixed = false);
	void calculateInertia() override;
	vector<float> getExtremes() override;
	Vec2f NarrowPhase(RigidBody* other) override;
	~Box();
};