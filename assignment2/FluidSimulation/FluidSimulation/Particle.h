#pragma once

#include <gfx/vec2.h>

using namespace std;

class Particle
{
public:

	Particle(const Vec2f & ConstructPos, float mass = 1.0f, float radius = 0.05f, bool isFixed = false);
	virtual ~Particle(void);

	void reset();
	void draw();

	//float getDensityAt(Vec2f p);
	float distTo(Vec2f p);
	float getW(float r);
	float getWGrad(float r);
	float getWLaplacian(float r);
	float getWVisLaplacian(float r);
	float getWPoly6(float r);
	Vec2f getWSpikyGrad(Vec2f r);

	int m_index;

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;

	float m_Density;
	float m_Quantity;
	float m_Pressure;

	// h
	float m_Radius;

	float m_Mass;
	Vec2f m_Force;

	bool m_isFixed;
};
