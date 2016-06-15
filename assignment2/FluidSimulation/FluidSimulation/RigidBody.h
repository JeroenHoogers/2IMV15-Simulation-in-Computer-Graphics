#pragma once
#include <gfx\vec2.h>
#include <vector>
#include "Particle.h"
#include "matrix.h"

using namespace std;

class RigidBody
{
private:


public:

	float m_Mass;
	float m_Inertia;

	Vec2f m_Force;

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	float m_Acceleration; //??
	matrix* m_Rotation;

	
	float m_Orientation;	// angle in radians
	float m_AngularVelocity; // 
	float m_Torque;



	vector<Particle*> m_Vertices;

	RigidBody(const Vec2f & ConstructPos);
	
	~RigidBody();

	void draw();
	void reset();

	void updateRotation(float angle);

	virtual void calculateInertia() = 0;



};

