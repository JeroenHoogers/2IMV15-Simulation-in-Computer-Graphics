#pragma once
#include <gfx\vec2.h>
#include <vector>
#include "Particle.h"
#include "matrix.h"
#include "IPhysicsObject.h"

using namespace std;

class RigidBody : public IPhysicsObject
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

	bool m_isFixed;


	vector<Particle*> m_Vertices;

	RigidBody(const Vec2f & ConstructPos, bool isFixed = false);
	
	~RigidBody();

	Vec2f getRadiusVec(Vec2f pos);
	float getMass() override;
	Vec2f getVelocity() override;
	void setVelocity(Vec2f) override;
	Vec2f getPosition() override;
	void setPosition(Vec2f) override;
	void addForce(Vec2f) override;

	void draw();
	void reset();
	Vec2f BroadPhase(RigidBody* other);
	virtual Vec2f NarrowPhase(RigidBody* other) = 0;
	virtual vector<float> getExtremes();

	void updateRotation(float angle);

	virtual void calculateInertia() = 0;



};

