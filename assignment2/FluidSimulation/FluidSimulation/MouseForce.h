#pragma once
#include "IForce.h"
#include "RigidBody.h"
#include <vector>

class MouseForce : public IForce
{
public:
	MouseForce(vector<Particle*> p, Vec2f mousePos, double dist, double ks, double kd);

	void draw() override;
	void apply() override;
	void newMousePosition(Vec2f mousePos);
	void selectParticles(vector<Particle*> particles);
	void clearParticles();

	void selectRigidbodies(vector<RigidBody*> rigidbodies);
	void clearRigidbodies();

	bool leftMouseDown;
	bool rightMouseDown;

private:
	vector<Particle*> m_particles;		// particles
	vector<RigidBody*> m_rigidbodies;		// rigid bodies
	Vec2f m_mousePos;		// mouse position
	double const m_dist;     // rest length
	double const m_ks, m_kd; // spring strength constants
};
