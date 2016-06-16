#pragma once
#include "IForce.h"
#include <vector>

class MouseForce : public IForce
{
public:
	MouseForce(vector<Particle*> p, Vec2f mousePos, double dist, double ks, double kd);

	void draw() override;
	void apply() override;
	void newMousePosition(Vec2f mousePos);
	void selectParticles(vector<Particle*> particles);
	void clearParticle();

	bool selected;

private:
	vector<Particle*> m_particles;		// particles
	Vec2f m_mousePos;		// mouse position
	double const m_dist;     // rest length
	double const m_ks, m_kd; // spring strength constants
};
