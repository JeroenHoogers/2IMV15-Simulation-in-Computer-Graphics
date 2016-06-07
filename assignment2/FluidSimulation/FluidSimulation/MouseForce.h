#pragma once
#include "IForce.h"
#include <vector>

class MouseForce : public IForce
{
public:
	MouseForce(Particle* p, Vec2f mousePos, double dist, double ks, double kd);

	void draw() override;
	void apply() override;
	void newMousePosition(Vec2f mousePos);
	void selectParticle(Particle * p);
	void clearParticle();

	bool selected;

private:
	Particle* m_p;		// particles
	Vec2f m_mousePos;		// mouse position
	double const m_dist;     // rest length
	double const m_ks, m_kd; // spring strength constants
};
