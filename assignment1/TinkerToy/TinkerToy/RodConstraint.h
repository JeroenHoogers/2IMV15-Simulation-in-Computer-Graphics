#pragma once

#include "Particle.h"
#include "IConstraint.h"

class RodConstraint : public IConstraint 
{
public:
	RodConstraint(Particle *p1, Particle * p2, double dist);

	void draw() override;

	float getC() override;
	float getCd() override;

	vector<Vec2f> getJ() override;
	vector<Vec2f> getJd() override;

	vector<Particle*> getParticles() override;

private:

	Particle * const m_p1;
	Particle * const m_p2;
	double const m_dist;
};
