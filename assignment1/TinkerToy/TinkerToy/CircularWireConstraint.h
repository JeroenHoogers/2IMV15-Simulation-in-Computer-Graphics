#pragma once
#include "IConstraint.h"
#include "Particle.h"

class CircularWireConstraint : public IConstraint
{
 public:
	CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

	void draw() override;
	float getC() override;
	float getCd() override;
	vector<Vec2f> getJ() override;
	vector<Vec2f> getJd() override;

	vector<Particle*> getParticles() override;

 private:

	Particle * const m_p;
	Vec2f const m_center;
	double const m_radius;
};
