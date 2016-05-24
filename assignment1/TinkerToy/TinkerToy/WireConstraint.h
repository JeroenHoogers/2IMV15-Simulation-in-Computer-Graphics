#pragma once
#include "IConstraint.h"
#include <vector>

class WireConstraint : public IConstraint
{
private:
	Particle* const m_p;
	Vec2f const m_pos;
	int const m_axis;

public:
	WireConstraint(Particle* p, const Vec2f & pos, const int axis);

	void draw() override;
	float getC() override;
	float getCd() override;
	vector<Vec2f> getJ() override;
	vector<Vec2f> getJd() override;

	vector<Particle*> getParticles() override;

};

