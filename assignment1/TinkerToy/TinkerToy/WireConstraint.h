#pragma once
#include "IConstraint.h"
#include <vector>

class WireConstraint : public IConstraint
{
private:
	Particle* const m_p;
	Vec2f const m_pos1;
	Vec2f const m_pos2;

public:
	WireConstraint(Particle* p, const Vec2f & pos1, const Vec2f & pos2);

	void draw() override;
	float getC() override;
	float getCd() override;
	vector<Vec2f> getJ() override;
	vector<Vec2f> getJd() override;

	vector<Particle*> getParticles() override;

};

