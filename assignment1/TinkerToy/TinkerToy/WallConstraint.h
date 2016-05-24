#pragma once

#include "Particle.h"
#include "IConstraint.h"

	class WallConstraint : public IConstraint
	{
	public:
		WallConstraint(Particle *p1, Vec2f min, Vec2f max, double dist);

		void draw() override;

		float getC() override;
		float getCd() override;

		vector<Vec2f> getJ() override;
		vector<Vec2f> getJd() override;

		vector<Particle*> getParticles() override;

	private:

		Particle * const m_p1;
		Vec2f const m_min;
		Vec2f const m_max;
		double const m_dist;
	};
