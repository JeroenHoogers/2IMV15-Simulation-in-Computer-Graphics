#pragma once

#include "Particle.h"
#include "IForce.h"

class SpringForce : public IForce {
public:
	SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd, bool draw = true);

	void draw() override;
	void apply() override;

private:

	Particle * const m_p1;   // particle 1
	Particle * const m_p2;   // particle 2 
	double const m_dist;     // rest length
	double const m_ks, m_kd; // spring strength constants
	bool m_drawEnabled; // enable drawing
};
