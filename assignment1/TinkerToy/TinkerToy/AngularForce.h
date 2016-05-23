#pragma once

#include "Particle.h"
#include "IForce.h"

class AngularForce : public IForce {
public:
	AngularForce(Particle *p1, Particle * p2, Particle * p3, double angle, double ks, double kd);

	void draw() override;
	void apply() override;

private:

	Particle * const m_p1;   // particle 1
	Particle * const m_p2;   // particle 2 
	Particle * const m_p3;   // particle 3 
	double const m_angle;     // rest length
	double const m_ks, m_kd; // spring strength constants
};
