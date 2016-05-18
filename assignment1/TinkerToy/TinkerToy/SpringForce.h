#pragma once

#include "Particle.h"
#include "Force.h"

class SpringForce : public Force{
 public:
  SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd);

  void draw() override;
  void apply() override;

 private:

  Particle * const m_particle1;   // particle 1
  Particle * const m_particle2;   // particle 2 
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
