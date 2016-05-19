#pragma once

#include "Particle.h"
#include "IForce.h"

class RodConstraint : public IForce {
 public:
  RodConstraint(Particle *p1, Particle * p2, double dist);

  void draw();
  void apply();

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  double const m_dist;
};
