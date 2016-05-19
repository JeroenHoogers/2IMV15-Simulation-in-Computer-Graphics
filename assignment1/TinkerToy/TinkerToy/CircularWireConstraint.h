#pragma once
#include "IForce.h"
#include "Particle.h"

class CircularWireConstraint : public IForce
{
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius);

  void draw();
  void apply();

 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
};
