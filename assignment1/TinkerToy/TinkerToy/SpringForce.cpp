#include "SpringForce.h"
//#include <GL/glut.h>
#include "GLUtil.h"

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) : Force(),
  m_particle1(p1), m_particle2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::draw() 
{
  glBegin(GL_LINES);
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f(m_particle1->m_Position[0], m_particle1->m_Position[1]);
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f(m_particle2->m_Position[0], m_particle2->m_Position[1]);
  glEnd();
}

void SpringForce::apply()
{
	// calculate positional and velocity differences
	Vec2f positionDiff = m_particle1->m_Position - m_particle2->m_Position;
	Vec2f velocityDiff = m_particle1->m_Velocity - m_particle2->m_Velocity;
	
	// calculate distance
	float distance = norm(positionDiff);
	// calculate dotproduct
	float dotProduct = velocityDiff * positionDiff;

	// calculate result force
	Vec2f result = (m_ks * (distance - m_dist) + m_kd * (dotProduct / distance)) * (positionDiff / distance);

	// apply force to both particles
	m_particle1->m_Force -= result;
	m_particle2->m_Force += result;
}