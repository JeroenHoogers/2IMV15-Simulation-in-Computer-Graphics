#include "SpringForce.h"
//#include <GL/glut.h>
#include "GLUtil.h"

SpringForce::SpringForce(Particle *particle1, Particle * particle2, double dist, double ks, double kd) : 
	m_particle1(particle1), m_particle2(particle2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::draw() 
{
  glBegin(GL_LINES);
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f(m_particle1->m_Position[0], m_particle1->m_Position[1]);
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f(m_particle2->m_Position[0], m_particle2->m_Position[1]);
  glEnd();

  // calculate positional and velocity differences
  Vec2f positionDiff = m_particle1->m_Position - m_particle2->m_Position;
  Vec2f velocityDiff = m_particle1->m_Velocity - m_particle2->m_Velocity;
  velocityDiff = -(velocityDiff / norm(velocityDiff)) * 0.3f;


  glBegin(GL_LINES);
  glColor3f(0.1, 0.7, 0.8);
  glVertex2f(m_particle2->m_Position[0], m_particle2->m_Position[1]);
  glColor3f(0.1, 0.7, 0.8);
  glVertex2f(m_particle2->m_Position[0] + velocityDiff [0], m_particle2->m_Position[1] + velocityDiff[1]);

  glEnd();
}

void SpringForce::apply()
{
	// calculate positional and velocity differences
	Vec2f positionDiff = m_particle1->m_Position - m_particle2->m_Position;
	Vec2f velocityDiff = m_particle1->m_Velocity - m_particle2->m_Velocity;
	
	// calculate distance
	//float distance = norm(positionDiff);
	float distance = sqrt(positionDiff[0] * positionDiff[0] + positionDiff[1] * positionDiff[1]);
	
	// calculate dotproduct
	float dotProduct = velocityDiff * positionDiff;


	// calculate result force
	Vec2f result = (m_ks * (distance - m_dist) + m_kd * (dotProduct / distance));
	result = result * (positionDiff / distance);

	// apply force to both particles
	m_particle1->m_Force -= result;
	m_particle2->m_Force += result;
}