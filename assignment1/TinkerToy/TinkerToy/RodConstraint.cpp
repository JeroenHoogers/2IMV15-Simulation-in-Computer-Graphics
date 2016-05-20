#include "RodConstraint.h"
//#include <GL/glut.h>
#include "GLUtil.h"

RodConstraint::RodConstraint(Particle *p1, Particle * p2, double dist) :
  m_p1(p1), m_p2(p2), m_dist(dist) {}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}

float RodConstraint::getC()
{
	//C(x1, y1, x2, y2) = Pow(x1 - x2) + Pow(y1-y2) - Pow(dist)

	//Calculate difference between particles
	Vec2f difference = m_p1->m_Position - m_p2->m_Position;

	//Calculate C(x1, y1, x2, y2)
	float result = ((difference[0] * difference[0]) + (difference[1] * difference[1]) - (m_dist * m_dist));

	return result;
}

float RodConstraint::getCd()
{
	//C(x1, y1, x2, y2) = 2 * (x1 - x2) + 2 * (y1-y2)
	Vec2f positionDiff = 2.0f * (m_p1->m_Position - m_p2->m_Position);
	Vec2f velocityDiff = 2.0f * (m_p1->m_Velocity - m_p2->m_Velocity);

	float result = positionDiff[0] * velocityDiff[0] + positionDiff[1] * velocityDiff[1];

	return result;
}