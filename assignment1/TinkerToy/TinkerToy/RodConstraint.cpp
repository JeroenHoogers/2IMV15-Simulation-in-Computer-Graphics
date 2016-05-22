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
	// C(x1, y1, x2, y2) = Pow(x1 - x2) + Pow(y1-y2) - Pow(dist)

	// calculate difference in position between particles
	Vec2f positionDiff = m_p1->m_Position - m_p2->m_Position;

	float C = (positionDiff * positionDiff) - (m_dist * m_dist);

	return C;
}

float RodConstraint::getCd()
{
	// C(x1, y1, x2, y2) = 2 * (x1 - x2) + 2 * (y1-y2)
	Vec2f positionDiff = 2.0f * (m_p1->m_Position - m_p2->m_Position);
	Vec2f velocityDiff = 2.0f * (m_p1->m_Velocity - m_p2->m_Velocity);

	// Take the dot product of both differences
	float Cd = positionDiff * velocityDiff;

	return Cd;
}

std::vector<Vec2f> RodConstraint::getJ() 
{
	std::vector<Vec2f> J;

	Vec2f positionDiff = 2.0f * (m_p1->m_Position - m_p2->m_Position);
	J.push_back(positionDiff);
	J.push_back(-positionDiff);
	
	return J;
}

std::vector<Vec2f> RodConstraint::getJd()
{
	std::vector<Vec2f> Jd;

	Vec2f velocityDiff = 2.0f * (m_p1->m_Velocity - m_p2->m_Velocity);
	Jd.push_back(velocityDiff);
	Jd.push_back(-velocityDiff);

	return Jd;
}