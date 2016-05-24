#include "WallConstraint.h"
//#include <GL/glut.h>
#include "GLUtil.h"

WallConstraint::WallConstraint(Particle *p1, Vec2f min, Vec2f max, double dist) :
	m_p1(p1), m_min(min), m_max(max), m_dist(dist) {}

void WallConstraint::draw()
{
	glBegin(GL_LINES);
	glColor3f(0.8, 0.7, 0.6);
	glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
	glEnd();

}

float WallConstraint::getC()
{
	// C(x1, y1, x2, y2) = Pow(x1 - x2) + Pow(y1-y2) - Pow(dist)

	// calculate difference in position between particles
	Vec2f positionDiff = m_p1->m_Position - (m_min - m_max);

	float C = (positionDiff * positionDiff);  // -(m_dist * m_dist);

	return C;
}

float WallConstraint::getCd()
{
	// C(x1, y1, x2, y2) = 2 * (x1 - x2) + 2 * (y1-y2)
	Vec2f positionDiff = 2.0f * (m_p1->m_Position - (m_min - m_max));
	Vec2f velocityDiff = 2.0f * (m_p1->m_Velocity);

	// Take the dot product of both differences
	float Cd = positionDiff * velocityDiff;

	return Cd;
}

vector<Vec2f> WallConstraint::getJ()
{
	vector<Vec2f> J;

	Vec2f positionDiff = 2.0f * (m_p1->m_Position - (m_min - m_max));
	J.push_back(positionDiff);
	J.push_back(-positionDiff);

	return J;
}

vector<Vec2f> WallConstraint::getJd()
{
	vector<Vec2f> Jd;

	Vec2f velocityDiff = 2.0f * (m_p1->m_Velocity - (m_min - m_max));
	Jd.push_back(velocityDiff);
	Jd.push_back(-velocityDiff);

	return Jd;
}

vector<Particle*> WallConstraint::getParticles()
{
	vector<Particle*> particles;

	particles.push_back(m_p1);

	return particles;
}