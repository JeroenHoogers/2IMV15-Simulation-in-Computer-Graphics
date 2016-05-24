#include "PointConstraint.h"
//#include <GL/glut.h>
#include "GLUtil.h"

PointConstraint::PointConstraint(Particle* p, const Vec2f & pos) :
	m_p(p), m_pos(pos) 
{

}

void PointConstraint::draw()
{
	glBegin(GL_LINES);
	glColor3f(0.8, 0.7, 0.6);
//	glVertex2f(m_p1->m_Position[0], m_p1->m_Position[1]);
	glColor3f(0.8, 0.7, 0.6);
	//glVertex2f(m_p2->m_Position[0], m_p2->m_Position[1]);
	glEnd();

}

vector<Particle*> PointConstraint::getParticles()
{
	vector<Particle*> particles;

	particles.push_back(m_p);

	return particles;
}

float PointConstraint::getC()
{
	// C(x1, y1, x2, y2) = Pow(x1 - x2) + Pow(y1-y2)
	// Idea: Rod / Circular wire constraint with distance = 0
	// calculate difference in position between the particle and the fixed point
	Vec2f positionDiff = m_p->m_Position - m_pos ;

	float C = (positionDiff * positionDiff);

	return C;

}

float PointConstraint::getCd()
{
	// C(x1, y1, x2, y2) = 2 * (x1 - x) + 2 * (y1-y)
	Vec2f positionDiff = (m_p->m_Position - m_pos) * 2.0f;
	Vec2f velocityDiff = (m_p->m_Velocity) * 2.0f;

	// Take the dot product of both differences
	float Cd = positionDiff * velocityDiff;

	return Cd;

}

vector<Vec2f> PointConstraint::getJ()
{
	vector<Vec2f> J;

	Vec2f positionDiff = (m_p->m_Position - m_pos) *  2.0f;
	J.push_back(positionDiff);

	return J;
}

vector<Vec2f> PointConstraint::getJd()
{
	vector<Vec2f> Jd;

	Vec2f velocityDiff = (m_p->m_Velocity) * 2.0f;
	Jd.push_back(velocityDiff);

	return Jd;
}
