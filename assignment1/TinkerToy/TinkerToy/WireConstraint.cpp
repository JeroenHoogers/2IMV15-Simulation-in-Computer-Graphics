#include "WireConstraint.h"


WireConstraint::WireConstraint(Particle* p, const Vec2f & pos1, const Vec2f & pos2) : 
	m_p(p), m_pos1(pos1), m_pos2(pos2) 
{

}

void WireConstraint::draw()
{
	glBegin(GL_LINES);
	glColor3f(0.8, 0.7, 0.6);
	glVertex2f(m_pos1[0], m_pos1[1]);
	glColor3f(0.8, 0.7, 0.6);
	glVertex2f(m_pos2[0], m_pos2[1]);
	glEnd();

}

float WireConstraint::getC()
{
	//C(x, y, x_c, y_c) = Pow(x - x_c) + Pow(y - y_c) - Pow(r)

	//Calculate difference vector between the particle and center of the wire circle
	Vec2f positionDiff = m_pos1 - m_pos2;

	//Calculate C(x, y, x_c, y_c)

	return -1.0f;
}

float WireConstraint::getCd()
{
	return -1.0f;
}

vector<Vec2f> WireConstraint::getJ()
{
	return vector<Vec2f>();
}

vector<Vec2f> WireConstraint::getJd()
{
	return vector<Vec2f>();
}

vector<Particle*> WireConstraint::getParticles()
{
	vector<Particle*> particles;
	particles.push_back(m_p);

 	return particles;
}