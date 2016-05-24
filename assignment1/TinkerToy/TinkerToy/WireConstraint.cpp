#include "WireConstraint.h"


WireConstraint::WireConstraint(Particle* p, const Vec2f & pos) : 
	m_p(p), m_pos(pos)
{

}

void WireConstraint::draw()
{
	glBegin(GL_LINES);
	glColor3f(0.8, 0.7, 0.6);
	glVertex2f(-1, m_pos[1]);
	glColor3f(0.8, 0.7, 0.6);
	glVertex2f(1, m_pos[1]);
	glEnd();

}

vector<Particle*> WireConstraint::getParticles()
{
	vector<Particle*> particles;
	particles.push_back(m_p);

	return particles;
}

float WireConstraint::getC()
{
	//C(x, y) = (y - h)
	float C = m_p->m_Position[1] - m_pos[1];
	return C;
}

float WireConstraint::getCd()
{
	float Cd = m_p->m_Velocity[1];
	return Cd;
}

vector<Vec2f> WireConstraint::getJ()
{
	vector<Vec2f> J;

	J.push_back(Vec2f(0, 1));

	return J;
}

vector<Vec2f> WireConstraint::getJd()
{
	vector<Vec2f> Jd;

	Jd.push_back(Vec2f(0, 0));

	return Jd;
}

