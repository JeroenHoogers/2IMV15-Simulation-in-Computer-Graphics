#include "WireConstraint.h"


WireConstraint::WireConstraint(Particle* p, const Vec2f & pos, const int axis) : 
	m_p(p), m_pos(pos), m_axis(axis)
{

}

void WireConstraint::draw()
{
	glBegin(GL_LINES);
	glColor3f(0.8, 0.7, 0.6);
	if (m_axis == 1)
	{
		glVertex2f(-1, m_pos[1]);
		glVertex2f(1, m_pos[1]);
	}
	else
	{
		glVertex2f(m_pos[0], -1);
		glVertex2f(m_pos[0], 1);
	}
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
	float C = m_p->m_Position[m_axis] - m_pos[m_axis];
	return C;
}

float WireConstraint::getCd()
{
	float Cd = m_p->m_Velocity[m_axis];
	return Cd;
}

vector<Vec2f> WireConstraint::getJ()
{
	vector<Vec2f> J;

	// x: Vec2f(1, 0)
	// y: Vec2d(0, 1)
	Vec2f axis = Vec2f(1 - m_axis, m_axis);
	
	J.push_back(axis);

	return J;
}

vector<Vec2f> WireConstraint::getJd()
{
	vector<Vec2f> Jd;

	Jd.push_back(Vec2f(0, 0));

	return Jd;
}

