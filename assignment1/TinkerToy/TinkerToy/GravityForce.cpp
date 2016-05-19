#include "GravityForce.h"

GravityForce::GravityForce(Particle* p) :
	m_p(p), m_Gravity(Vec2f(0, -0.000981)) 
{
}

void GravityForce::draw()
{
	Vec2f direction = m_Gravity;
	direction = (direction / norm(direction)) * 0.2f;
	glBegin(GL_LINES);
	glColor3f(0.6, 0.7, 0.4);
	glVertex2f(m_p->m_Position[0], m_p->m_Position[1]);
	glColor3f(0.6, 0.7, 0.4);
	glVertex2f(m_p->m_Position[0] + direction[0], m_p->m_Position[1] + direction[1]);
	glEnd();
}

void GravityForce::apply()
{
	// TODO: check if mass is implemented correctly
	m_p->m_Force += m_Gravity * m_p->m_Mass;
}
