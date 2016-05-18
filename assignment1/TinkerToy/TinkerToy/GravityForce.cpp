#include "GravityForce.h"

GravityForce::GravityForce(Particle* particle) :
	m_particle(particle), m_Gravity(Vec2f(0, -9.81)) {}

void GravityForce::draw()
{
	glBegin(GL_LINES);
	glColor3f(0.6, 0.7, 0.4);
	glVertex2f(m_particle->m_Position[0], m_particle->m_Position[1]);
	glColor3f(0.6, 0.7, 0.4);
	glVertex2f(m_particle->m_Position[0] + m_Gravity[0], m_particle->m_Position[1] + m_Gravity[1]);
	glEnd();
}

void GravityForce::apply()
{
	m_particle->m_Force += m_particle->m_Mass * m_Gravity;
}
